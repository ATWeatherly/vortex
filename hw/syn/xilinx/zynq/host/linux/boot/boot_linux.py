#!/usr/bin/env python3
# One-shot Arty Z7 Vortex/Linux boot orchestrator (JTAG + UART, no network/SD-pull).
#
# Flow: reset PS -> catch U-Boot (Ctrl-C out of its BOOTP loop) -> JTAG-stage
# any requested files into DDR (xsdb PA writes) -> U-Boot fatwrite them onto
# the SD FAT partition -> set bootargs (mem=256M walls off the Vortex window,
# blacklist the stale PL driver) -> stage kernel FIT from SD -> JTAG-program
# the Vortex bitstream -> bootm -> login -> mount SD -> fix FCLK0 to 50 MHz
# via SLCR -> verify shim magic.
#
# Usage: boot_linux.py [--put host_path[:sd_name]] ... [--bit path]
# Files land on the SD FAT partition; Linux sees them under /mnt/sd.
import os
import subprocess
import sys
import termios
import time
import select

UART = os.environ.get("UART_DEV", "/dev/ttyUSB1")
XSDB = os.path.expanduser("~/xilinx/2025.2/Vivado/bin/xsdb")
DEFAULT_BIT = "/home/aweatherly8/projects/vortex/build/hw/syn/xilinx/zynq/vortex_z7.bit"
STAGE_ADDR = 0x08000000  # DDR staging during U-Boot (low RAM, U-Boot is idle)
BOOTARGS = "console=ttyPS0,115200 earlycon mem=256M modprobe.blacklist=smartio"


class Uart:
    def __init__(self):
        self.fd = os.open(UART, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = 0; a[1] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[3] = 0; a[4] = termios.B115200; a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)

    def drain(self, t=0.6):
        buf = b""
        end = time.time() + t
        while time.time() < end:
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                try:
                    buf += os.read(self.fd, 4096)
                except BlockingIOError:
                    pass
        return buf

    def sendl(self, s):
        for ch in s.encode():
            os.write(self.fd, bytes([ch]))
            time.sleep(0.004)
        os.write(self.fd, b"\n")

    def expect(self, pats, timeout_s, initial=b""):
        buf = initial
        end = time.time() + timeout_s
        while time.time() < end:
            buf += self.drain(0.5)
            for p in pats:
                if p in buf:
                    return p, buf
        return None, buf

    def ucmd(self, c, timeout_s=15):
        """U-Boot command with Ctrl-C preamble and prompt wait; retries once."""
        for _ in range(2):
            os.write(self.fd, b"\x03")
            time.sleep(0.2)
            self.drain(0.4)
            self.sendl(c)
            p, buf = self.expect([b"Zynq> "], timeout_s)
            if p:
                return buf.decode("utf-8", "replace")
        raise RuntimeError(f"U-Boot command failed: {c}\n{buf.decode('utf-8','replace')}")


def xsdb(script):
    r = subprocess.run([XSDB, "-eval", script], capture_output=True, text=True, timeout=600)
    return r.stdout + r.stderr


def main():
    puts = []
    bit = DEFAULT_BIT
    args = sys.argv[1:]
    while args:
        a = args.pop(0)
        if a == "--put":
            spec = args.pop(0)
            host, _, name = spec.partition(":")
            puts.append((host, name or os.path.basename(host)))
        elif a == "--bit":
            bit = args.pop(0)
        else:
            print(f"unknown arg {a}")
            return 1

    u = Uart()

    print("== resetting PS ==")
    print(xsdb('connect; targets -set -filter {name =~ "APU*"}; catch {stop}; rst -system; puts RST_OK').strip()[-40:])

    print("== waiting for U-Boot ==")
    p, _ = u.expect([b"autoboot", b"BOOTP", b"Zynq> "], 90)
    if p is None:
        print("no U-Boot detected")
        return 1
    time.sleep(1)
    u.ucmd("echo UBOOT_OK")

    for host, name in puts:
        sz = os.path.getsize(host)
        words = (sz + 3) // 4
        print(f"== staging {host} ({sz} B) -> SD:{name} ==")
        out = xsdb(
            'connect; targets -set -filter {name =~ "*Cortex-A9*#0"}; catch {stop}; '
            f'mwr -address-space PA -bin -file {host} {STAGE_ADDR:#x} {words}; '
            'catch {con}; puts STAGE_OK')
        if "STAGE_OK" not in out:
            print(out[-300:])
            return 1
        u.ucmd(f"fatwrite mmc 0 {STAGE_ADDR:#x} {name} {sz:#x}", 60)

    print("== configuring boot ==")
    u.ucmd("setenv sdbootdev 0")
    u.ucmd(f"setenv bootargs '{BOOTARGS}'")
    u.ucmd("run cp_kernel2ram", 40)

    print(f"== programming bitstream {bit} ==")
    out = xsdb('connect; targets -set -filter {name =~ "xc7z010"}; '
               f'fpga -file {bit}; puts BIT_OK')
    if "BIT_OK" not in out:
        print(out[-300:])
        return 1

    print("== booting ==")
    u.drain(0.5)
    u.sendl("bootm 0x10000000")
    p, _ = u.expect([b"login:"], 150)
    if p is None:
        print("no login prompt")
        return 1
    u.sendl("root"); time.sleep(2); u.drain(1)
    u.sendl("root"); time.sleep(3); u.drain(1)

    print("== post-boot setup ==")
    u.sendl("mkdir -p /mnt/sd && mount /dev/mmcblk0p1 /mnt/sd && echo MOUNT_OK")
    print(u.drain(4).decode("utf-8", "replace")[-120:])
    # FCLK0 -> 50 MHz (IOPLL 1000 MHz / (10*2)), then shim magic check
    u.sendl("devmem 0xF8000008 32 0xDF0D; devmem 0xF8000170 32 0x00200A00; devmem 0xF8000004 32 0x767B; devmem 0x43C0001C")
    out = u.drain(4).decode("utf-8", "replace")
    print(out[-120:])
    ok = "0x56585A37" in out
    print("SHIM:", "OK" if ok else "MISSING")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
