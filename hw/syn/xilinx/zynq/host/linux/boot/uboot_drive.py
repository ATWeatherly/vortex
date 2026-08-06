#!/usr/bin/env python3
# Drive the Arty Z7's U-Boot over the UART (/dev/ttyUSB1) without pyserial.
# Usage: uboot_drive.py <mode> [args...]
#   probe                  - catch autoboot, stop it, run recon commands, log all
#   cmd "<command>" ...    - assume prompt is live, run command(s), print output
#   boot <kernel> <initrd> <dtb>  - issue bootz with the given hex addresses
# The board must be reset externally (xsdb rst -system) right after this
# script starts in 'probe' mode; it waits for the autoboot banner.
import os
import sys
import termios
import time
import select

DEV = os.environ.get("UART_DEV", "/dev/ttyUSB1")
PROMPT = b"Zynq> "


class Uart:
    def __init__(self):
        self.fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        # raw 115200 8N1
        attrs[0] = 0                      # iflag
        attrs[1] = 0                      # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
        attrs[3] = 0                      # lflag
        attrs[4] = termios.B115200        # ispeed
        attrs[5] = termios.B115200        # ospeed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def read_until(self, patterns, timeout_s, log=True):
        """Read until any of `patterns` (bytes) appears or timeout. Returns
        (matched_pattern_or_None, buffer)."""
        buf = b""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                try:
                    chunk = os.read(self.fd, 4096)
                except BlockingIOError:
                    continue
                if chunk:
                    buf += chunk
                    if log:
                        sys.stdout.write(chunk.decode("utf-8", "replace"))
                        sys.stdout.flush()
                    for p in patterns:
                        if p in buf:
                            return p, buf
        return None, buf

    def send(self, data: bytes):
        os.write(self.fd, data)

    def sendline(self, s: str):
        # slow-feed to be kind to the UART FIFO
        for ch in s.encode():
            os.write(self.fd, bytes([ch]))
            time.sleep(0.002)
        os.write(self.fd, b"\n")

    def command(self, cmd, timeout_s=15):
        """Run one command at a live prompt, return its output text."""
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.sendline(cmd)
        _, buf = self.read_until([PROMPT], timeout_s)
        return buf.decode("utf-8", "replace")


def probe(u: Uart):
    print("== waiting for U-Boot autoboot banner (reset the board now) ==")
    pat, _ = u.read_until([b"Hit any key to stop autoboot", b"stop autoboot"], 120)
    if pat is None:
        print("\n== NO AUTOBOOT BANNER SEEN ==")
        return 1
    # mash a key repeatedly to catch the window
    for _ in range(20):
        u.send(b"\r")
        time.sleep(0.05)
    pat, _ = u.read_until([PROMPT], 10)
    if pat is None:
        print("\n== NO PROMPT AFTER KEYPRESS ==")
        return 1
    print("\n== GOT U-BOOT PROMPT; running recon ==")
    for cmd in ["version", "bdinfo", "printenv", "help bootz",
                "help bootm", "help booti", "fatls mmc 0", "mmc info"]:
        print(f"\n===== {cmd} =====")
        u.command(cmd, 20)
    print("\n== PROBE DONE (leaving U-Boot at prompt) ==")
    return 0


def main():
    u = Uart()
    mode = sys.argv[1] if len(sys.argv) > 1 else "probe"
    if mode == "probe":
        return probe(u)
    if mode == "cmd":
        for c in sys.argv[2:]:
            print(u.command(c, 30))
        return 0
    if mode == "boot":
        kernel, initrd, dtb = sys.argv[2:5]
        u.sendline(f"bootz {kernel} {initrd} {dtb}")
        # stream boot output for a while
        u.read_until([b"login:", b"# ", b"Kernel panic"], 180)
        return 0
    print(f"unknown mode {mode}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
