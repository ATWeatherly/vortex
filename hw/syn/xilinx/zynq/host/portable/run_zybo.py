#!/usr/bin/env python3
"""
run_zybo.py — cross-platform Vortex-on-Zybo-Z7-20 orchestrator.

Boots Linux on the board over JTAG and runs a command on it. Works on Windows,
Linux and macOS; replaces the Linux-only run_linux_zybo.py.

Everything it needs is a prebuilt binary shipped in this bundle — no Vortex
build tree, no cross-compiler, no dtc. The only host requirements are Vivado's
xsdb (for JTAG) and pyserial (for the UART).

    python run_zybo.py --list                      # check wiring, don't boot
    python run_zybo.py                             # boot + run the default tests
    python run_zybo.py --cmd "uname -a"            # boot + run one command
    python run_zybo.py --skip-jtag --cmd "ls /tmp" # board already booted

Layout expected (all paths relative to this file; override with flags):

    run_zybo.py
    uartlib.py
    jtag_boot_linux.tcl
    bit/vortex_z7.bit
    bit/ps7_init.tcl
    boot/u-boot.elf
    boot/image_slim.ub
    payload/memcp
    payload/payload.tar
"""
import argparse
import os
import platform
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from uartlib import Uart, find_port  # noqa: E402

# DDR staging addresses. Both sit above the kernel's mem= ceiling so Linux
# never allocates over them before we extract.
FIT_ADDR = "0x10000000"
PAYLOAD_ADDR = "0x20000000"

DEFAULT_TESTS = (
    "cd /tmp && "
    "echo '== T1 vecadd n=16 ==' && ./vecadd_cp_arm -k vecadd_kernel.vxbin -n 16; "
    "echo '== T2 vecadd n=4096 ==' && ./vecadd_cp_arm -k vecadd_kernel.vxbin -n 4096; "
    "echo '== T3 llama2 -v 1 ==' && "
    "./llama2_perf stories260K.bin -z tok512.bin -t 0 -n 40 -v 1; "
    "echo '== T4 llama2 -v 0 ==' && "
    "./llama2_perf stories260K.bin -z tok512.bin -t 0 -n 40 -v 0"
)


# --------------------------------------------------------------- xsdb ------
def find_xsdb(explicit=None):
    """Locate xsdb. On Windows it is xsdb.bat and must be run via the shell."""
    if explicit:
        if not os.path.exists(explicit):
            sys.exit(f"--xsdb path does not exist: {explicit}")
        return explicit

    env = os.environ.get("XSDB")
    if env and os.path.exists(env):
        return env

    # already on PATH?
    for name in ("xsdb", "xsdb.bat", "xsdb.cmd"):
        found = shutil.which(name)
        if found:
            return found

    # common install roots
    win = platform.system() == "Windows"
    roots = []
    if win:
        roots += [r"C:\Xilinx\Vivado", r"C:\Xilinx\Vitis",
                  r"C:\Program Files\Xilinx\Vivado"]
    else:
        roots += [os.path.expanduser("~/xilinx"), "/opt/Xilinx/Vivado",
                  "/tools/Xilinx/Vivado"]
    exe = "xsdb.bat" if win else "xsdb"
    for root in roots:
        if not os.path.isdir(root):
            continue
        # <root>/<version>/[Vivado/]bin/xsdb
        for dirpath, dirnames, filenames in os.walk(root):
            if exe in filenames and os.path.basename(dirpath) == "bin":
                return os.path.join(dirpath, exe)
            # don't descend forever into a big install tree
            if dirpath.count(os.sep) - root.count(os.sep) > 3:
                dirnames[:] = []

    sys.exit(
        "could not find xsdb. Install Vivado/Vitis, then either add its bin\n"
        "directory to PATH or pass --xsdb <path to xsdb.bat>.\n"
        r"  typical: C:\Xilinx\Vivado\2025.2\bin\xsdb.bat"
    )


def run_xsdb(xsdb, args, timeout=900):
    """Invoke xsdb. .bat needs shell=True on Windows; a plain exe does not."""
    cmd = [xsdb] + args
    use_shell = platform.system() == "Windows" and xsdb.lower().endswith((".bat", ".cmd"))
    printable = " ".join(f'"{c}"' if " " in c else c for c in cmd)
    print("+", printable, flush=True)
    if use_shell:
        return subprocess.run(printable, shell=True, timeout=timeout)
    return subprocess.run(cmd, timeout=timeout)


# ------------------------------------------------------------- payload -----
def deliver_memcp(u, memcp_path):
    """Push the memcp binary onto the board as printf octal escapes.

    The board's busybox has no base64 applet, and /dev/mem cannot be read with
    read()/dd for non-System-RAM — so memcp (which mmaps it) is the only way to
    recover the JTAG-staged payload, and it has to arrive over the UART.
    """
    with open(memcp_path, "rb") as f:
        data = f.read()
    print(f"== delivering memcp ({len(data)} B) over UART ==", flush=True)
    u.shell_cmd("rm -f /tmp/memcp")
    chunk = 48
    total = (len(data) + chunk - 1) // chunk
    for i in range(0, len(data), chunk):
        esc = "".join(f"\\{b:03o}" for b in data[i:i + chunk])
        u.sendline(f"printf '{esc}' >> /tmp/memcp", char_delay=0)
        time.sleep(0.05)  # let the shell keep up
        n = i // chunk + 1
        if n % 100 == 0 or n == total:
            print(f"   {n}/{total} lines", flush=True)
    time.sleep(1)
    u.clear()
    print(u.shell_cmd(
        "chmod +x /tmp/memcp && ls -la /tmp/memcp && md5sum /tmp/memcp", 30),
        flush=True)


# ---------------------------------------------------------------- main -----
def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__)
    ap.add_argument("--list", action="store_true",
                    help="show serial ports + located xsdb, then exit")
    ap.add_argument("--skip-jtag", action="store_true",
                    help="board is already booted; just run the command")
    ap.add_argument("--cmd", default=None,
                    help="shell command to run on the board (default: the test set)")
    ap.add_argument("--cmd-timeout", type=int, default=480)
    ap.add_argument("--mem", default="240M",
                    help="Linux mem= spec; the CP host pool lives at 0x0F000000")
    ap.add_argument("--xsdb", default=None, help="path to xsdb / xsdb.bat")
    ap.add_argument("--port", default=None, help="serial port (e.g. COM4)")
    ap.add_argument("--bit", default=os.path.join(HERE, "bit", "vortex_z7.bit"))
    ap.add_argument("--ps7", default=os.path.join(HERE, "bit", "ps7_init.tcl"))
    ap.add_argument("--uboot", default=os.path.join(HERE, "boot", "u-boot.elf"))
    ap.add_argument("--fit", default=os.path.join(HERE, "boot", "image_slim.ub"))
    ap.add_argument("--payload", default=os.path.join(HERE, "payload", "payload.tar"))
    ap.add_argument("--memcp", default=os.path.join(HERE, "payload", "memcp"))
    ap.add_argument("--tcl", default=os.path.join(HERE, "jtag_boot_linux.tcl"))
    args = ap.parse_args()

    if args.list:
        import serial.tools.list_ports as lp
        print("serial ports:")
        for p in lp.comports():
            vid = f"{p.vid:04x}" if p.vid else "----"
            pid = f"{p.pid:04x}" if p.pid else "----"
            print(f"  {p.device:12} vid:pid={vid}:{pid}  loc={p.location}  {p.description}")
        try:
            print("UART autodetect ->", find_port())
        except SystemExit as e:
            print(e)
        print("xsdb ->", find_xsdb(args.xsdb))
        return 0

    cmd = args.cmd or DEFAULT_TESTS

    if not args.skip_jtag:
        missing = [p for p in (args.bit, args.ps7, args.uboot, args.fit,
                               args.payload, args.memcp, args.tcl)
                   if not os.path.exists(p)]
        if missing:
            print("ERROR: missing bundle files:", file=sys.stderr)
            for m in missing:
                print("  " + m, file=sys.stderr)
            return 1
        xsdb = find_xsdb(args.xsdb)

    u = Uart(args.port)
    print(f"== UART: {u.dev} @ 115200 ==", flush=True)

    try:
        if not args.skip_jtag:
            print("== JTAG: programming PL, staging DDR, loading U-Boot ==", flush=True)
            r = run_xsdb(xsdb, [
                args.tcl, args.bit, args.ps7, args.uboot,
                f"{args.fit}@{FIT_ADDR}",
                f"{args.payload}@{PAYLOAD_ADDR}",
            ])
            if r.returncode != 0:
                print("\nxsdb failed. Most common causes:\n"
                      "  - board not powered or JTAG cable not connected\n"
                      "  - Digilent drivers not installed (install Vivado's cable drivers)\n"
                      "  - another xsdb/Vivado/hw_server session already owns the cable",
                      file=sys.stderr)
                return 1

            # U-Boot: break out of autoboot, then boot the FIT.
            p, _ = u.wait_for([b"Hit any key", b"Zynq> "], 30)
            if p == b"Hit any key":
                u.send(b"\x03")
            time.sleep(0.5)
            # Keep U-Boot's ramdisk/dtb relocation below the mem= ceiling —
            # by default it relocates to the top of the full 1 GB, which mem=
            # hides from the kernel, silently stranding the ramdisk.
            u.uboot_cmd("setenv initrd_high 0x0F000000")
            u.uboot_cmd("setenv fdt_high 0x0F000000")
            print(u.uboot_cmd(
                f"setenv bootargs console=ttyPS0,115200 mem={args.mem}"),
                flush=True)
            u.clear()

            print("== booting Linux ==", flush=True)
            u.sendline(f"bootm {FIT_ADDR}")
            p, data = u.wait_for([b"login:"], 180)
            if not p:
                print("no login prompt; last output:\n" +
                      data[-800:].decode("utf-8", "replace"), file=sys.stderr)
                return 1
            u.sendline("root")
            p, _ = u.wait_for([b"Password:"], 15)
            if p:
                u.sendline("root")
            u.wait_for([b"# "], 15)
            u.sendline("stty -echo")   # clean, reproducible captures
            time.sleep(0.5)
            u.clear()

            print(u.shell_cmd("uname -a && sed -n 1p /proc/meminfo"), flush=True)

            deliver_memcp(u, args.memcp)

            size = os.path.getsize(args.payload)
            print("== extracting payload ==", flush=True)
            print(u.shell_cmd(
                f"cd /tmp && /tmp/memcp {PAYLOAD_ADDR[2:]} {size} - | tar xf - && ls -la",
                timeout_s=300), flush=True)

        print(f"+ {cmd}", flush=True)
        print(u.shell_cmd(cmd, timeout_s=args.cmd_timeout), flush=True)
    finally:
        u.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
