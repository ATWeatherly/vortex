# Vortex on Zybo Z7-20 — portable run bundle

Boots Linux on a Digilent Zybo Z7-20 over JTAG, loads the Vortex bitstream,
and runs GPU workloads on it. Everything device-side is prebuilt — **no Vortex
source tree, no cross-compiler, no FPGA build** is needed on this machine.

Verified on Linux against the real board; the host scripts are pure Python +
pyserial and carry no OS-specific code.

---

## 0. Getting this onto the laptop

```
git clone https://github.com/<you>/vortex-zybo-z7-20.git
cd vortex-zybo-z7-20
python -m pip install pyserial
python run_zybo.py --list
```

The repo carries ~29 MB of prebuilt device artifacts (bitstream, kernel FIT,
U-Boot, payload) committed directly - no Git LFS, no submodules, nothing to
fetch separately. A plain clone is everything you need.

> **Line endings matter here.** `.gitattributes` marks every device artifact
> as binary, so a normal clone is safe even with `core.autocrlf=true`. If you
> ever copy these files around by other means, keep them byte-exact: a single
> translated byte in the bitstream or FIT means the board fails to boot with
> no useful error. Verify a fresh clone with:
>
> ```
> git fsck
> sha256sum -c SHA256SUMS.txt      # or: certutil -hashfile bit\vortex_z7.bit SHA256
> ```

## 1. Requirements

| | |
|---|---|
| **Vivado or Vitis** | Any 2025.x. Only `xsdb` and the cable drivers are used — not the GUI, not a license. |
| **Python** | 3.8 or newer. |
| **pyserial** | `python -m pip install pyserial` |
| **Cable drivers** | Installed by Vivado. On Windows, if the board does not appear, run Vivado's `installLibs` / "Install Cable Drivers" step once as Administrator. |

Hardware: Zybo Z7-20, USB cable to the **PROG/UART** port (the micro-USB next
to the power switch), power switch on. The single FTDI chip carries both JTAG
and the serial console — one cable does everything. No SD card, no jumper
changes; the board's QSPI factory image is never touched and is left intact.

## 2. Check the wiring

```
python run_zybo.py --list
```

Expected:

```
serial ports:
  COM4   vid:pid=0403:6010  loc=...  USB Serial Port
UART autodetect -> COM4
xsdb -> C:\Xilinx\Vivado\2025.2\bin\xsdb.bat
```

Both lines must resolve before going further.

- **No `0403:6010` port** → cable drivers missing, board unpowered, or cable in
  the wrong socket.
- **`xsdb` not found** → pass `--xsdb "C:\Xilinx\Vivado\2025.2\bin\xsdb.bat"`,
  or set the `XSDB` environment variable.
- **Wrong port chosen** → the FTDI exposes two channels; channel A is JTAG and
  channel B is the console. Autodetect prefers channel B, but if it guesses
  wrong, force it: `--port COM5` (or `set UART_DEV=COM5`).

## 3. Run

```
python run_zybo.py
```

Takes about 4–5 minutes. Sequence: program the PL → stage the kernel image and
payload into DDR over JTAG → boot U-Boot → boot Linux → push `memcp` over the
UART → extract the payload → run four tests.

Expected output:

```
== T1 vecadd n=16 ==
vecadd vortex2: n=16 buf=64B
PERF: instrs=104, cycles=1283, IPC=0.081
PASSED!
== T2 vecadd n=4096 ==
vecadd vortex2: n=4096 buf=16384B
PERF: instrs=24688, cycles=179739, IPC=0.137
PASSED!
== T3 llama2 -v 1 ==
>> Using vortex acceleration (vortex2/CP)
Once upon a time, there was a little girl named Lily. She loved to play
outside in the park. One day, she saw a big, r
achieved tok/s: 12.28
[vxmath_v2] calls=1440 cache_hits=1404 | device 3245.8 ms
== T4 llama2 -v 0 ==
achieved tok/s: 236.36
```

`instrs` counts are deterministic and should match to the digit. `cycles`,
`tok/s` and `device ms` vary by well under 1% between runs.

### Other invocations

```
python run_zybo.py --cmd "uname -a"             # boot, then one command
python run_zybo.py --skip-jtag --cmd "ls /tmp"  # board already booted — seconds, not minutes
python run_zybo.py --cmd "cd /tmp && ./llama2_perf stories260K.bin -z tok512.bin -t 0 -n 100 -v 1"
```

`--skip-jtag` reuses the running board, so iterating on a command costs no
reboot. The board stays up until it is power-cycled or re-run without the flag.

## 4. Reading the results

**T1/T2 — `vecadd`** is the stock Vortex regression test running on the full
`sw/runtime` stack through the Command Processor. `PASSED!` means the device
computed and the host verified every element.

**T3 — `llama2`** runs the unmodified llama.cpp host with **every matmul
offloaded to Vortex**. Check all three before trusting it:

- `>> Using vortex acceleration (vortex2/CP)` — offload is **opt-in via `-v 1`**.
  Without it the benchmark silently runs on the ARM cores and still prints
  correct text.
- `device <N> ms` is non-zero — real device time.
- `calls=1440 cache_hits=1404` — 1,440 matmuls, so 36 real weight uploads.

**T4 — the same model with no offload**, for comparison. It is ~19× faster
than T3. That is expected and is the honest framing of this port: a working
host/device split producing bit-exact results on a $300 board, **not** a
speedup. The generated text in T3 and T4 is byte-identical to an x86 reference.

## 5. What is in the bundle

```
run_zybo.py           orchestrator (JTAG + UART + boot + run)
uartlib.py            serial layer — threaded reader, U-Boot/shell helpers
jtag_boot_linux.tcl   xsdb script: program PL, ps7_init, L2 flush, stage DDR
bit/vortex_z7.bit     Vortex bitstream — XC7Z020, 2W x 4T + FPU, 59 MHz,
                      CP integration, PERF_ENABLE. 87.46% LUTs, WNS +0.152 ns
bit/ps7_init.tcl      PS bring-up (stands in for an FSBL)
boot/u-boot.elf       Digilent PetaLinux 2017.4 prebuilt
boot/image_slim.ub    kernel FIT: PL-stripped DTB + slimmed initramfs
payload/memcp         ARM static binary — recovers the DDR-staged payload
payload/payload.tar   vecadd_cp_arm, llama2_perf, both .vxbin kernels,
                      stories260K.bin, tok512.bin
```

Two details in `image_slim.ub` are load-bearing, not cosmetic. The stock
Digilent device tree describes *their* PL design, whose video timing controller
sits at `0x43C00000` — the address this bitstream uses for its control window —
and probing it raises an imprecise external abort that kills init. And their
144 MB initramfs cannot coexist with Linux at `mem=240M` on a tmpfs rootfs. The
shipped FIT has `amba_pl` removed and the initramfs cut to 39 MB.

## 6. How it works

The board's DDR is partitioned by address rather than being split across a bus:

```
0x00000000  Linux (240 MB, capped by the mem= bootarg)
0x0F000000  CP-visible host pool, 16 MB — command ring and DMA staging
0x10000000  Vortex device memory — buffers, kernel image
0x1FF00000  IO region — console rings, exit code
0x43C00000  Vortex control window (AXI-Lite on M_AXI_GP0)
```

Everything at or above `0x0F000000` is invisible to the kernel allocator, so
the Command Processor owns it outright — no pinning, no IOMMU, no bounce
buffers. "Copy to device" is a DDR write that happens to originate in the PL.

The model is sent to the board **once**, as part of `payload.tar`, before Linux
boots. Nothing streams from this machine during a run; pulling the JTAG cable
mid-generation would not stop it. Inside the board, each weight matrix uploads
to device memory once and is cached (hence 1,404 hits out of 1,440 calls).

## 7. Troubleshooting

**`Memory read error at 0xF8F00208. Cannot halt processor core, timeout`**
Stale JTAG session holding the cable. Close any Vivado/Vitis/xsdb instance and
kill leftover `hw_server` processes, then retry. (`jtag_boot_linux.tcl` already
halts both cores before `ps7_init`, which fixes the common form of this.)

**`no login prompt`**
Boot stalled. Usually the wrong serial channel — pass `--port` explicitly. If
the port is right, watch the console in a terminal (115200 8N1) during a run to
see where it stops.

**`shell command timed out`**
The console framing lost sync. Re-run with `--skip-jtag`; if it persists,
power-cycle and do a full run. Never run a second serial monitor against the
same port while this script is running — two readers on one FTDI corrupt both
captures.

**Tests pass but `tok/s` is much lower than above**
Another process is contending for the board, or `-v 1` was dropped (check for
the `vortex2/CP` banner and non-zero `device ms`).

## 8. Reproducing or changing the bitstream

Not needed to run anything here. Rebuilding requires the Vortex source tree,
Vivado with the Digilent board files, and roughly 2 hours of synthesis:

```sh
cd <vortex-build>/hw/syn/xilinx/zynq
CP=1 AREA_OPT=1 BOARD=zybo-z7-20 CLK_MHZ=59 CONFIGS='...' make build
```

The shipped bitstream is the `PERF_ENABLE` variant (2 warps x 4 threads). A
4x4 build without performance counters reaches 14.99 tok/s on T3 but needs a
device kernel rebuilt at matching config. The full architecture of the port —
topology, memory model, the host HAL, and the known sharp edges — is documented
separately in the port's architecture reference.
