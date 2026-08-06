# Vortex on the XC7Z010: What "It Runs" Does and Doesn't Mean

Vortex (the RISC-V GPGPU) runs on the Arty Z7-10's XC7Z010 — one of the
smallest Zynq-7000 parts (17,600 LUTs, 60 BRAM36, 80 DSP48E1) — executing a
multi-warp kernel from PS DDR3 with verified-correct results. Getting there
required cutting the configuration to the bone and replacing the entire host
software stack. This document enumerates every caveat, so claims about the
result can be made precisely.

Demonstrated:
- `tests/kernel/vecadd` (hostless mode): 16 work-items across 4 hardware
  threads, correct results and clean exit code over two consecutive runs,
  each from a full PS system reset.
- **The llama2 benchmark** (`bench/benchmarks/dnn/llama2`, host/device
  split): the unmodified `llama.cpp` host program running under Linux on the
  board's ARM cores, offloading every matmul to Vortex, generating
  **token-for-token golden-identical story text** on stories260K and
  verified generated tokens on stories15M. See section 5 for the precise
  claim and its caveats.

---

## 1. What was taken out or shrunk (vs. Vortex defaults)

| Feature | Default | This port | Why |
|---|---|---|---|
| Floating-point unit (`EXT_F`) | on | **removed** | The F32 soft-FPU alone is ~27k LUTs — 1.5× the whole chip. FP in kernels works only as soft-float library calls on the integer ALU. The DSP-based FPU IP was synthesized and **definitively does not fit either**: 20.9k LUTs at 2×2 threads, 19.1k even at 1 warp with LMEM freed (7-series DSP48E1 mapping needs ~7k LUTs of glue) — before the ~0.7k the PS interconnect adds. Hardware FP on this family starts at the XC7Z020. |
| Warps × threads | 4 × 4 | **2 × 2** | 4 hardware threads total (SIMD width 2). 4-thread variants exceed the device (`w2t4` ≈ 20.8k LUTs). 4 warps × 2 threads fits standalone (14.7k) but not with atomics + PS interconnect. |
| I-cache / D-cache | 16 KB, 4-way each | **4 KB, direct-mapped each** | Area (LUTs + BRAM). |
| Internal memory bus | 512-bit | **64-bit** (`MEM_BLOCK_SIZE=8`) | The make-or-break change: the 512-bit datapath alone pushes the core to 18k+ LUTs (unplaceable). Also shrinks cache lines to 8 bytes, which costs miss-rate performance. |
| Command Processor (`VX_cp_core`) | present in the FPGA shell | **removed** | Area. Consequence: **the entire standard host runtime (`sw/runtime`, `vortex.h`/`vortex2.h`, and anything built on it, e.g. OpenCL/PoCL) is unusable.** The host drives Vortex's raw DCR bus through a custom ~180-line AXI-Lite shim instead. |
| Kernel model | host-driven kernels | **hostless, or host-driven via libvortex-lite** | The stock `tests/regression/*` host-API tests still cannot run, but `host/linux/libvortex_lite.cpp` reimplements the legacy `vx_*` API surface over `/dev/mem` (launch = DCR writes + start pulse; args delivered via the KMU's `STARTUP_ARG`→`MSCRATCH` path), which is how the llama2 benchmark's host runs unmodified. |
| D-cache write policy | writeback | **write-through** (`DCACHE_WRITEBACK=0`) | Required for host-driven use: with no CP there is no host-triggered cache flush, so results must reach DRAM eagerly for the ARM to read them. Costs store bandwidth; irrelevant to read-dominated kernels. |
| L2/L3 caches, graphics (TEX/RASTER/OM/RTU), tensor (TCU), DMA, virtual memory | off by default | off | No change, listed for completeness. |
| Atomics (`EXT_A`) | off | **added (required)** | Not a cut — a forced addition: kernels compile with `-march=rv32imaf`, and the console ring's overflow path executes `amoadd.w`; without `EXT_A` that is an illegal instruction that kills the warp. Cost: the on-chip debug ILA no longer fits alongside it. |
| Async-BRAM netlist patch | applied | **disabled** | Precaution (one less unsimulated netlist transformation); costs ~600 LUTs of LUTRAM. |

Resulting utilization (post-route, full design incl. PS interconnect + shim,
write-through-dcache production build): **14,282 / 17,600 LUTs (81%)**,
~39 BRAM36-equivalents (~66%), 8 DSPs (10%), timing met at 50 MHz with
+6.9 ns slack (Fmax ≈ 75 MHz). Headroom for additions is essentially zero —
anything new means removing something else.

## 2. Performance caveats

- **50 MHz**, 4 hardware threads, no hardware FP: this is a proof of
  execution, not a compute platform. Peak integer throughput is on the order
  of 200 MOPS theoretical; a desktop CPU core is orders of magnitude faster.
- All memory traffic funnels through one 64-bit AXI HP port with 4 KB
  caches and 8-byte lines; cold instruction fetches were measured at ~50
  cycles each. vecadd (16 elements, print-heavy) takes ~1.4M cycles.
- Console output is a lossy shared-ring mechanism drained by the ARM core
  running uncached (slow): printed lines interleave between harts and can be
  clipped under overflow. This is cosmetic — computation and verification
  are unaffected — but exact console transcripts are not reproducible.

## 3. Host/software-stack caveats

- **No OpenCL, no stock Vortex runtime.** Two host modes exist, neither of
  them the standard stack: (a) a ~2 KB freestanding bare-metal program on
  the Cortex-A9 (no BSP — the Vitis 2025.2 platform generator fails on this
  XSA), loaded over JTAG via `xsdb`, results on the UART; (b) **Linux**
  (the board's stock PetaLinux 2019.2 SD image, booted with `mem=256M` to
  wall the kernel off from the Vortex DDR window) with `libvortex-lite`, a
  ~250-line userspace reimplementation of the legacy `vx_*` API over
  `/dev/mem`. Both are JTAG/UART-orchestrated by `host/linux/boot/
  boot_linux.py`; there is no self-booting deployment of the Vortex side
  (files are delivered by JTAG-staging DDR and U-Boot `fatwrite` to SD).
- Kernels must be integer-only (or accept soft-float), linked at
  `0x1000_0000` with a relocated memory map (stacks/IO moved into Zynq DDR
  range) that is **baked into the generated headers of the whole build
  tree** — the same tree cannot simultaneously target default-map platforms.
- The flat kernel image carries no BSS and the device startup code does not
  zero it; the JTAG script zeroes a 2 MB window before loading. Skipping
  this reproduces heap corruption from stale DRAM.
- The board's SD-card U-Boot boots before JTAG takes over and leaves the
  ARM MMU/L1/L2 enabled; the flow must force caches off, halt CPU1, and
  flush+disable the L2 every run (all automated in `run.tcl`/`start.S`).
  Stale L2 hits otherwise mask all of Vortex's memory writes from the ARM.
  Setting the boot-mode jumper to JTAG would remove this class of hazard.

## 4. Validation-scope caveats

- **Runs on real silicon**: every result here is a physical FPGA executing
  kernels from board DDR3 with results verified by the ARM host — nothing
  about it is simulated. The hardware-proven set is: vecadd (hostless,
  twice), and the llama2 matmul kernel across **thousands of consecutive
  launches** (~25 per token × 40 tokens on stories260K, plus the stories15M
  runs) with golden-verified outputs. The broader Vortex regression suite
  has not run on the board. Confidence beyond these workloads rests on RTL
  simulation of the same frozen configuration (Verilator, including a
  hostile AXI memory model with backpressure, 20–60-cycle latencies, and
  out-of-order responses), which is supporting evidence, not a hardware
  result.
- The final bitstream has no debug ILA (it cannot coexist with `EXT_A` at
  this utilization); hardware debug requires rebuilding a reduced config.
- One RTL-adjacent repo change was required: a `DISABLE_ASYNC_BRAM_PATCH`
  guard in `hw/rtl/VX_platform.vh`. The stock repo has no Zynq-7000/7-series
  support; everything in `hw/syn/xilinx/zynq/` is new.

## 5. The llama2 result: precise claim and caveats

**Claim:** the llama2 benchmark (`bench/benchmarks/dnn/llama2` — Karpathy
llama2.c ported to the Vortex host API) runs with its host program's logic
unmodified on Linux on the board's ARM cores, offloading **every matmul** to
Vortex, and produces story text **token-for-token identical to the x86
reference** on stories260K (40/40 tokens, deterministic argmax) with
verified generated tokens on stories15M.

| Configuration | Model | tok/s |
|---|---|---|
| x86 reference (`-v 0`) | stories260K | 4,875 |
| ARM-on-board only (`-v 0`) | stories260K | 222.9 |
| Vortex matmuls, first-light build | stories260K | 0.119 (~8.4 s/token) |
| **Vortex matmuls, tuned build** | stories260K | **0.197** (~5.1 s/token) |
| Vortex matmuls, first-light build | stories15M (60 MB, benchmark default) | 0.00216 (~7.7 min/token) |
| **Vortex matmuls, tuned build** | stories15M | **0.00349** (~4.8 min/token) |

The tuned build (measured after a systematic performance pass, all steps
golden-verified): fused single-rounding soft-float MAC (+18%), 16-byte cache
lines (`MEM_BLOCK_SIZE=16`, the internal-bus/line-size knob decoupled from
the 64-bit AXI port by the width adapter), 66.7 MHz (Fmax 70.3), 8 KB 2-way
icache + 2-way dcache, `LMEM_DISABLE` to pay for it — 14.5k LUTs (82%).
Instrumented attribution along the way established: 99.6% of runtime is
device-side kernel execution; host↔device copies and launch overhead are
negligible (weight caching and a writeback-dcache experiment both measured
as no-ops); associativity was worth only +4%. The end state is
**instruction-throughput-bound**: ~306 cycles per hart per MAC is simply the
cost of IEEE-correct SoftFloat mulAdd on a 4-thread in-order core — further
gains require sacrificing FP fidelity (fast-path MAC), not better caching.

Caveats on that claim:

- **Division of labor is the benchmark's own**: only the four GEMM shapes run
  on Vortex (as upstream designed it); RMSNorm, attention softmax, RoPE,
  tokenizer, and sampling run on the ARM host in hardware FP32. The device
  work is real (~25 kernel launches per token, thousands per run) but it is
  matmul offload, not whole-model-on-GPU.
- **All device math is soft-float**: FP32 via Berkeley-SoftFloat libcall
  wrappers on the integer ALU (`rv32ima`/`ilp32`, fully freestanding — no
  prebuilt device libc is soft-float-ABI compatible). IEEE-correct, hence
  the token-exact match, but ~2 orders of magnitude slower than hardware FP.
- **Performance is dominated by design-inherited overheads**: the benchmark
  re-uploads all touched weights to the device every call (the entire model
  streams over an uncached `/dev/mem` mapping once per token), and
  `libvortex-lite` re-uploads and re-zeroes the kernel image every launch
  (each launch is a full GPU reset). None of this was optimized; the numbers
  are an existence proof, not a tuned result.
- **stories15M ran 5 forward passes total** (3 prompt steps + a 2-step
  generation run whose tokens matched golden) — enough to verify the
  pipeline and time it, not a long-form generation.
- Device-kernel toolchain caveats worth knowing: clang miscompiles
  `vx_spawn.c`'s warp-activation stub at this configuration (that one file
  must be GCC-compiled, as upstream's own library happens to be), and the
  empty-`.tdata` case of `link32.ld` yields a misaligned TLS pointer for odd
  harts (fixed with an aligned `__thread` anchor).

## 6. What a bigger part would buy

The Arty Z7-**20** (XC7Z020, 53,200 LUTs — same board family, same flow,
`DEVICE=xc7z020clg400-1`) would fit roughly the default 4×4-thread core with
16 KB caches, atomics, a debug ILA, and plausibly the DSP-based hardware FPU
— i.e., most of this document's section 1 disappears. The Command
Processor + standard runtime would still need a port (or a Linux/PYNQ host
story) as a separate effort.

---

*One-sentence summary for citation:* Vortex runs on the XC7Z010 at 50 MHz as
a 2-warp × 2-thread integer-only core (no FPU, quarter-size caches, 64-bit
memory bus, no Command Processor — a minimal reimplemented host API instead
of the stock runtime), using 81% of the device, and produces
verified-correct results on the vecadd kernel and on the llama2 benchmark,
where Linux on the chip's ARM cores offloads every matmul to Vortex and
generates token-for-token reference-identical text at 0.119 tok/s
(stories260K) — an architectural existence proof, not a performance result.
