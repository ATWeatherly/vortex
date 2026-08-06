# Upstream contribution drafts (review before pushing anything public)

Five self-contained packages discovered/produced by the Arty Z7-10 port.
Nothing here has been pushed anywhere; each section is a ready-to-file
PR description or issue body.

---

## 1. PR: Zynq-7000 (Arty Z7) port — `hw/syn/xilinx/zynq`

**Target:** vortexgpgpu/vortex. **Branch:** `feature_zynq_arty_z7` (local).

Adds a Vivado block-design flow for Zynq-7000 (validated on the XC7Z010 /
Arty Z7-10, the smallest practical part): `Vortex_axi` + a ~180-line
AXI-Lite shim driving the DCR bus/reset/start directly (no Command
Processor — it does not fit), a bare-metal host, and a Linux host runtime
(`libvortex-lite`, legacy `vx_*` API over `/dev/mem`) with self-booting SD
setup. Includes a fitting config for 17.6k LUTs (no FPU, 2×2 threads,
16-byte lines, 66.7 MHz) and CAVEATS.md documenting exactly what was
validated (vecadd + the llama2 benchmark's matmuls, golden-verified).
Cost to core RTL: one opt-out guard (see item 3-adjacent change in
`VX_platform.vh`).

## 2. PR: `dut`: only define `FPU_IP` when the config has an FPU

**Target:** vortexgpgpu/vortex. **Commit ready:** `b0fa86621`.

`hw/syn/xilinx/dut/{core,vortex}/Makefile` set `FPU_IP = 1` unconditionally.
The natural workaround for FPU-less configs, `make vortex FPU_IP=`, exports
an *empty* `FPU_IP` into the recipe environment; `project.tcl` gates on
`info exists ::env(FPU_IP)` and then runs `xilinx_ip_gen.tcl` with an empty
ip_dir and dies. Fix: `ifeq (,$(filter -DVX_CFG_EXT_F_DISABLE,$(CONFIGS)))`
around the define.

## 3. Issue: `hw/syn/xilinx/sandbox` is stale and cannot launch kernels

**Target:** vortexgpgpu/vortex.

Two independent breakages against current RTL:
- `Vortex_wrap.sv` / `Vortex_top.v` do not connect `Vortex_axi`'s `start`
  input. Since the KMU rework (`busy = running`, set only by the `start`
  pulse — `VX_kmu.sv`), releasing reset no longer launches anything: the
  sandbox can only produce an idle core.
- `testbench.v` uses `` `VX_DCR_BASE_STARTUP_ADDR0``, which no longer
  exists (`VX_DCR_KMU_STARTUP_ADDR0` since the KMU DCR block); the sim
  fails to elaborate against generated headers.
Suggest either porting the fixes from the zynq flow (which wraps the same
module correctly) or marking the sandbox deprecated in its README.

## 4. Issue: clang/llvm-vortex miscompiles `vx_spawn.c`'s warp-activation stub (rv32ima/ilp32)

**Target:** vortexgpgpu/llvm (llvm-vortex), cc vortexgpgpu/vortex.

At `-march=rv32ima_zicond -mabi=ilp32 -O2`, clang (llvm-vortex 20.1.8,
`+xvortex`) CSEs the two `vx_warp_id()` CSR reads in the spawn stub into a
single entry-time read. Lanes activated mid-function by the stub's `vx_tmc`
then reach the final `vx_tmc(0 == vx_warp_id())` holding the *stale*
pre-activation warp id, so warp 0 silently retires and `vx_spawn_threads`'s
groups path (block_dim > 1) never completes — the kernel exits without
writing an exit code. GCC-compiled `vx_spawn.o` (as upstream's own
`libvortex.a` happens to be) is correct; `-mllvm -vortex-branch-divergence=0`
does not help. Differential-build evidence and a COUT-instrumented trace:
`bench/benchmarks/dnn/llama2/zynq/kernel/REPORT.md`. CSR reads that are
warp-state-dependent presumably need to be modeled as volatile/convergent
for the xvortex target.

## 5. Issue: `link32.ld` + empty `.tdata` yields misaligned TLS pointers

**Target:** vortexgpgpu/vortex.

When a kernel contributes no `.tdata` (easy in freestanding builds),
`link32.ld` computes `__tbss_offset = 2` and `__tls_block_size = 0x22`
(unaligned), so odd harts receive a `tp` misaligned by 2 —
`vx_start.S`'s per-hart TLS stride arithmetic then produces misaligned
thread-local accesses (trap on rtlsim; silent misalignment hazard on
hardware). Newlib links mask this by always contributing `.tdata`.
Suggested fix: `ALIGN(4)` on the tdata/tbss boundaries (or align
`__tls_block_size`) in `link32.ld`; workaround used in the port: a
4-byte-aligned `__thread int` anchor object.

---

**Proposed order:** 2 (trivial), 5 (small linker fix), 3 (issue only),
4 (issue with repro), 1 (the big PR, after the others land or reference it).
