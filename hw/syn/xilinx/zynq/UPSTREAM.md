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

## 6. Bug: CP queue reset is documented but unwired — second host process reads phantom completions

`VX_cp_axil_regfile` documents `Q_CONTROL.reset_pulse` (bit 1) and
`CP_CTRL.reset_all` (bit 1), and drives a `q_reset_pulse[]` output. In
`VX_cp_core` that output is discarded:

```systemverilog
  // Reset pulse from regfile (Q_CONTROL.reset / CP_CTRL.reset_all) is
  // not propagated to CPEs as a separate signal. ...
  for (genvar q = 0; q < NUM_QUEUES; ++q) begin : g_unused_reset
    `UNUSED_VAR (q_reset_pulse[q])
  end
```

So a host cannot reset the CP. On a real device the CP is only reset when
the FPGA is reprogrammed, and its fetch head + retired-seqnum counter
survive host-process exit. `Device::cp_init_` then programs a fresh ring
and starts counting `cp_expected_seqnum_` from 0, while the hardware's
`Q_SEQNUM` is still at (say) 21000. Every `cp_batch_end` poll —
`if (seqnum32 >= target) break;` — is satisfied instantly, so the host
reads result buffers before the CP has written them.

Symptom: the first process after programming the bitstream is correct;
every later process returns stale/garbage data, intermittently and
proportional to how fast the host submits (pacing the host — e.g. adding
per-call verification — hides it completely). Nothing reports an error:
`Q_ERROR` stays 0 and every command does eventually execute.

Reproduced on a Zynq-7000 (XC7Z020) CP integration: run any two runtime
processes back-to-back on one bitstream load.

Two fixes, ideally both:
1. **RTL**: honor the documented reset — propagate `q_reset_pulse` to the
   CPE fetch/retire state (clear head/tail/seqnum), or delete the bits
   from the register map and its documentation.
2. **Runtime** (what this port does, works with today's RTL): read
   `Q_SEQNUM` at open and adopt it — `cp_expected_seqnum_ = hw_seqnum;
   cp_tail_ = hw_seqnum * CP_CL_BYTES;` — so the host resumes the
   hardware's sequence instead of assuming a fresh device.

Also worth hardening regardless: `sw/runtime`'s MMIO writes rely on a
release fence *before* the doorbell but none after; on ARM the ring
(Normal/Device host memory) and the CP regfile (Device MMIO) are separate
regions and need barriers on both sides.
