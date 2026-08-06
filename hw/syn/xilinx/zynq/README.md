# Vortex on Zynq-7000 (Arty Z7-10)

Runs a minimized Vortex on the XC7Z010 (17,600 LUTs): the ARM PS is the host,
driving Vortex's raw DCR sideband through a small AXI4-Lite shim
(`vortex_axil_shim.sv`), with Vortex's AXI4 memory master on `S_AXI_HP0`
sharing the PS DDR3. There is no Command Processor and no `sw/runtime/`
involvement — kernels run in *hostless mode* (`tests/kernel/*`), where
`vx_start` self-launches `main()`.

## Frozen XC7Z010 configuration

```
-DVX_CFG_EXT_F_DISABLE                      # FPU does not fit (F32 soft-FPU alone ~27k LUTs)
-DVX_CFG_EXT_A_ENABLE                       # REQUIRED: kernels are rv32imaf; vx_putchar's
                                            #   ring-overflow path executes amoadd.w — an
                                            #   illegal-instruction warp-kill without EXT_A
-DVX_CFG_NUM_WARPS=2 -DVX_CFG_NUM_THREADS=2
-DVX_CFG_ICACHE_SIZE=4096 -DVX_CFG_ICACHE_NUM_WAYS=1
-DVX_CFG_DCACHE_SIZE=4096 -DVX_CFG_DCACHE_NUM_WAYS=1
-DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=1
-DVX_CFG_MEM_BLOCK_SIZE=8                   # 64-bit internal bus, matches S_AXI_HP0
-DDISABLE_ASYNC_BRAM_PATCH                  # skip the netlist patch (guard added in
                                            #   VX_platform.vh); costs ~600 LUTs of LUTRAM
```

Full design post-route (xc7z010clg400-1, 50 MHz): 14,213 LUTs (81%),
WNS +6.7 ns (Fmax ≈ 75 MHz). With `MEM_BLOCK_SIZE=64` (512-bit bus) the core
alone needs 18k+ LUTs and does not fit; 4 threads (`w2t4`) does not fit
either; EXT_A + a System ILA together fail slice packing — drop the ILA
(`ENABLE_ILA` env in project.tcl) when atomics are enabled.

## Memory map (Vortex phys ≡ Zynq phys)

| Region                          | Address                  |
|---------------------------------|--------------------------|
| Kernel image + heap (link base) | 0x1000_0000              |
| IO region (COUT rings)          | 0x1FF0_0000 – 0x1FF1_0000|
| Exit code word                  | 0x1FF0_8308              |
| Per-hart stacks (grow down)     | below 0x1FFF_0000        |
| LMEM window (never reaches AXI) | 0x1FFF_0000 – 0x1FFF_4000|
| DCR shim (AXI-Lite, M_AXI_GP0)  | 0x43C0_0000              |

The relocated `VX_MEM_*` values are **baked into the generated headers** (the
defaults put stacks/IO outside Zynq's DDR). After (re)configuring a build
tree, regenerate both:

```sh
cd <build>
Z='-DVX_MEM_STACK_BASE_ADDR=536805376 -DVX_MEM_IO_BASE_ADDR=535822336 -DVX_MEM_IO_END_ADDR=535887872'
XLEN=32 python3 ../ci/gen_config.py --config ../VX_types.toml --cflags "$Z" --format verilog --resolved --output hw/VX_types.vh
XLEN=32 python3 ../ci/gen_config.py --config ../VX_types.toml --cflags "$Z" --format cpp --resolved --output sw/VX_types.h
```

(Values are decimal because `gen_config.py` overrides don't parse hex:
536805376 = 0x1FFF0000, 535822336 = 0x1FF00000, 535887872 = 0x1FF10000.
`LMEM_BASE`, `IO_COUT`, `EXIT_CODE` derive automatically.)

## Shim register map (0x43C0_0000)

| Offset | Register  | Behavior                                            |
|--------|-----------|-----------------------------------------------------|
| 0x00   | CTRL      | [0] vx_reset, RW, **resets to 1**                   |
| 0x04   | STATUS    | [0] busy, [1] dcr_rsp_seen (RO)                     |
| 0x08   | DCR_ADDR  | [11:0] DCR address (RW)                             |
| 0x0C   | DCR_WDATA | write pulses a 1-cycle DCR write at DCR_ADDR        |
| 0x10   | DCR_RDATA | last dcr_rsp_data (RO)                              |
| 0x14   | START     | write pulses Vortex_axi's `start` (KMU launch)      |
| 0x1C   | MAGIC     | 0x56585A37 "VXZ7" (RO)                              |

Launch protocol (per `VX_kmu.sv`, whose `busy` only rises on the `start`
pulse): clear CTRL[0] to release reset, write the KMU DCRs, write START, wait
for busy to rise then fall, read the exit code. (The sandbox testbench's
"DCRs during reset, launch on reset release" sequence predates the KMU and no
longer launches anything.)

## Build

```sh
# one-time: Digilent board files for the PS7 DDR3/MIO preset
git clone https://github.com/Digilent/vivado-boards ~/tools/digilent-vivado-boards

cd <build>/hw/syn/xilinx/zynq
source <Xilinx>/Vivado/settings64.sh
export VERILATOR=$HOME/tools/verilator/bin/verilator
make build          # -> vortex_z7.bit, vortex_z7.xsa (+ post_impl_util.rpt, timing.rpt)
```

`DEVICE=xc7z020clg400-1` retargets an Arty Z7-20 (room for a much larger
config there). `CLK_MHZ=25` lowers FCLK_CLK0 if timing fails.

## Kernel

```sh
cd <build>/tests/kernel/vecadd
CONFIGS="<frozen configs above>" STARTUP_ADDR=0x10000000 make
# validate in simulation first (ELF path carries the entry point):
../../../sim/rtlsim/rtlsim vecadd.elf
# flat image for the board:
$HOME/tools/llvm-vortex/bin/llvm-objcopy -O binary vecadd.elf vecadd.bin
```

## Run (JTAG + UART)

```sh
cd <build>/hw/syn/xilinx/zynq
vitis -s <src>/hw/syn/xilinx/zynq/host/build_app.py   # platform + app -> ws/
picocom -b 115200 /dev/ttyUSB1                        # in another terminal
xsdb <src>/hw/syn/xilinx/zynq/host/run.tcl vortex_z7.bit ws/ps7_init.tcl \
    ws/vortex_host/build/vortex_host.elf ../../../../tests/kernel/vecadd/vecadd.bin
```

Expected UART output: the shim banner, the vecadd hostless-mode banner
(streamed from the COUT rings), and `PASS (exit=0)`.

## Notes / gotchas

- **SD-boot U-Boot poisons the CPU's memory view.** After `rst -system` the
  board boots its SD-card U-Boot, which enables the MMU, L1, and the shared
  PL310 L2 — and JTAG `dow`/`con` inherit all of it. Stale L2 hits then mask
  Vortex's HP-port writes from the CPU indefinitely (exit code and COUT reads
  return pre-run values forever), and dirty evictions can clobber DRAM under
  Vortex. `start.S` therefore forces SCTLR MMU/caches off at entry, and
  `run.tcl` halts CPU1 and cleans+invalidates+disables the L2 before `con`.
- **The flat kernel image carries no BSS, and `vx_start` does not zero it**
  (the vxbin runtime loader normally handles it). `run.tcl` zeroes a 2 MB
  window at the kernel base before `dow -data`; without this the malloc pool
  and globals inherit stale DRAM garbage — symptoms include misaligned heap
  pointers and byte-smeared (`v | v<<24`) buffer values.
- The A9 runs with MMU/caches off (see above), so all its accesses are
  uncached and trivially coherent with the HP-port path.
- Kernels must not execute F-extension instructions (`EXT_F_DISABLE`); the
  repo's kernel flow still compiles with `-march=rv32imaf`, which is fine for
  integer kernels — rtlsim (same config) traps any stray FP instruction.
- The low 1 MB of DDR is not reachable through S_AXI_HP0 (Zynq address
  filtering); everything Vortex touches lives at 0x1000_0000+.
- `hw/syn/xilinx/dut` `make vortex FPU_IP=` leaks an empty `FPU_IP` env var
  into project.tcl (which tests `info exists`); this flow never sets it.
