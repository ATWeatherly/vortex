#!/usr/bin/env bash
# Build the Zynq CP runtime + its ARM test host from a fresh clone.
#
# This is the whole bootstrap for the host-software side of the Zybo Z7-20
# port: it generates the board's relocated headers, builds libvortex_zynq.a
# (the full sw/runtime with the /dev/mem CP backend), links the vecadd
# regression host against it, and optionally packs a payload.tar ready for
# the portable runner in hw/syn/xilinx/zynq/host/portable.
#
#   ./build_runtime.sh                  # headers + libvortex_zynq.a + vecadd host
#   ./build_runtime.sh --payload        # ...and pack payload.tar
#   ./build_runtime.sh --clean
#
# Requirements:
#   * an ARM hard-float cross-compiler. Default expects
#     $HOME/tools/arm-linux-musleabihf-cross (https://musl.cc/#binaries,
#     arm-linux-musleabihf-cross.tgz). Override with CROSS=<prefix>.
#   * python3 (for ci/gen_config.py)
#   * the Vortex kernel toolchain is NOT needed unless you also rebuild
#     device kernels (.vxbin) — the prebuilt ones work as-is.
#
# Runs on any Linux host (incl. WSL2). Nothing here needs Vivado; nothing
# here needs the board.
set -euo pipefail

cd "$(dirname "$0")"
VORTEX_HOME="$(pwd)"
BUILD_TREE="${BUILD_TREE:-$VORTEX_HOME/build}"
CROSS="${CROSS:-$HOME/tools/arm-linux-musleabihf-cross/bin/arm-linux-musleabihf-}"

# ---------------------------------------------------------------------------
# The Zybo Z7-20 memory map. These are the values baked into the board's
# bitstream and kernels: device memory at 0x10000000, IO/COUT at 0x1FF00000,
# stacks below 0x1FFF0000. The stock defaults put stacks and IO outside
# Zynq's DDR entirely, so every build MUST override them.
#
# Decimal because gen_config.py's scalar parser does not accept hex:
#   268435456 = 0x10000000   535822336 = 0x1FF00000
#   536805376 = 0x1FFF0000   535887872 = 0x1FF10000
# ---------------------------------------------------------------------------
ZYNQ_MEM_MAP=(
  -DVX_MEM_USER_BASE_ADDR=268435456
  -DVX_MEM_STACK_BASE_ADDR=536805376
  -DVX_MEM_IO_BASE_ADDR=535822336
  -DVX_MEM_IO_END_ADDR=535887872
)

# Hardware config of the shipped bitstream (perf_config_B: 2 warps x 4
# threads, FPU, 8K 2-way caches, LMEM off, 16-byte lines, perf counters).
# Device shape is read from CP_DEV_CAPS at runtime, so a mismatch here is
# not fatal — but keep it honest.
ZYNQ_CONFIGS="${ZYNQ_CONFIGS:--DVX_CFG_EXT_A_ENABLE -DVX_CFG_NUM_WARPS=2 -DVX_CFG_NUM_THREADS=4 -DVX_CFG_ICACHE_SIZE=8192 -DVX_CFG_ICACHE_NUM_WAYS=2 -DVX_CFG_DCACHE_SIZE=8192 -DVX_CFG_DCACHE_NUM_WAYS=2 -DVX_CFG_LMEM_DISABLE -DVX_CFG_PLATFORM_MEMORY_NUM_BANKS=1 -DVX_CFG_MEM_BLOCK_SIZE=16}"

say() { printf '\n=== %s ===\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

if [ "${1:-}" = "--clean" ]; then
  rm -rf "$BUILD_TREE/sw" "$BUILD_TREE/hw" payload
  make -C sw/runtime/zynq -f Makefile.arm clean 2>/dev/null || true
  echo "cleaned"
  exit 0
fi

# -- 0. sanity ---------------------------------------------------------------
command -v python3 >/dev/null || die "python3 not found"
[ -x "${CROSS}g++" ] || die "ARM cross-compiler not found at ${CROSS}g++
  Get it from https://musl.cc/#binaries (arm-linux-musleabihf-cross.tgz),
  unpack into \$HOME/tools/, or set CROSS=<prefix>."

say "toolchain"
echo "  CROSS      = $CROSS"
"${CROSS}g++" --version | head -1 | sed 's/^/  /'
echo "  BUILD_TREE = $BUILD_TREE"

# -- 1. generated headers ----------------------------------------------------
# sw/runtime includes VX_types.h / VX_config.h from $BUILD_TREE/{sw,hw}.
# These carry the relocated memory map above; without them the runtime
# builds against the stock map and DMAs into Linux RAM.
say "generating headers with the Zybo memory map"
mkdir -p "$BUILD_TREE/sw" "$BUILD_TREE/hw"
CF="${ZYNQ_MEM_MAP[*]} -DVX_CFG_XLEN=32"

gen() { # <toml> <format> <out>
  python3 ci/gen_config.py --config "$1" --cflags "$CF" \
          --format "$2" --resolved --output "$3"
  echo "  wrote $3"
}
gen VX_types.toml  cpp     "$BUILD_TREE/sw/VX_types.h"
gen VX_types.toml  verilog "$BUILD_TREE/hw/VX_types.vh"
gen VX_config.toml cpp     "$BUILD_TREE/sw/VX_config.h"
gen VX_config.toml verilog "$BUILD_TREE/hw/VX_config.vh"

# Fail loudly if the relocation did not take — this is the single most
# consequential value in the whole build (see CAVEATS: a stock
# VX_MEM_USER_BASE_ADDR made the CP DMA into Linux's RAM).
grep -q "define VX_MEM_USER_BASE_ADDR 268435456" "$BUILD_TREE/sw/VX_types.h" \
  || die "VX_MEM_USER_BASE_ADDR is not 0x10000000 — the memory map did not apply"
echo "  verified: USER_BASE=0x10000000  IO=0x1FF00000  STACK=0x1FFF0000"

# -- 2. the runtime ----------------------------------------------------------
say "building libvortex_zynq.a (sw/runtime + Zynq CP backend)"
make -C sw/runtime/zynq -f Makefile.arm \
     VORTEX_HOME="$VORTEX_HOME" BUILD_TREE="$BUILD_TREE" CROSS="$CROSS" \
     CONFIGS="$ZYNQ_CONFIGS"

LIB="sw/runtime/zynq/libvortex_zynq.a"
[ -f "$LIB" ] || die "$LIB was not produced"

# -- 3. the ARM test host ----------------------------------------------------
# The stock vecadd regression, linked statically against the runtime. Static
# because the board's rootfs is a minimal initramfs with no matching libc,
# and dispatcher_static.cpp removes the dlopen path the shared build needs.
say "building vecadd_cp_arm (static)"
"${CROSS}g++" -std=c++17 -O2 -static \
  -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard \
  -I sw/runtime/include -I sw/common -I "$BUILD_TREE/sw" \
  tests/regression/vecadd/main.cpp \
  "$LIB" -lpthread \
  -o sw/runtime/zynq/vecadd_cp_arm
"${CROSS}strip" sw/runtime/zynq/vecadd_cp_arm
file sw/runtime/zynq/vecadd_cp_arm | sed 's/^/  /'

# -- 4. optional payload -----------------------------------------------------
if [ "${1:-}" = "--payload" ]; then
  say "packing payload.tar"
  KBIN="$BUILD_TREE/tests/regression/vecadd/kernel.vxbin"
  [ -f "$KBIN" ] || die "device kernel not found: $KBIN
  Prebuilt kernels ship in the portable bundle's payload.tar; rebuilding
  them needs the Vortex kernel toolchain (llvm-vortex + riscv32-gnu)."
  rm -rf payload && mkdir -p payload
  cp sw/runtime/zynq/vecadd_cp_arm payload/
  cp "$KBIN" payload/vecadd_kernel.vxbin
  tar cf payload.tar -C payload .
  echo "  payload.tar: $(stat -c%s payload.tar) bytes"
  tar tf payload.tar | sed 's/^/    /'
  cat <<'EOF'

Run it with the portable runner (on the machine wired to the board):
  python run_zybo.py --payload <path>/payload.tar \
      --cmd "cd /tmp && ./vecadd_cp_arm -k vecadd_kernel.vxbin -n 16"
EOF
fi

say "done"
echo "  $LIB"
echo "  sw/runtime/zynq/vecadd_cp_arm"
