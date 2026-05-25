#!/bin/bash

# =============================================================================
# Vortex MPI XRT Real-Hardware Regression Test Script
# =============================================================================
# Runs the 8 upstream MPI regression tests across two Alveo U50s on flubber1.
# Each MPI rank (0, 1) is automatically mapped to the U50 at that device index
# via the OMPI_COMM_WORLD_RANK fallback added to runtime/xrt/vortex.cpp.
#
# Usage:
#   ./run_mpi_xrt_regression.sh --fpga-bin-dir=<path-to-dir-with-xclbin>
#                               [--np=<num-devices>]
#                               [--skip-build]
#
#   --fpga-bin-dir : directory containing vortex_afu.xclbin  (REQUIRED)
#   --np           : number of MPI ranks / U50s (default: 2)
#   --skip-build   : skip configure + runtime build (reuse most-recent build dir)
#
# Output:
#   - Build directory: build_mpi_xrt_YYYYMMDD_HHMMSS/
#   - Logs: build_mpi_xrt_YYYYMMDD_HHMMSS/regression_logs/
#   - Summary: build_mpi_xrt_YYYYMMDD_HHMMSS/regression_logs/summary.txt
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# =============================================================================
# Argument parsing
# =============================================================================

SKIP_BUILD=0
NP=2
FPGA_BIN_DIR=""

for arg in "$@"; do
    case $arg in
        --fpga-bin-dir=*) FPGA_BIN_DIR="${arg#*=}" ;;
        --np=*)           NP="${arg#*=}" ;;
        --skip-build)     SKIP_BUILD=1 ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 --fpga-bin-dir=<path> [--np=<n>] [--skip-build]"
            exit 1
            ;;
    esac
done

if [ -z "$FPGA_BIN_DIR" ]; then
    echo "ERROR: --fpga-bin-dir is required"
    echo "Usage: $0 --fpga-bin-dir=<path-to-dir-with-xclbin> [--np=<n>] [--skip-build]"
    exit 1
fi

if [ ! -f "$FPGA_BIN_DIR/vortex_afu.xclbin" ]; then
    echo "ERROR: vortex_afu.xclbin not found in $FPGA_BIN_DIR"
    exit 1
fi

# =============================================================================
# Configuration
# =============================================================================

export PATH=/home/tools/verilator/bin:$PATH

TESTS=(
    mpi_vecadd
    mpi_dotproduct
    mpi_diverge
    mpi_sgemm
    mpi_blocked_sgemm
    mpi_conv3
    mpi_neighbor_a2a_conv3
    mpi_put_dotproduct
)
TOTAL=${#TESTS[@]}

FAILURE_PATTERNS="FAILED|%Error:|Assertion failed|Aborted|core dumped|make: \*\*\*|Segmentation fault"

# =============================================================================
# Build directory setup
# =============================================================================

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BUILD_DIR="$SCRIPT_DIR/build_mpi_xrt_$TIMESTAMP"

echo "=============================================="
echo "  Vortex MPI XRT Real-Hardware Regression"
echo "=============================================="
echo "  Timestamp:    $TIMESTAMP"
echo "  Build Dir:    $BUILD_DIR"
echo "  FPGA_BIN_DIR: $FPGA_BIN_DIR"
echo "  NP (ranks):   $NP"
echo "  Total Tests:  $TOTAL"
echo "=============================================="
echo ""

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Export so make picks it up when expanding $(FPGA_BIN_DIR) in common.mk
export FPGA_BIN_DIR

# =============================================================================
# Configure and build (XRT runtime only — no RTL sim needed for real hardware)
# =============================================================================

if [ $SKIP_BUILD -eq 0 ]; then
    echo "[STEP 1/3] Configuring..."
    if ! ../configure --xlen=32 --tooldir=/home/tools; then
        echo "ERROR: Configuration failed!"
        exit 1
    fi

    echo ""
    echo "[STEP 2/3] Building XRT runtime..."
    if ! make -s -C runtime/xrt; then
        echo "ERROR: XRT runtime build failed!"
        exit 1
    fi

    echo ""
    echo "Build completed successfully!"
    echo ""
else
    echo "[SKIP] Skipping configure and build (--skip-build)"
    PREVIOUS_BUILD_DIR=$(ls -td "$SCRIPT_DIR"/build_mpi_xrt_* 2>/dev/null | grep -v "$BUILD_DIR" | head -n 1)
    if [ -z "$PREVIOUS_BUILD_DIR" ]; then
        echo "ERROR: No previous build_mpi_xrt_* directory found!"
        exit 1
    fi
    echo "Using previous build: $PREVIOUS_BUILD_DIR"
    cp -r "$PREVIOUS_BUILD_DIR"/ci "$PREVIOUS_BUILD_DIR"/runtime "$PREVIOUS_BUILD_DIR"/tests \
          "$PREVIOUS_BUILD_DIR"/config.mk "$PREVIOUS_BUILD_DIR"/Makefile \
          "$PREVIOUS_BUILD_DIR"/hw "$PREVIOUS_BUILD_DIR"/kernel ./
fi

# =============================================================================
# Run tests
# =============================================================================

set +e

echo "[STEP 3/3] Running $TOTAL MPI tests on real FPGA (NP=$NP)..."
echo ""

LOG_DIR="$BUILD_DIR/regression_logs"
mkdir -p "$LOG_DIR"
SUMMARY_FILE="$LOG_DIR/summary.txt"

PASSED=()
FAILED=()

for i in "${!TESTS[@]}"; do
    TEST="${TESTS[$i]}"
    TEST_NUM=$((i + 1))
    LOG_FILE="$LOG_DIR/${TEST}.log"

    printf "[%d/%d] Running %-30s ... " "$TEST_NUM" "$TOTAL" "$TEST"

    ./ci/blackbox.sh \
        --driver=xrt \
        --app="$TEST" \
        --np="$NP" \
        --target=hw \
        > "$LOG_FILE" 2>&1
    EXIT_CODE=$?

    if [ $EXIT_CODE -ne 0 ] || grep -qE "$FAILURE_PATTERNS" "$LOG_FILE"; then
        echo "FAILED (exit=$EXIT_CODE)"
        FAILED+=("$TEST")
    else
        echo "PASSED"
        PASSED+=("$TEST")
    fi
done

# =============================================================================
# Summary
# =============================================================================

echo ""
echo "=============================================="
echo "  SUMMARY"
echo "=============================================="
echo "  Total:   $TOTAL"
echo "  Passed:  ${#PASSED[@]}"
echo "  Failed:  ${#FAILED[@]}"
echo ""

{
    echo "=============================================="
    echo "  Vortex MPI XRT Real-Hardware Regression Summary"
    echo "=============================================="
    echo "  Date:         $(date)"
    echo "  Build Dir:    $BUILD_DIR"
    echo "  FPGA_BIN_DIR: $FPGA_BIN_DIR"
    echo "  NP:           $NP"
    echo ""
    echo "=============================================="
    echo "  RESULTS: ${#PASSED[@]}/${TOTAL} PASSED"
    echo "=============================================="
    echo ""
    echo "PASSED (${#PASSED[@]}):"
    for t in "${PASSED[@]}"; do echo "  [PASS] $t"; done
    echo ""
    echo "FAILED (${#FAILED[@]}):"
    for t in "${FAILED[@]}"; do echo "  [FAIL] $t"; done
} > "$SUMMARY_FILE"

if [ ${#FAILED[@]} -gt 0 ]; then
    echo "Failed tests:"
    for t in "${FAILED[@]}"; do
        echo "  - $t (see $LOG_DIR/${t}.log)"
    done
    echo ""
fi

if command -v bc &> /dev/null; then
    PASS_RATE=$(echo "scale=1; ${#PASSED[@]} * 100 / $TOTAL" | bc)
    echo "Pass rate: $PASS_RATE% (${#PASSED[@]}/$TOTAL)"
else
    echo "Pass rate: ${#PASSED[@]}/$TOTAL"
fi

echo ""
echo "Detailed logs: $LOG_DIR/"
echo "Summary file:  $SUMMARY_FILE"
echo ""

if [ ${#FAILED[@]} -gt 0 ]; then
    exit 1
else
    echo "All tests passed!"
    exit 0
fi
