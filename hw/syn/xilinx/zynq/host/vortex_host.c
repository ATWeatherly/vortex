// Copyright © 2019-2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Bare-metal Cortex-A9 host for Vortex on Zynq-7000 (Arty Z7).
//
// Expects the kernel's flat image (llvm-objcopy -O binary of a hostless-mode
// ELF linked at KERNEL_BASE) to already sit at KERNEL_BASE — the xsct run
// script downloads it with `dow -data <kernel>.bin 0x10000000` before `con`.
//
// Launch protocol (per hw/syn/xilinx/sandbox/testbench.v): program the KMU
// startup DCR while Vortex is held in reset, release reset, wait for busy to
// rise then fall, then read the exit-code word the kernel's _Exit stored.

#include <stdio.h>
#include <xil_io.h>
#include <xil_mmu.h>
#include <xil_cache.h>
#include <xpseudo_asm.h>
#include <sleep.h>
#include <xtime_l.h>

// vortex_axil_shim register file (M_AXI_GP0)
#define SHIM_BASE       0x43C00000u
#define SHIM_CTRL       (SHIM_BASE + 0x00)  // [0] vx_reset (resets to 1)
#define SHIM_STATUS     (SHIM_BASE + 0x04)  // [0] busy
#define SHIM_DCR_ADDR   (SHIM_BASE + 0x08)
#define SHIM_DCR_WDATA  (SHIM_BASE + 0x0C)  // write pulses a DCR write
#define SHIM_MAGIC      (SHIM_BASE + 0x1C)  // 0x56585A37 "VXZ7"
#define SHIM_MAGIC_VAL  0x56585A37u

// Device memory map (must match the VX_MEM_* overrides baked into the build's
// VX_types.vh/.h and the kernel link address — see ../README.md)
#define KERNEL_BASE     0x10000000u
#define SHARED_BASE     0x10000000u  // uncached window start (1 MB sections)
#define SHARED_END      0x20000000u
#define IO_BASE         0x1FF00000u
#define IO_SIZE         0x00010000u
#define EXIT_CODE_ADDR  0x1FF08308u

// COUT stream rings @ IO_BASE: wr[64] u32, rd[64] u32, data[64][512], lost[64] u32
#define COUT_SLOTS      64u
#define COUT_RING       512u
#define COUT_WR(i)      (IO_BASE + 4u * (i))
#define COUT_RD(i)      (IO_BASE + 4u * (COUT_SLOTS + (i)))
#define COUT_DATA(i, k) (IO_BASE + 8u * COUT_SLOTS + (i) * COUT_RING + (k))
#define COUT_LOST(i)    (IO_BASE + 8u * COUT_SLOTS + COUT_SLOTS * COUT_RING + 4u * (i))

// KMU DCR addresses (build/sw/VX_types.h)
#define DCR_STARTUP_ADDR0  0x010
#define DCR_STARTUP_ADDR1  0x011
#define DCR_STARTUP_ARG0   0x014
#define DCR_STARTUP_ARG1   0x015
#define DCR_BLOCK_DIM_X    0x016
#define DCR_BLOCK_DIM_Y    0x017
#define DCR_BLOCK_DIM_Z    0x018
#define DCR_GRID_DIM_X     0x019
#define DCR_GRID_DIM_Y     0x01A
#define DCR_GRID_DIM_Z     0x01B
#define DCR_LMEM_SIZE      0x01C
#define DCR_BLOCK_SIZE     0x01D
#define DCR_WARP_STEP_X    0x01E
#define DCR_WARP_STEP_Y    0x01F
#define DCR_WARP_STEP_Z    0x020
#define DCR_CLUSTER_DIM_X  0x021
#define DCR_CLUSTER_DIM_Y  0x022
#define DCR_CLUSTER_DIM_Z  0x023

#define VX_NUM_THREADS  2  // must match -DVX_CFG_NUM_THREADS of the bitstream

static void dcr_write(u32 addr, u32 value) {
    Xil_Out32(SHIM_DCR_ADDR, addr);
    Xil_Out32(SHIM_DCR_WDATA, value);  // pulses dcr_req_valid
}

// Drain new bytes from every hart's COUT ring to the UART.
static void cout_drain(u32 *rd_state) {
    for (u32 i = 0; i < COUT_SLOTS; ++i) {
        u32 wr = Xil_In32(COUT_WR(i));
        while (rd_state[i] != wr) {
            outbyte((char)Xil_In8(COUT_DATA(i, rd_state[i] % COUT_RING)));
            rd_state[i]++;
        }
        Xil_Out32(COUT_RD(i), rd_state[i]);
    }
}

// Wait until (busy == want) or timeout; drains COUT while polling.
static int wait_busy(u32 want, u32 timeout_s, u32 *rd_state) {
    XTime start, now;
    XTime_GetTime(&start);
    for (;;) {
        if ((Xil_In32(SHIM_STATUS) & 1u) == want)
            return 0;
        cout_drain(rd_state);
        XTime_GetTime(&now);
        if ((now - start) / COUNTS_PER_SECOND >= timeout_s)
            return -1;
    }
}

int main(void) {
    static u32 cout_rd[COUT_SLOTS];

    xil_printf("\r\n=== Vortex on Arty Z7 (Zynq-7000) bare-metal host ===\r\n");

    u32 magic = Xil_In32(SHIM_MAGIC);
    if (magic != SHIM_MAGIC_VAL) {
        xil_printf("FAIL: shim magic mismatch: got 0x%08x, want 0x%08x\r\n", magic, SHIM_MAGIC_VAL);
        return 1;
    }
    xil_printf("shim alive (magic ok), busy=%u\r\n", Xil_In32(SHIM_STATUS) & 1u);

    // Make the window Vortex accesses uncached on the A9 side: the S_AXI_HP
    // path is not coherent with the A9 caches. 1 MB MMU sections.
    for (u32 a = SHARED_BASE; a < SHARED_END; a += 0x100000u)
        Xil_SetTlbAttributes(a, NORM_NONCACHE);
    dsb();

    // Ensure Vortex is held in reset (it powers up that way).
    Xil_Out32(SHIM_CTRL, 1);

    u32 first_word = Xil_In32(KERNEL_BASE);
    if (first_word == 0u || first_word == 0xFFFFFFFFu) {
        xil_printf("FAIL: no kernel image at 0x%08x (first word 0x%08x). "
                   "Load it with: dow -data vecadd.bin 0x10000000\r\n",
                   KERNEL_BASE, first_word);
        return 1;
    }

    // Clear the IO region (COUT rings + exit code), seed the exit-code word.
    for (u32 a = IO_BASE; a < IO_BASE + IO_SIZE; a += 4)
        Xil_Out32(a, 0);
    Xil_Out32(EXIT_CODE_ADDR, 0xDEADBEEFu);
    for (u32 i = 0; i < COUT_SLOTS; ++i)
        cout_rd[i] = 0;
    dsb();

    // Program the KMU launch DCRs while in reset (hostless mode: kernel's
    // vx_start self-launches main; dims mirror sim/rtlsim/main.cpp).
    dcr_write(DCR_STARTUP_ADDR0, KERNEL_BASE);
    dcr_write(DCR_STARTUP_ADDR1, 0);
    dcr_write(DCR_STARTUP_ARG0,  0);
    dcr_write(DCR_STARTUP_ARG1,  0);
    dcr_write(DCR_GRID_DIM_X,    1);
    dcr_write(DCR_GRID_DIM_Y,    1);
    dcr_write(DCR_GRID_DIM_Z,    1);
    dcr_write(DCR_BLOCK_DIM_X,   1);
    dcr_write(DCR_BLOCK_DIM_Y,   1);
    dcr_write(DCR_BLOCK_DIM_Z,   1);
    dcr_write(DCR_LMEM_SIZE,     0);
    dcr_write(DCR_BLOCK_SIZE,    1);
    dcr_write(DCR_WARP_STEP_X,   VX_NUM_THREADS);
    dcr_write(DCR_WARP_STEP_Y,   0);
    dcr_write(DCR_WARP_STEP_Z,   0);
    dcr_write(DCR_CLUSTER_DIM_X, 1);
    dcr_write(DCR_CLUSTER_DIM_Y, 1);
    dcr_write(DCR_CLUSTER_DIM_Z, 1);
    dsb();

    xil_printf("releasing vortex reset...\r\n");
    Xil_Out32(SHIM_CTRL, 0);

    if (wait_busy(1, 5, cout_rd) != 0) {
        xil_printf("FAIL: busy never rose (kernel did not start)\r\n");
        return 1;
    }
    xil_printf("vortex running...\r\n");

    if (wait_busy(0, 120, cout_rd) != 0) {
        xil_printf("FAIL: timeout waiting for completion (busy stuck high)\r\n");
        return 1;
    }
    cout_drain(cout_rd);

    u32 exit_code = Xil_In32(EXIT_CODE_ADDR);
    if (exit_code == 0u) {
        xil_printf("\r\nPASS (exit=0)\r\n");
    } else {
        xil_printf("\r\nFAIL (exit=0x%08x)\r\n", exit_code);
    }
    return (exit_code == 0u) ? 0 : 1;
}
