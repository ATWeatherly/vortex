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

// Freestanding bare-metal Cortex-A9 host for Vortex on Zynq-7000 (Arty Z7).
// No BSP: drives PS UART0 directly and keeps the MMU/caches off, so all
// memory the A9 touches is uncached — trivially coherent with Vortex's
// S_AXI_HP0 traffic.
//
// Expects the kernel's flat image (llvm-objcopy -O binary of a hostless-mode
// ELF linked at KERNEL_BASE) preloaded at KERNEL_BASE by the xsdb run script.
//
// Launch protocol (per VX_kmu.sv: `busy = running`, set only by the `start`
// pulse): release Vortex's reset, program the KMU DCRs, pulse START, wait
// busy rise then fall, read the exit-code word the kernel's _Exit stored.

#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define REG8(a)  (*(volatile uint8_t *)(a))

// ---- vortex_axil_shim register file (M_AXI_GP0) ----
#define SHIM_BASE       0x43C00000u
#define SHIM_CTRL       (SHIM_BASE + 0x00)  // [0] vx_reset (resets to 1)
#define SHIM_STATUS     (SHIM_BASE + 0x04)  // [0] busy
#define SHIM_DCR_ADDR   (SHIM_BASE + 0x08)
#define SHIM_DCR_WDATA  (SHIM_BASE + 0x0C)  // write pulses a DCR write
#define SHIM_START      (SHIM_BASE + 0x14)  // write pulses vx_start (KMU launch)
#define SHIM_MAGIC      (SHIM_BASE + 0x1C)
#define SHIM_MAGIC_VAL  0x56585A37u

// ---- device memory map (must match VX_MEM_* overrides + kernel link base) ----
#define KERNEL_BASE     0x10000000u
#define IO_BASE         0x1FF00000u
#define IO_SIZE         0x00010000u
#define EXIT_CODE_ADDR  0x1FF08308u

// COUT stream rings @ IO_BASE: wr[64] u32, rd[64] u32, data[64][512], lost[64] u32
#define COUT_SLOTS      64u
#define COUT_RING       512u
#define COUT_WR(i)      (IO_BASE + 4u * (i))
#define COUT_RD(i)      (IO_BASE + 4u * (COUT_SLOTS + (i)))
#define COUT_DATA(i, k) (IO_BASE + 8u * COUT_SLOTS + (i) * COUT_RING + (k))

// ---- KMU DCR addresses (build/sw/VX_types.h) ----
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

#ifndef VX_NUM_THREADS
#define VX_NUM_THREADS  2  /* must match -DVX_CFG_NUM_THREADS of the bitstream */
#endif

// ---- PS UART0 (MIO14/15, 115200; ref clock 100 MHz per PCW config) ----
#ifndef UART_BASE
#define UART_BASE   0xE0000000u   /* Arty Z7: UART0; Zybo Z7: build with -DUART_BASE=0xE0001000 (UART1) */
#endif
#define UART_CR     (UART_BASE + 0x00)
#define UART_MR     (UART_BASE + 0x04)
#define UART_BAUDGEN (UART_BASE + 0x18)
#define UART_SR     (UART_BASE + 0x2C)
#define UART_FIFO   (UART_BASE + 0x30)
#define UART_BAUDDIV (UART_BASE + 0x34)
#define UART_SR_TXFULL (1u << 4)

// ---- A9 global timer (CPU_3x2x clock = 325 MHz for a 650 MHz APU) ----
#define GT_BASE     0xF8F00200u
#define GT_LO       (GT_BASE + 0x00)
#define GT_HI       (GT_BASE + 0x04)
#define GT_CTRL     (GT_BASE + 0x08)
#define GT_TICKS_PER_S 325000000ull

static void dmb_(void) { __asm__ volatile("dmb" ::: "memory"); }

static void uart_init(void) {
    REG32(UART_CR) = 0x3;                  // reset TX/RX paths
    while (REG32(UART_CR) & 0x3) {}
    REG32(UART_MR) = 0x20;                 // 8N1, ref clock
    REG32(UART_BAUDGEN) = 124;             // 100 MHz / (124 * (6+1)) = 115207
    REG32(UART_BAUDDIV) = 6;
    REG32(UART_CR) = 0x14;                 // enable TX + RX
}

static void uart_putc(char c) {
    while (REG32(UART_SR) & UART_SR_TXFULL) {}
    REG32(UART_FIFO) = (uint32_t)(uint8_t)c;
}

static void uart_puts(const char *s) {
    for (; *s; ++s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s);
    }
}

static void uart_puthex(uint32_t v) {
    uart_puts("0x");
    for (int i = 28; i >= 0; i -= 4)
        uart_putc("0123456789abcdef"[(v >> i) & 0xF]);
}

static uint64_t gt_read(void) {
    uint32_t hi, lo;
    do {
        hi = REG32(GT_HI);
        lo = REG32(GT_LO);
    } while (REG32(GT_HI) != hi);
    return ((uint64_t)hi << 32) | lo;
}

static void gt_init(void) { REG32(GT_CTRL) = 1; }

static void dcr_write(uint32_t addr, uint32_t value) {
    REG32(SHIM_DCR_ADDR) = addr;
    dmb_();
    REG32(SHIM_DCR_WDATA) = value;  // pulses dcr_req_valid
    dmb_();
}

static void cout_drain(uint32_t *rd_state) {
    for (uint32_t i = 0; i < COUT_SLOTS; ++i) {
        uint32_t wr = REG32(COUT_WR(i));
        while (rd_state[i] != wr) {
            char c = (char)REG8(COUT_DATA(i, rd_state[i] % COUT_RING));
            if (c == '\n') uart_putc('\r');
            uart_putc(c);
            rd_state[i]++;
        }
        REG32(COUT_RD(i)) = rd_state[i];
    }
}

// Wait until (busy == want) or timeout; drains COUT while polling.
static int wait_busy(uint32_t want, uint32_t timeout_s, uint32_t *rd_state) {
    uint64_t start = gt_read();
    for (;;) {
        if ((REG32(SHIM_STATUS) & 1u) == want)
            return 0;
        cout_drain(rd_state);
        if ((gt_read() - start) >= (uint64_t)timeout_s * GT_TICKS_PER_S)
            return -1;
    }
}

static uint32_t cout_rd[COUT_SLOTS];

int main(void) {
    uart_init();
    gt_init();

    uart_puts("\n=== Vortex on Arty Z7 (Zynq-7000) bare-metal host ===\n");

    uint32_t magic = REG32(SHIM_MAGIC);
    if (magic != SHIM_MAGIC_VAL) {
        uart_puts("FAIL: shim magic mismatch: got ");
        uart_puthex(magic);
        uart_puts("\n");
        return 1;
    }
    uart_puts("shim alive (magic ok)\n");

    // Ensure Vortex is held in reset (it powers up that way).
    REG32(SHIM_CTRL) = 1;
    dmb_();

    uint32_t first_word = REG32(KERNEL_BASE);
    if (first_word == 0u || first_word == 0xFFFFFFFFu) {
        uart_puts("FAIL: no kernel image at ");
        uart_puthex(KERNEL_BASE);
        uart_puts(" — load it with: dow -data vecadd.bin 0x10000000\n");
        return 1;
    }

    // Clear the IO region (COUT rings + exit code), seed the exit-code word.
    for (uint32_t a = IO_BASE; a < IO_BASE + IO_SIZE; a += 4)
        REG32(a) = 0;
    REG32(EXIT_CODE_ADDR) = 0xDEADBEEFu;
    for (uint32_t i = 0; i < COUT_SLOTS; ++i)
        cout_rd[i] = 0;
    dmb_();

    // Bring Vortex out of reset (KMU stays idle until the START pulse), then
    // program the launch DCRs (hostless mode; dims mirror sim/rtlsim/main.cpp).
    uart_puts("releasing vortex reset...\n");
    REG32(SHIM_CTRL) = 0;
    dmb_();
    for (volatile int i = 0; i < 1000; ++i) {}  // >> VX_CFG_RESET_DELAY(8) cycles @50MHz

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

    uart_puts("pulsing start...\n");
    REG32(SHIM_START) = 1;
    dmb_();

    if (wait_busy(1, 5, cout_rd) != 0) {
        uart_puts("FAIL: busy never rose (kernel did not start)\n");
        return 1;
    }
    uart_puts("vortex running...\n");

    if (wait_busy(0, 120, cout_rd) != 0) {
        uart_puts("FAIL: timeout waiting for completion (busy stuck high)\n");
        return 1;
    }
    cout_drain(cout_rd);

    uint32_t exit_code = REG32(EXIT_CODE_ADDR);
    if (exit_code == 0u) {
        uart_puts("\nPASS (exit=0)\n");
    } else {
        uart_puts("\nFAIL (exit=");
        uart_puthex(exit_code);
        uart_puts(")\n");
    }
    return (exit_code == 0u) ? 0 : 1;
}
