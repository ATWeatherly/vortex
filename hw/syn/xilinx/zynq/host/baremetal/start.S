// Minimal Cortex-A9 startup: mask interrupts, set the stack, zero .bss,
// call main, park. Loaded over JTAG (xsdb dow) after ps7_init, so clocks,
// DDR, and MIO are already configured and the MMU/caches are off.

    .section .vectors, "ax"
    .globl _start
_start:
    cpsid   if                  // mask IRQ/FIQ

    // Force MMU, D-cache, I-cache, branch predictor OFF. When the board
    // SD-boots U-Boot before JTAG takes over, dow/con inherit U-Boot's
    // SCTLR (MMU+caches enabled) — which makes every "uncached" access
    // below silently cacheable and breaks HP-port coherency with Vortex.
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #0x1        // M: MMU
    bic     r0, r0, #0x4        // C: D-cache
    bic     r0, r0, #0x1800     // I: I-cache, Z: branch predictor
    mcr     p15, 0, r0, c1, c0, 0
    dsb
    isb

    ldr     sp, =__stack_top

    // zero .bss
    ldr     r0, =__bss_start
    ldr     r1, =__bss_end
    mov     r2, #0
1:  cmp     r0, r1
    bge     2f
    str     r2, [r0], #4
    b       1b
2:
    bl      main
3:  wfe
    b       3b
