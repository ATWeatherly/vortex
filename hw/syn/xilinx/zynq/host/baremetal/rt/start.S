/* Bare-metal Cortex-A9 startup for the Vortex Zynq host runtime.
 *
 * Unlike the minimal vecadd host's start.S (which forces caches OFF), this
 * runtime ENABLES MMU + caches for host memory and Device-maps the Vortex
 * window — bm_platform_init() builds the section table before the enable
 * here would matter: the sequence is
 *   sp -> known SCTLR state (all off) -> VFP on -> zero BSS
 *      -> bm_platform_init() (builds table, sets TTBR/DACR, enables
 *         MMU+caches itself via bm_mmu_enable) -> C runtime init -> main.
 */
    .syntax unified
    .arm
    .fpu vfpv3
    .section .vectors, "ax"
    .global _start
_start:
    cpsid   if                      /* mask IRQ/FIQ; no handlers installed */

    /* known state: MMU/caches/branch-predict off (JTAG entry may inherit
       anything from the previous occupant) */
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #0x1            /* M: MMU */
    bic     r0, r0, #0x4            /* C: dcache */
    bic     r0, r0, #0x1000         /* I: icache */
    bic     r0, r0, #0x800          /* Z: branch predict */
    mcr     p15, 0, r0, c1, c0, 0
    isb

    /* invalidate icache + TLBs + branch predictor */
    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0   /* ICIALLU */
    mcr     p15, 0, r0, c8, c7, 0   /* TLBIALL */
    mcr     p15, 0, r0, c7, c5, 6   /* BPIALL */
    dsb
    isb

    /* VFPv3 on (before any FP instruction anywhere) */
    mrc     p15, 0, r0, c1, c0, 2   /* CPACR */
    orr     r0, r0, #(0xF << 20)    /* CP10+CP11 full access */
    mcr     p15, 0, r0, c1, c0, 2
    isb
    mov     r0, #0x40000000
    vmsr    fpexc, r0               /* FPEXC.EN */

    ldr     sp, =__stack_top

    /* zero BSS */
    ldr     r0, =__bss_start__
    ldr     r1, =__bss_end__
    mov     r2, #0
1:  cmp     r0, r1
    strlo   r2, [r0], #4
    blo     1b

    bl      bm_platform_init        /* MMU table + enable, UART, timer */

    /* C++ static constructors */
    ldr     r4, =__init_array_start
    ldr     r5, =__init_array_end
2:  cmp     r4, r5
    beq     3f
    ldr     r3, [r4], #4
    blx     r3
    b       2b
3:
    bl      bm_main_shim            /* builds argv from bm_cmdline, calls main */

    /* fall through into exit with main's return code in r0 */
    .global _exit
_exit:
    bl      bm_exit_report          /* prints exit banner over UART */
4:  wfe
    b       4b

    .size _start, . - _start
