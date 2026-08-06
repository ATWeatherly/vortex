/* Bare-metal Vortex host for Zynq-7000: runs from PS DDR at 0x00100000
 * (below the Vortex window at 0x10000000+; the low 1 MB is skipped because
 * it is remappable/OCM-shadowed). 64 KB stack. */
ENTRY(_start)

MEMORY {
    RAM (rwx) : ORIGIN = 0x00100000, LENGTH = 0x00100000
}

SECTIONS {
    . = ORIGIN(RAM);
    .text : { KEEP(*(.vectors)) *(.text*) } > RAM
    .rodata : { *(.rodata*) } > RAM
    .data : { *(.data*) } > RAM
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM
    . = ALIGN(8);
    . += 0x10000;
    __stack_top = .;
}
