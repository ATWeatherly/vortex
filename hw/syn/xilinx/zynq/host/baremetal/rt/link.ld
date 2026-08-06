/* Bare-metal Cortex-A9 host runtime for Vortex on Zynq-7000.
 *
 * Memory plan (physical == virtual, MMU flat-mapped by bm_platform.c):
 *   0x0010_0000  host image (text/data/bss), heap grows up to 0x0F00_0000
 *   0x0FFF_0000  initial stack top (grows down)
 *   0x1000_0000  Vortex device window (Device-mapped, uncached)
 *   0x2000_0000+ preloaded blobs (cacheable, host-only; see bm_blob_add)
 */
ENTRY(_start)

MEMORY
{
  DDR (rwx) : ORIGIN = 0x00100000, LENGTH = 0x0EF00000
}

SECTIONS
{
  .text : {
    KEEP(*(.vectors))
    *(.text*)
    *(.rodata*)
  } > DDR

  .init_array : ALIGN(4) {
    __init_array_start = .;
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    __init_array_end = .;
  } > DDR

  .fini_array : ALIGN(4) {
    __fini_array_start = .;
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))
    __fini_array_end = .;
  } > DDR

  .ARM.exidx : ALIGN(4) {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > DDR

  .data : ALIGN(4) {
    *(.data*)
  } > DDR

  /* L1 page table needs 16 KB alignment */
  .mmu_l1 (NOLOAD) : ALIGN(16384) {
    *(.mmu_l1)
  } > DDR

  .bss (NOLOAD) : ALIGN(8) {
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    __bss_end__ = .;
  } > DDR

  _end = .;
  __heap_start = .;
  __heap_end = 0x0F000000;
  __stack_top = 0x0FFF0000;
}
