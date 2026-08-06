CROSS ?= arm-none-eabi-
# BOARD=zybo-z7-20 selects PS UART1; default is Arty Z7 UART0.
BOARD ?= arty-z7-10
ifeq ($(BOARD),zybo-z7-20)
BOARD_DEFS = -DUART_BASE=0xE0001000u
else
BOARD_DEFS =
endif
VX_NUM_THREADS ?= 2
CFLAGS = -mcpu=cortex-a9 -marm -O2 -Wall -Wextra -ffreestanding -nostdlib $(BOARD_DEFS) -DVX_NUM_THREADS=$(VX_NUM_THREADS)

all: vortex_host.elf

vortex_host.elf: start.S vortex_host.c link.ld
	$(CROSS)gcc $(CFLAGS) -T link.ld start.S vortex_host.c -o $@
	$(CROSS)size $@

clean:
	rm -f vortex_host.elf

.PHONY: all clean
