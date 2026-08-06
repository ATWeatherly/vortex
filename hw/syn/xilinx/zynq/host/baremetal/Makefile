CROSS ?= arm-none-eabi-
CFLAGS = -mcpu=cortex-a9 -marm -O2 -Wall -Wextra -ffreestanding -nostdlib

all: vortex_host.elf

vortex_host.elf: start.S vortex_host.c link.ld
	$(CROSS)gcc $(CFLAGS) -T link.ld start.S vortex_host.c -o $@
	$(CROSS)size $@

clean:
	rm -f vortex_host.elf

.PHONY: all clean
