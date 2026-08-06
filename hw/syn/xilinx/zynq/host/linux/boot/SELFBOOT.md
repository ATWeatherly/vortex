# Self-booting setup (no JTAG after power-on)

The board boots the full Vortex-Linux stack unattended. Mechanism: the
U-Boot environment in SPI flash (survives power cycles) replaces `bootcmd`
with `run vortexboot`, installed once via `saveenv`:

```
setenv sdbootdev 0
setenv partid 1
setenv vortexboot 'echo VORTEX self-boot; \
  mw.l 0xF8000008 0xDF0D; mw.l 0xF8000170 0x00100F00; mw.l 0xF8000004 0x767B; \
  setenv bootargs console=ttyPS0,115200 earlycon mem=256M modprobe.blacklist=smartio; \
  fatload mmc 0 0x8000000 vortex_z7.bit; fpga loadb 0 0x8000000 ${filesize}; \
  mw.l 0xF8000008 0xDF0D; mw.l 0xF8000900 0xF; mw.l 0xF8000240 0x0; mw.l 0xF8000004 0x767B; \
  fatload mmc 0 0x10000000 image.ub; bootm 0x10000000'
setenv bootcmd 'run vortexboot'
saveenv
```

Notes:
- The first `mw.l` triple sets FCLK0 = IOPLL/15 = 66.67 MHz (SLCR unlock /
  FPGA0_CLK_CTRL / relock) — keep in sync with the bitstream's `CLK_MHZ`.
- The second triple after `fpga loadb` is the `ps7_post_config` equivalent
  (LVL_SHFTR_EN = 0xF, FPGA_RST_CTRL = 0): **U-Boot's fpga command does not
  run post-config** ("INFO: post config was not run") and without it the PL
  is programmed but unreachable — the shim reads hang.
- `vortex_z7.bit` lives on the SD FAT partition; update it via the JTAG +
  `fatwrite` path (`boot_linux.py --put`) or any SD access. `image.ub` is
  the stock PetaLinux FIT already on the card.
- The stock `uEnv.txt` path was abandoned: the shipped `uenvboot` wrapper
  fails during autoboot (its `if test -n $uenvcmd` construct breaks on
  semicolon-laden variables, and its existence test depends on `sdbootdev`/
  `partid` that nothing defines on a fresh boot).
- Recovery: `run eraseenv` at a U-Boot prompt (Ctrl-C during the BOOTP loop
  or autoboot countdown) restores the shipped default environment.
- The JTAG flow (`boot_linux.py`) still works and overrides everything.
