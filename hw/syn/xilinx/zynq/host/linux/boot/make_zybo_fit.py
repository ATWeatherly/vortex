#!/usr/bin/env python3
# Build a bootable FIT for the JTAG-Linux flow on the Zybo Z7-20 from
# Digilent's PetaLinux 2017.4 prebuilt image.ub.
#
# Two transformations are required (both learned the hard way — see
# CAVEATS.md section 9):
#   1. Strip the `amba_pl` node from their device tree. It describes THEIR
#      PL design; the video timing controller sits at 0x43C00000, which is
#      our Vortex control window, and probing it raises an imprecise
#      external abort that kills init ("Attempted to kill init!").
#   2. Slim the 144 MB initramfs (it carries a full on-target GCC). The
#      rootfs is a tmpfs, so the original does not fit alongside Linux at
#      mem=240M and the kernel panics with OOM.
#
# Needs only dtc (no mkimage/u-boot-tools).
#
# Usage: make_zybo_fit.py [<image.ub>] [<outdir>]

import os, re, struct, subprocess, sys, gzip

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/tools/zybo-z7-20-petalinux/Zybo-Z7-20/pre-built/linux/images/image.ub")
OUT = sys.argv[2] if len(sys.argv) > 2 else "fit"
os.makedirs(OUT, exist_ok=True)

# --- 1. pull the subimages out of the source FIT (it is just a DTB) --------
b = open(SRC, "rb").read()
magic, _, off_struct, off_strings = struct.unpack(">IIII", b[:16])
assert magic == 0xd00dfeed, "not a FIT/DTB"

def cstr(o):
    return b[o:b.index(b"\0", o)].decode()

p, path, subimages = off_struct, [], {}
while True:
    tok = struct.unpack(">I", b[p:p+4])[0]; p += 4
    if tok == 1:                                   # BEGIN_NODE
        name = cstr(p); p += len(name) + 1; p = (p + 3) & ~3
        path.append(name)
    elif tok == 2:                                 # END_NODE
        path.pop()
    elif tok == 3:                                 # PROP
        ln, nameoff = struct.unpack(">II", b[p:p+8]); p += 8
        if cstr(off_strings + nameoff) == "data" and len(path) >= 2:
            subimages[path[-1]] = b[p:p+ln]
        p += ln; p = (p + 3) & ~3
    elif tok == 9:                                 # END
        break

def pick(prefix):
    for k, v in subimages.items():
        if k.startswith(prefix):
            return v
    raise SystemExit(f"no '{prefix}' subimage in {SRC}")

kernel, fdt, ramdisk = pick("kernel"), pick("fdt"), pick("ramdisk")
open(f"{OUT}/kernel.bin", "wb").write(kernel)
print(f"kernel  {len(kernel)/1e6:7.2f} MB")

# --- 2. strip amba_pl from the device tree --------------------------------
open(f"{OUT}/system.dtb", "wb").write(fdt)
dts = subprocess.run(["dtc", "-I", "dtb", "-O", "dts", f"{OUT}/system.dtb"],
                     capture_output=True, text=True).stdout
i = dts.index("\tamba_pl {")
depth, j = 0, i
while True:                                        # find the matching brace
    if dts[j] == "{": depth += 1
    elif dts[j] == "}":
        depth -= 1
        if depth == 0: break
    j += 1
j = dts.index(";", j) + 1
open(f"{OUT}/system_nopl.dts", "w").write(dts[:i] + dts[j:])
subprocess.run(["dtc", "-I", "dts", "-O", "dtb", "-o", f"{OUT}/system_nopl.dtb",
                f"{OUT}/system_nopl.dts"], check=True, capture_output=True)
print(f"dtb     stripped amba_pl ({j-i} bytes of PL nodes removed)")

# --- 3. slim the initramfs (stream-filter the cpio, no extraction) --------
DROP = ("usr/libexec/gcc", "usr/lib/arm-xilinx-linux-gnueabi", "usr/include",
        "usr/bin/arm-xilinx-linux-gnueabi-", "usr/lib/perl", "usr/share/autoconf",
        "usr/share/gettext", "usr/lib/libperl", "usr/share/automake",
        "usr/share/doc", "usr/share/man", "usr/share/info", "usr/share/X11",
        "usr/lib/libX11", "usr/lib/gconv")
c, out, p, kept, dropped = gzip.decompress(ramdisk), bytearray(), 0, 0, 0
while p < len(c):
    start = p
    fsize = int(c[p+54:p+62], 16)
    nsize = int(c[p+94:p+102], 16)
    name  = c[p+110:p+110+nsize-1].decode()
    p = (p + 110 + nsize + 3) & ~3
    p = (p + fsize + 3) & ~3
    if name == "TRAILER!!!":
        out += c[start:p]; break
    if any(name.startswith(d) for d in DROP):
        dropped += fsize
    else:
        out += c[start:p]; kept += fsize
open(f"{OUT}/rootfs_slim.cpio.gz", "wb").write(gzip.compress(bytes(out), 9))
print(f"initramfs {kept/1e6:5.1f} MB kept, {dropped/1e6:5.1f} MB dropped")

# --- 4. reassemble ---------------------------------------------------------
open(f"{OUT}/image_slim.its", "w").write('''/dts-v1/;
/ {
    description = "Vortex Zybo JTAG-boot: Digilent kernel + slim initramfs, PL-free DTB";
    #address-cells = <1>;
    images {
        kernel@0 { description = "Linux Kernel"; data = /incbin/("kernel.bin");
                   type = "kernel"; arch = "arm"; os = "linux";
                   compression = "none"; load = <0x8000>; entry = <0x8000>; };
        fdt@0    { description = "FDT (amba_pl removed)"; data = /incbin/("system_nopl.dtb");
                   type = "flat_dt"; arch = "arm"; compression = "none"; };
        ramdisk@0{ description = "slim initramfs"; data = /incbin/("rootfs_slim.cpio.gz");
                   type = "ramdisk"; arch = "arm"; os = "linux"; compression = "none"; };
    };
    configurations {
        default = "conf@1";
        conf@1 { description = "kernel + fdt + slim ramdisk";
                 kernel = "kernel@0"; fdt = "fdt@0"; ramdisk = "ramdisk@0"; };
    };
};
''')
subprocess.run(["dtc", "-I", "dts", "-O", "dtb", "-o", f"{OUT}/image_slim.ub",
                f"{OUT}/image_slim.its"], check=True, capture_output=True, cwd=".")
print(f"\n=> {OUT}/image_slim.ub  ({os.path.getsize(f'{OUT}/image_slim.ub')/1e6:.1f} MB)")
