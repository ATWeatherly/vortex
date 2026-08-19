// memcp <phys_hex> <len> <outfile|->: copy physical memory to a file (or
// stdout) through a /dev/mem mmap.
//
// Used on the board to recover a payload that was staged into DDR over JTAG
// before Linux booted: read()/dd on /dev/mem fails for non-System-RAM
// ("Bad address"), but an mmap of the same physical range works.
//
// Cross-build (static, so it runs on any rootfs):
//   arm-linux-musleabihf-gcc -O2 -static -o memcp memcp.c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

int main(int argc, char** argv) {
    if (argc != 4) { fprintf(stderr, "usage: memcp <phys_hex> <len> <out|->\n"); return 1; }
    unsigned long phys = strtoul(argv[1], 0, 16);
    unsigned long len  = strtoul(argv[2], 0, 0);
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }
    unsigned long off = phys & 0xFFF;               // mmap needs page alignment
    void* m = mmap(0, len + off, PROT_READ, MAP_SHARED, fd, phys - off);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }
    FILE* out = (argv[3][0] == '-' && !argv[3][1]) ? stdout : fopen(argv[3], "wb");
    if (!out) { perror(argv[3]); return 1; }
    if (fwrite((char*)m + off, 1, len, out) != len) { perror("fwrite"); return 1; }
    if (out != stdout) fclose(out);
    return 0;
}
