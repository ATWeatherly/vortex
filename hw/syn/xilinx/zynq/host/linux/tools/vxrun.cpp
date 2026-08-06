// vxrun: launch a hostless Vortex kernel (.vxbin) from Linux via
// libvortex-lite, stream its COUT rings to stdout, and exit with the
// kernel's exit code. Usage: vxrun <kernel.vxbin> [timeout_s]
#include "../vortex.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#define IO_BASE   0x1FF00000ull
#define IO_SIZE   0x10000ull
#define EXIT_OFF  0x8308ull
#define SLOTS     64u
#define RING      512u

static uint8_t io[IO_SIZE];

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <kernel.vxbin> [timeout_s]\n", argv[0]); return 2; }
    unsigned timeout_s = (argc > 2) ? atoi(argv[2]) : 300;

    vx_device_h dev;
    vx_buffer_h kbuf, iobuf;
    if (vx_dev_open(&dev)) return 2;
    if (vx_upload_kernel_file(dev, argv[1], &kbuf)) return 2;
    if (vx_mem_reserve(dev, IO_BASE, IO_SIZE, VX_MEM_READ_WRITE, &iobuf)) return 2;

    // clear IO region, seed the exit word
    memset(io, 0, sizeof(io));
    io[EXIT_OFF] = 0xEF; io[EXIT_OFF+1] = 0xBE; io[EXIT_OFF+2] = 0xAD; io[EXIT_OFF+3] = 0xDE;
    vx_copy_to_dev(iobuf, io, 0, IO_SIZE);

    if (vx_start(dev, kbuf, nullptr)) return 2;   // hostless: MSCRATCH = 0

    uint32_t rd[SLOTS] = {0};
    unsigned waited_ms = 0;
    bool done = false;
    while (!done) {
        // poll busy via a cheap 1ms ready-wait; drain COUT between polls
        done = (vx_ready_wait(dev, 1) == 0);
        vx_copy_from_dev(io, iobuf, 0, 8 * SLOTS + SLOTS * RING);
        const uint32_t* wr = (const uint32_t*)io;
        for (uint32_t s = 0; s < SLOTS; ++s) {
            while (rd[s] != wr[s]) {
                putchar(io[8 * SLOTS + s * RING + (rd[s] % RING)]);
                rd[s]++;
            }
        }
        fflush(stdout);
        if (!done && ++waited_ms > timeout_s * 1000u) {
            fprintf(stderr, "vxrun: timeout\n");
            return 3;
        }
    }
    vx_copy_from_dev(io, iobuf, 0, IO_SIZE);
    uint32_t code;
    memcpy(&code, io + EXIT_OFF, 4);
    fprintf(stderr, "vxrun: exit code 0x%08x\n", code);
    vx_dev_close(dev);
    return (code == 0) ? 0 : 1;
}
