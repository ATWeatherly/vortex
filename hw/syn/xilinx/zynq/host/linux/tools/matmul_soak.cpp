// matmul_soak: long-duration randomized stress of the llama2 matmul kernel
// through libvortex-lite — N launches with random shapes/data, each verified
// against a host-computed reference (ARM VFP) within relative tolerance
// (the device uses fused single-rounding soft-float, so bitwise equality is
// not expected). Exercises launches, the allocator free-list, and the
// uncached result mailbox back-to-back.
// Usage: matmul_soak <kernel.vxbin> [iters=1000] [seed=1]
#include "../vortex.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>

struct matmul_kernel_args_t {
    uint32_t grid_dim[2];
    uint32_t block_dim[2];
    uint64_t A_addr, B_addr, C_addr;
    int M, N, K;
    uint32_t use_tcu;
};

#define MAILBOX_ADDR 0x1FF09000ull
#define MAILBOX_MAX  0x7000ull

static uint32_t xs = 1;
static uint32_t rnd() { xs ^= xs << 13; xs ^= xs >> 17; xs ^= xs << 5; return xs; }
static float rndf() { return ((int)(rnd() % 2000) - 1000) / 250.0f; }

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <kernel.vxbin> [iters] [seed]\n", argv[0]); return 2; }
    int iters = (argc > 2) ? atoi(argv[2]) : 1000;
    xs = (argc > 3) ? atoi(argv[3]) : 1;

    vx_device_h dev;
    vx_buffer_h kbuf, mailbox;
    if (vx_dev_open(&dev)) return 2;
    if (vx_upload_kernel_file(dev, argv[1], &kbuf)) return 2;
    if (vx_mem_reserve(dev, MAILBOX_ADDR, MAILBOX_MAX, VX_MEM_READ, &mailbox)) return 2;

    const int MAXM = 512, MAXK = 256;
    float *A = new float[MAXM * MAXK], *B = new float[MAXK], *C = new float[MAXM], *R = new float[MAXM];
    int fails = 0;

    for (int it = 0; it < iters; ++it) {
        int M = 4 * (1 + rnd() % (MAXM / 4));
        int K = 1 + rnd() % MAXK;
        for (int i = 0; i < M * K; ++i) A[i] = rndf();
        for (int k = 0; k < K; ++k) B[k] = rndf();
        for (int i = 0; i < M; ++i) {
            float s = 0;
            for (int k = 0; k < K; ++k) s += A[i * K + k] * B[k];
            R[i] = s;
        }
        // fresh buffers each iteration to churn the allocator
        vx_buffer_h ab, bb, cb, argb;
        matmul_kernel_args_t args = {};
        args.M = M; args.N = 1; args.K = K;
        args.block_dim[0] = 4; args.block_dim[1] = 1;
        args.grid_dim[0] = (M + 3) / 4; args.grid_dim[1] = 1;
        if (vx_mem_alloc(dev, M * K * 4, VX_MEM_READ, &ab) ||
            vx_mem_alloc(dev, K * 4, VX_MEM_READ, &bb) ||
            vx_mem_alloc(dev, M * 4, VX_MEM_WRITE, &cb) ||
            vx_mem_alloc(dev, sizeof(args), VX_MEM_READ, &argb)) { fprintf(stderr, "alloc fail\n"); return 2; }
        vx_mem_address(ab, &args.A_addr);
        vx_mem_address(bb, &args.B_addr);
        vx_mem_address(cb, &args.C_addr);
        vx_copy_to_dev(ab, A, 0, M * K * 4);
        vx_copy_to_dev(bb, B, 0, K * 4);
        vx_copy_to_dev(argb, &args, 0, sizeof(args));
        if (vx_start(dev, kbuf, argb)) { fprintf(stderr, "start fail @%d\n", it); return 2; }
        if (vx_ready_wait(dev, 120000)) { fprintf(stderr, "timeout @%d M=%d K=%d\n", it, M, K); return 2; }
        vx_copy_from_dev(C, mailbox, 0, M * 4);
        for (int i = 0; i < M; ++i) {
            float d = fabsf(C[i] - R[i]);
            float tol = 1e-4f * (1.0f + fabsf(R[i]));
            if (d > tol) {
                if (fails < 5)
                    fprintf(stderr, "MISMATCH it=%d i=%d dev=%g ref=%g\n", it, i, C[i], R[i]);
                fails++;
                break;
            }
        }
        vx_mem_free(ab); vx_mem_free(bb); vx_mem_free(cb); vx_mem_free(argb);
        if ((it + 1) % 100 == 0) { printf("%d/%d ok (fails=%d)\n", it + 1, iters, fails); fflush(stdout); }
    }
    printf("SOAK %s: %d iters, %d fails\n", fails ? "FAIL" : "PASS", iters, fails);
    vx_dev_close(dev);
    return fails ? 1 : 0;
}
