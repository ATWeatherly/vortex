// libvortex-lite: legacy Vortex host API over /dev/mem for the Arty Z7-10
// Vortex port (no Command Processor). The Linux side sees only the low 256 MB
// (mem=256M bootarg); the upper 256 MB (0x1000_0000..0x1FFF_FFFF) is the
// device world: kernel image + BSS, device buffers, and the IO/COUT region.
// Control goes through the vortex_axil_shim at 0x43C0_0000.
//
// Launch protocol (validated on silicon with vecadd): hold reset, write the
// KMU DCRs (STARTUP_ADDR = kernel image base, STARTUP_ARG = args pointer,
// boot-CTA dims 1/1/1), release reset, pulse START, wait busy rise+fall.
// The KMU delivers STARTUP_ARG into the warp's MSCRATCH (VX_scheduler.sv).
//
// Every vx_start re-uploads the kernel image and re-zeroes its BSS: each
// launch is a full GPU reset re-running crt0, and the flat image carries no
// BSS-zeroing of its own.
#include "vortex.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

// ---- physical layout ----
#define SHIM_PHYS       0x43C00000u
#define DEV_PHYS_BASE   0x10000000u
#define DEV_PHYS_SIZE   0x10000000u
#define KERNEL_BASE     0x10000000u          // kernel link address (STARTUP_ADDR)
#define ALLOC_BASE      0x11000000u          // device buffer pool start
#define ALLOC_END       0x1FEFFFFFu          // below the IO/COUT region
#define IO_BASE_OFF     (0x1FF00000u - DEV_PHYS_BASE)

// ---- shim registers ----
#define SHIM_CTRL       0x00                 // [0] vx_reset (POR=1)
#define SHIM_STATUS     0x04                 // [0] busy
#define SHIM_DCR_ADDR   0x08
#define SHIM_DCR_WDATA  0x0C                 // write pulses a DCR write
#define SHIM_START      0x14                 // write pulses vx_start
#define SHIM_MAGIC      0x1C
#define SHIM_MAGIC_VAL  0x56585A37u

// ---- KMU DCRs ----
#define DCR_STARTUP_ADDR0 0x010
#define DCR_STARTUP_ADDR1 0x011
#define DCR_STARTUP_ARG0  0x014
#define DCR_STARTUP_ARG1  0x015
#define DCR_BLOCK_DIM_X   0x016
#define DCR_GRID_DIM_X    0x019
#define DCR_LMEM_SIZE     0x01C
#define DCR_BLOCK_SIZE    0x01D
#define DCR_WARP_STEP_X   0x01E
#define DCR_CLUSTER_DIM_X 0x021

#define VX_NUM_CORES    1
#define VX_NUM_WARPS    2
#define VX_NUM_THREADS  2

struct Device {
    int fd = -1;
    volatile uint32_t* shim = nullptr;
    volatile uint8_t*  dev = nullptr;       // maps DEV_PHYS_BASE..+256MB
    uint64_t alloc_next = ALLOC_BASE;
    // cached kernel image for per-launch re-upload
    std::vector<uint8_t> kimg;
    uint64_t k_min_vma = 0, k_max_vma = 0;
};

struct Buffer {
    Device* dev;
    uint64_t addr;
    uint64_t size;
};

static void shim_wr(Device* d, uint32_t off, uint32_t v) {
    d->shim[off / 4] = v;
    __sync_synchronize();
}
static uint32_t shim_rd(Device* d, uint32_t off) {
    return d->shim[off / 4];
}
static void dcr_wr(Device* d, uint32_t addr, uint32_t v) {
    shim_wr(d, SHIM_DCR_ADDR, addr);
    shim_wr(d, SHIM_DCR_WDATA, v);
}

// word-wise copies: the /dev/mem mapping is Device-type memory; byte-lane
// access is legal but word access is ~4x faster and the sizes are 4-aligned
// in practice (fall back to bytes at the edges).
static void dev_memcpy_to(volatile uint8_t* dst, const uint8_t* src, uint64_t n) {
    uint64_t i = 0;
    if (((uintptr_t)dst & 3) == 0 && ((uintptr_t)src & 3) == 0) {
        for (; i + 4 <= n; i += 4)
            *(volatile uint32_t*)(dst + i) = *(const uint32_t*)(src + i);
    }
    for (; i < n; ++i) dst[i] = src[i];
}
static void dev_memcpy_from(uint8_t* dst, const volatile uint8_t* src, uint64_t n) {
    uint64_t i = 0;
    if (((uintptr_t)dst & 3) == 0 && ((uintptr_t)src & 3) == 0) {
        for (; i + 4 <= n; i += 4)
            *(uint32_t*)(dst + i) = *(const volatile uint32_t*)(src + i);
    }
    for (; i < n; ++i) dst[i] = src[i];
}
static void dev_memset(volatile uint8_t* dst, uint8_t v, uint64_t n) {
    uint64_t i = 0;
    uint32_t w = v * 0x01010101u;
    if (((uintptr_t)dst & 3) == 0) {
        for (; i + 4 <= n; i += 4) *(volatile uint32_t*)(dst + i) = w;
    }
    for (; i < n; ++i) dst[i] = v;
}

static uint64_t now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

extern "C" {

int vx_dev_open(vx_device_h* hdevice) {
    Device* d = new Device();
    d->fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (d->fd < 0) { perror("open /dev/mem"); delete d; return -1; }
    d->shim = (volatile uint32_t*)mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, d->fd, SHIM_PHYS);
    d->dev = (volatile uint8_t*)mmap(nullptr, DEV_PHYS_SIZE, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, d->fd, DEV_PHYS_BASE);
    if (d->shim == MAP_FAILED || d->dev == MAP_FAILED) {
        perror("mmap"); delete d; return -1;
    }
    if (shim_rd(d, SHIM_MAGIC) != SHIM_MAGIC_VAL) {
        fprintf(stderr, "libvortex-lite: shim magic mismatch (bitstream loaded?)\n");
        delete d; return -1;
    }
    shim_wr(d, SHIM_CTRL, 1);  // hold Vortex in reset until first launch
    *hdevice = d;
    return 0;
}

int vx_dev_close(vx_device_h hdevice) {
    Device* d = (Device*)hdevice;
    shim_wr(d, SHIM_CTRL, 1);
    close(d->fd);
    delete d;
    return 0;
}

int vx_dev_caps(vx_device_h hdevice, uint32_t caps_id, uint64_t* value) {
    (void)hdevice;
    switch (caps_id) {
    case VX_CAPS_NUM_THREADS: *value = VX_NUM_THREADS; break;
    case VX_CAPS_NUM_WARPS:   *value = VX_NUM_WARPS; break;
    case VX_CAPS_NUM_CORES:   *value = VX_NUM_CORES; break;
    case VX_CAPS_CACHE_LINE_SIZE: *value = 8; break;
    case VX_CAPS_GLOBAL_MEM_SIZE: *value = DEV_PHYS_SIZE; break;
    case VX_CAPS_LOCAL_MEM_SIZE:  *value = 16384; break;
    default: *value = 0; break;
    }
    return 0;
}

int vx_mem_alloc(vx_device_h hdevice, uint64_t size, int flags, vx_buffer_h* hbuffer) {
    (void)flags;
    Device* d = (Device*)hdevice;
    uint64_t addr = (d->alloc_next + 63) & ~63ull;
    if (addr + size > ALLOC_END) {
        fprintf(stderr, "libvortex-lite: device pool exhausted\n");
        return -1;
    }
    d->alloc_next = addr + size;
    Buffer* b = new Buffer{d, addr, size};
    *hbuffer = b;
    return 0;
}

int vx_mem_reserve(vx_device_h hdevice, uint64_t address, uint64_t size, int flags, vx_buffer_h* hbuffer) {
    (void)flags;
    Device* d = (Device*)hdevice;
    if (address < DEV_PHYS_BASE || address + size > DEV_PHYS_BASE + DEV_PHYS_SIZE) {
        fprintf(stderr, "libvortex-lite: reserve outside device window\n");
        return -1;
    }
    *hbuffer = new Buffer{d, address, size};
    return 0;
}

int vx_mem_free(vx_buffer_h hbuffer) {
    // bump allocator: individual frees are no-ops (llama2's buffers are
    // grow-only caches anyway)
    delete (Buffer*)hbuffer;
    return 0;
}

int vx_mem_address(vx_buffer_h hbuffer, uint64_t* address) {
    *address = ((Buffer*)hbuffer)->addr;
    return 0;
}

int vx_copy_to_dev(vx_buffer_h hbuffer, const void* host_ptr, uint64_t dst_offset, uint64_t size) {
    Buffer* b = (Buffer*)hbuffer;
    dev_memcpy_to(b->dev->dev + (b->addr - DEV_PHYS_BASE) + dst_offset,
                  (const uint8_t*)host_ptr, size);
    return 0;
}

int vx_copy_from_dev(void* host_ptr, vx_buffer_h hbuffer, uint64_t src_offset, uint64_t size) {
    Buffer* b = (Buffer*)hbuffer;
    dev_memcpy_from((uint8_t*)host_ptr,
                    b->dev->dev + (b->addr - DEV_PHYS_BASE) + src_offset, size);
    return 0;
}

int vx_upload_kernel_file(vx_device_h hdevice, const char* filename, vx_buffer_h* hbuffer) {
    Device* d = (Device*)hdevice;
    FILE* f = fopen(filename, "rb");
    if (!f) { perror(filename); return -1; }
    uint64_t hdr[2];
    if (fread(hdr, 8, 2, f) != 2) { fclose(f); return -1; }
    d->k_min_vma = hdr[0];
    d->k_max_vma = hdr[1];
    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 16, SEEK_SET);
    d->kimg.resize(fsz - 16);
    if (fread(d->kimg.data(), 1, d->kimg.size(), f) != d->kimg.size()) { fclose(f); return -1; }
    fclose(f);
    if (d->k_min_vma < DEV_PHYS_BASE ||
        d->k_max_vma > ALLOC_BASE) {
        fprintf(stderr, "libvortex-lite: kernel vma [0x%llx,0x%llx) outside window\n",
                (unsigned long long)d->k_min_vma, (unsigned long long)d->k_max_vma);
        return -1;
    }
    Buffer* b = new Buffer{d, d->k_min_vma, d->k_max_vma - d->k_min_vma};
    *hbuffer = b;
    return 0;
}

int vx_start(vx_device_h hdevice, vx_buffer_h hkernel, vx_buffer_h harguments) {
    Device* d = (Device*)hdevice;
    Buffer* kb = (Buffer*)hkernel;
    Buffer* ab = (Buffer*)harguments;

    shim_wr(d, SHIM_CTRL, 1);  // hold reset

    // fresh image every launch: payload + zeroed BSS tail
    uint64_t off = d->k_min_vma - DEV_PHYS_BASE;
    dev_memcpy_to(d->dev + off, d->kimg.data(), d->kimg.size());
    uint64_t bss = (d->k_max_vma - d->k_min_vma) - d->kimg.size();
    if (bss) dev_memset(d->dev + off + d->kimg.size(), 0, bss);

    dcr_wr(d, DCR_STARTUP_ADDR0, (uint32_t)kb->addr);
    dcr_wr(d, DCR_STARTUP_ADDR1, 0);
    dcr_wr(d, DCR_STARTUP_ARG0, (uint32_t)ab->addr);
    dcr_wr(d, DCR_STARTUP_ARG1, 0);
    for (int i = 0; i < 3; i++) {
        dcr_wr(d, DCR_GRID_DIM_X + i, 1);
        dcr_wr(d, DCR_BLOCK_DIM_X + i, 1);
        dcr_wr(d, DCR_CLUSTER_DIM_X + i, 1);
        dcr_wr(d, DCR_WARP_STEP_X + i, (i == 0) ? VX_NUM_THREADS : 0);
    }
    dcr_wr(d, DCR_LMEM_SIZE, 0);
    dcr_wr(d, DCR_BLOCK_SIZE, 1);

    shim_wr(d, SHIM_CTRL, 0);  // release reset
    usleep(10);                // >> RESET_DELAY(8) cycles @ 50 MHz
    shim_wr(d, SHIM_START, 1); // launch

    // busy must rise before ready_wait polls for fall
    uint64_t t0 = now_ms();
    while (!(shim_rd(d, SHIM_STATUS) & 1)) {
        if (now_ms() - t0 > 2000) {
            fprintf(stderr, "libvortex-lite: busy never rose after start\n");
            return -1;
        }
    }
    return 0;
}

int vx_ready_wait(vx_device_h hdevice, uint64_t timeout_ms) {
    Device* d = (Device*)hdevice;
    uint64_t t0 = now_ms();
    while (shim_rd(d, SHIM_STATUS) & 1) {
        if (now_ms() - t0 > timeout_ms) {
            fprintf(stderr, "libvortex-lite: kernel timeout (busy stuck)\n");
            return -1;
        }
    }
    return 0;
}

} // extern "C"
