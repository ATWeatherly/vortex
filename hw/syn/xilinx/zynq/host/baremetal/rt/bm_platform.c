// Bare-metal Cortex-A9 host runtime for Vortex on Zynq-7000: MMU + caches,
// PS UART stdio, A9 global-timer clocks, newlib syscalls with blob-backed
// file IO, and an mmap shim. See bm_platform.h for the application contract.
//
// MMU map (flat, 1 MB sections, single domain in manager mode):
//   0x0000_0000-0x0FFF_FFFF  Normal WB/WA cacheable  (host image/heap/stack)
//   0x1000_0000-0x1FFF_FFFF  Device                  (Vortex window — must
//                                                     stay coherent with the
//                                                     PL's S_AXI_HP0 traffic)
//   0x2000_0000-0x3FFF_FFFF  Normal WB/WA cacheable  (preloaded blobs, host-only)
//   0x4000_0000-0xFFFF_FFFF  Device                  (PL + PS peripherals)
//
// Host-cached regions never alias Vortex traffic, so coherence holds by
// construction — no flush/invalidate calls anywhere.

#include "bm_platform.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

#define REG32(a) (*(volatile uint32_t *)(a))

// ---- PS UART (default: Zybo Z7 UART1; override with -DBM_UART_BASE=...) ----
#ifndef BM_UART_BASE
#define BM_UART_BASE 0xE0001000u
#endif
#define UART_CR      (BM_UART_BASE + 0x00)
#define UART_MR      (BM_UART_BASE + 0x04)
#define UART_BAUDGEN (BM_UART_BASE + 0x18)
#define UART_SR      (BM_UART_BASE + 0x2C)
#define UART_FIFO    (BM_UART_BASE + 0x30)
#define UART_BAUDDIV (BM_UART_BASE + 0x34)
#define UART_SR_TXFULL (1u << 4)

// ---- A9 global timer (CPU_3x2x = CPU/2; Zybo Z7-20: 666.67/2 MHz) ----
#ifndef BM_GT_HZ
#define BM_GT_HZ 333333333ull
#endif
#define GT_LO   0xF8F00200u
#define GT_HI   0xF8F00204u
#define GT_CTRL 0xF8F00208u

// ------------------------------------------------------------------ UART --
static void uart_init(void) {
    REG32(UART_CR) = 0x3;
    while (REG32(UART_CR) & 0x3) {}
    REG32(UART_MR) = 0x20;       // 8N1, ref clock (100 MHz per PCW config)
    REG32(UART_BAUDGEN) = 124;   // 100 MHz / (124 * (6+1)) = 115207
    REG32(UART_BAUDDIV) = 6;
    REG32(UART_CR) = 0x14;       // TX + RX enable
}

static void uart_putc(char c) {
    while (REG32(UART_SR) & UART_SR_TXFULL) {}
    REG32(UART_FIFO) = (uint32_t)(uint8_t)c;
}

// ----------------------------------------------------------------- timer --
static uint64_t gt_read(void) {
    uint32_t hi, lo;
    do {
        hi = REG32(GT_HI);
        lo = REG32(GT_LO);
    } while (REG32(GT_HI) != hi);
    return ((uint64_t)hi << 32) | lo;
}

uint64_t bm_now_ms(void) { return gt_read() / (BM_GT_HZ / 1000ull); }

void bm_udelay(uint32_t us) {
    uint64_t end = gt_read() + (BM_GT_HZ / 1000000ull) * us;
    while (gt_read() < end) {}
}

// ------------------------------------------------------------------- MMU --
static uint32_t l1_table[4096] __attribute__((section(".mmu_l1"), aligned(16384)));

#define SEC_NORMAL 0x11C0Eu  // section, TEX=001 C=1 B=1 (WB/WA), AP=11, S=1
#define SEC_DEVICE 0x00C16u  // section, TEX=000 C=0 B=1 (shareable device), AP=11, XN

static void mmu_setup(void) {
    for (uint32_t i = 0; i < 4096; ++i) {
        uint32_t attr;
        if (i < 0x100)      attr = SEC_NORMAL;  // host low 256 MB
        else if (i < 0x200) attr = SEC_DEVICE;  // Vortex window
        else if (i < 0x400) attr = SEC_NORMAL;  // blob region
        else                attr = SEC_DEVICE;  // peripherals
        l1_table[i] = (i << 20) | attr;
    }

    __asm__ volatile(
        "mcr p15, 0, %0, c2, c0, 0\n"   // TTBR0
        "mov r0, #0\n"
        "mcr p15, 0, r0, c2, c0, 2\n"   // TTBCR = 0
        "mvn r0, #0\n"
        "mcr p15, 0, r0, c3, c0, 0\n"   // DACR: all domains manager
        "mcr p15, 0, r0, c8, c7, 0\n"   // TLBIALL (r0 value ignored)
        "dsb\n"
        "isb\n"
        "mrc p15, 0, r0, c1, c0, 0\n"
        "orr r0, r0, #0x1\n"            // M
        "orr r0, r0, #0x4\n"            // C
        "orr r0, r0, #0x1000\n"         // I
        "orr r0, r0, #0x800\n"          // Z
        "mcr p15, 0, r0, c1, c0, 0\n"
        "dsb\n"
        "isb\n"
        :: "r"((uint32_t)(uintptr_t)l1_table) : "r0", "memory");
}

// ------------------------------------------------------------ blob table --
typedef struct {
    const char* path;
    uint8_t*    addr;
    size_t      size;
} Blob;

#define MAX_BLOBS 8
static Blob   blobs[MAX_BLOBS];
static int    num_blobs;

void bm_blob_add(const char* path, void* addr, size_t size) {
    if (num_blobs < MAX_BLOBS)
        blobs[num_blobs++] = (Blob){path, (uint8_t*)addr, size};
}

// basename-tolerant match: "data/model.bin" matches a blob named "model.bin"
static int blob_find(const char* path) {
    for (int i = 0; i < num_blobs; ++i) {
        const char* bp = blobs[i].path;
        size_t pl = strlen(path), bl = strlen(bp);
        if (pl >= bl && strcmp(path + pl - bl, bp) == 0)
            return i;
        if (bl >= pl && strcmp(bp + bl - pl, path) == 0)
            return i;
    }
    return -1;
}

// --------------------------------------------------------------- fd table --
typedef struct {
    int    blob;   // index into blobs[], or -1 free
    size_t pos;
} Fd;

#define MAX_FDS 16
#define FD_BASE 3
static Fd fds[MAX_FDS];

// ------------------------------------------------------- newlib syscalls --
extern char __heap_start, __heap_end;
static char* brk_cur = &__heap_start;

void* _sbrk(ptrdiff_t incr) {
    char* prev = brk_cur;
    if (brk_cur + incr > &__heap_end) {
        errno = ENOMEM;
        return (void*)-1;
    }
    brk_cur += incr;
    return prev;
}

int _write(int fd, const void* buf, size_t n) {
    (void)fd;
    const char* p = (const char*)buf;
    for (size_t i = 0; i < n; ++i) {
        if (p[i] == '\n') uart_putc('\r');
        uart_putc(p[i]);
    }
    return (int)n;
}

int _open(const char* path, int flags, ...) {
    (void)flags;
    int bi = blob_find(path);
    if (bi < 0) { errno = ENOENT; return -1; }
    for (int i = 0; i < MAX_FDS; ++i) {
        if (fds[i].blob < 0) {
            fds[i].blob = bi;
            fds[i].pos = 0;
            return FD_BASE + i;
        }
    }
    errno = EMFILE;
    return -1;
}

static Fd* fd_get(int fd) {
    int i = fd - FD_BASE;
    if (i < 0 || i >= MAX_FDS || fds[i].blob < 0) return 0;
    return &fds[i];
}

int _read(int fd, void* buf, size_t n) {
    Fd* f = fd_get(fd);
    if (!f) { errno = EBADF; return -1; }
    Blob* b = &blobs[f->blob];
    size_t avail = b->size - f->pos;
    if (n > avail) n = avail;
    memcpy(buf, b->addr + f->pos, n);
    f->pos += n;
    return (int)n;
}

int _close(int fd) {
    Fd* f = fd_get(fd);
    if (!f) { errno = EBADF; return -1; }
    f->blob = -1;
    return 0;
}

off_t _lseek(int fd, off_t off, int whence) {
    Fd* f = fd_get(fd);
    if (!f) { errno = EBADF; return -1; }
    Blob* b = &blobs[f->blob];
    long base = (whence == SEEK_SET) ? 0
              : (whence == SEEK_CUR) ? (long)f->pos
              : (long)b->size;
    long np = base + (long)off;
    if (np < 0 || np > (long)b->size) { errno = EINVAL; return -1; }
    f->pos = (size_t)np;
    return (off_t)np;
}

int _fstat(int fd, struct stat* st) {
    memset(st, 0, sizeof(*st));
    Fd* f = fd_get(fd);
    if (f) {
        st->st_mode = S_IFREG;
        st->st_size = (off_t)blobs[f->blob].size;
    } else {
        st->st_mode = S_IFCHR;
    }
    return 0;
}

int _isatty(int fd) { return fd >= 0 && fd < FD_BASE; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _getpid(void) { return 1; }

int _gettimeofday(struct timeval* tv, void* tz) {
    (void)tz;
    uint64_t us = gt_read() / (BM_GT_HZ / 1000000ull);
    tv->tv_sec = (time_t)(us / 1000000ull);
    tv->tv_usec = (suseconds_t)(us % 1000000ull);
    return 0;
}

// llama.cpp calls clock_gettime(CLOCK_REALTIME) directly.
int clock_gettime(clockid_t clk, struct timespec* ts) {
    (void)clk;
    uint64_t t = gt_read();
    ts->tv_sec = (time_t)(t / BM_GT_HZ);
    ts->tv_nsec = (long)((t % BM_GT_HZ) * (1000000000ull / 100) / (BM_GT_HZ / 100));
    return 0;
}

// ------------------------------------------------------------- mmap shim --
void* mmap(void* addr, size_t length, int prot, int flags, int fd, off_t offset) {
    (void)addr; (void)prot; (void)flags;
    Fd* f = fd_get(fd);
    if (!f || offset != 0) return (void*)-1;
    Blob* b = &blobs[f->blob];
    if (length > b->size) return (void*)-1;
    return b->addr;
}

int munmap(void* addr, size_t length) { (void)addr; (void)length; return 0; }

// ---- crt glue normally provided by the startfiles we exclude ----
void* __dso_handle __attribute__((visibility("hidden"))) = &__dso_handle;
void _init(void) {}
void _fini(void) {}

// ------------------------------------------------------------- app entry --
__attribute__((weak)) const char* bm_cmdline = "app";
__attribute__((weak)) void bm_app_config(void) {}

void bm_platform_init(void) {
    for (int i = 0; i < MAX_FDS; ++i) fds[i].blob = -1;
    mmu_setup();
    uart_init();
    REG32(GT_CTRL) = 1;
}

extern int main(int argc, char** argv);

int bm_main_shim(void) {
    static char cmdbuf[512];
    static char* argv[24];
    strncpy(cmdbuf, bm_cmdline, sizeof(cmdbuf) - 1);
    int argc = 0;
    char* p = cmdbuf;
    while (*p && argc < 23) {
        while (*p == ' ') *p++ = 0;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ') p++;
    }
    argv[argc] = 0;
    bm_app_config();
    _write(1, "\n[bm] runtime up: MMU+caches on, VFPv3 on\n", 42);
    return main(argc, argv);
}

void bm_exit_report(int code) {
    char msg[40];
    int n = 0;
    const char* pre = "\n[bm] exit=";
    while (pre[n]) { msg[n] = pre[n]; n++; }
    if (code < 0) { msg[n++] = '-'; code = -code; }
    char digits[12]; int d = 0;
    do { digits[d++] = '0' + (code % 10); code /= 10; } while (code && d < 11);
    while (d) msg[n++] = digits[--d];
    msg[n++] = '\n';
    _write(1, msg, n);
}
