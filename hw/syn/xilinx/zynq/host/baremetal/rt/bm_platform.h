// Bare-metal host runtime for Vortex on Zynq-7000 — public interface.
//
// The runtime provides: UART stdio, A9 global-timer clocks, newlib syscalls
// (heap, write, blob-backed file IO), an mmap shim over preloaded blobs,
// and flat-mapped MMU with caches ON for host memory and Device (uncached)
// mappings for the Vortex window + peripherals.
//
// An application supplies (typically in a small bm_config.c):
//   const char* bm_cmdline;                 // argv[0] onwards, space-split
//   void bm_app_config(void);               // called before main: register blobs
// and registers each JTAG-preloaded file with bm_blob_add().

#ifndef BM_PLATFORM_H
#define BM_PLATFORM_H
#ifndef __ASSEMBLER__

#include <stddef.h>
#include <stdint.h>
#include <time.h>

// newlib for arm-none-eabi has no POSIX clocks: no clock_gettime and no
// CLOCK_MONOTONIC. The runtime implements clock_gettime over the A9 global
// timer and ignores the clock id.
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC CLOCK_REALTIME
#endif

#ifdef __cplusplus
extern "C" {
#endif

int clock_gettime(clockid_t clk, struct timespec* ts);

// Register a preloaded blob so fopen()/open()/mmap() of `path` resolve to
// the memory at `addr` (host-cacheable region, e.g. 0x20000000+).
void bm_blob_add(const char* path, void* addr, size_t size);

// Application-provided (weak defaults exist):
extern const char* bm_cmdline;   // e.g. "llama2 model.bin -z tok.bin -t 0 -n 40"
void bm_app_config(void);        // register blobs here; runs before main

// Microsecond busy-wait on the A9 global timer.
void bm_udelay(uint32_t us);

// Millisecond monotonic timestamp (global timer).
uint64_t bm_now_ms(void);

#ifdef __cplusplus
}
#endif

#endif // !__ASSEMBLER__
#endif // BM_PLATFORM_H
