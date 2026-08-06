// Minimal sys/mman.h for the bare-metal Vortex host runtime: newlib for
// arm-none-eabi ships no mmap. The runtime implements mmap()/munmap() over
// the preloaded-blob table (bm_platform.c) — mapping a blob-backed fd
// returns the blob's memory directly.
#ifndef BM_SYS_MMAN_H
#define BM_SYS_MMAN_H

#include <stddef.h>
#include <sys/types.h>

#define PROT_READ   0x1
#define PROT_WRITE  0x2
#define MAP_PRIVATE 0x02
#define MAP_SHARED  0x01
#define MAP_FAILED  ((void*)-1)

#ifdef __cplusplus
extern "C" {
#endif

void* mmap(void* addr, size_t length, int prot, int flags, int fd, off_t offset);
int   munmap(void* addr, size_t length);

#ifdef __cplusplus
}
#endif

#endif
