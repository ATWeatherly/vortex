// Minimal legacy Vortex host API for the Arty Z7 libvortex-lite backend.
// Self-contained replacement for the stock <vortex.h> (which drags in the
// Command-Processor runtime); constants only need to agree between this
// header and libvortex_lite.cpp — nothing else interprets them.
#ifndef __VX_VORTEX_LITE_H__
#define __VX_VORTEX_LITE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* vx_device_h;
typedef void* vx_buffer_h;

#define VX_CAPS_VERSION       0x0
#define VX_CAPS_NUM_THREADS   0x1
#define VX_CAPS_NUM_WARPS     0x2
#define VX_CAPS_NUM_CORES     0x3
#define VX_CAPS_CACHE_LINE_SIZE 0x4
#define VX_CAPS_GLOBAL_MEM_SIZE 0x5
#define VX_CAPS_LOCAL_MEM_SIZE  0x6
#define VX_CAPS_ISA_FLAGS     0x7
#define VX_CAPS_NUM_MEM_BANKS 0x8

#define VX_MEM_READ        0x1
#define VX_MEM_WRITE       0x2
#define VX_MEM_READ_WRITE  0x3

#define VX_MAX_TIMEOUT     (24*60*60*1000u)

int vx_dev_open(vx_device_h* hdevice);
int vx_dev_close(vx_device_h hdevice);
int vx_dev_caps(vx_device_h hdevice, uint32_t caps_id, uint64_t* value);
int vx_mem_alloc(vx_device_h hdevice, uint64_t size, int flags, vx_buffer_h* hbuffer);
int vx_mem_reserve(vx_device_h hdevice, uint64_t address, uint64_t size, int flags, vx_buffer_h* hbuffer);
int vx_mem_free(vx_buffer_h hbuffer);
int vx_mem_address(vx_buffer_h hbuffer, uint64_t* address);
int vx_copy_to_dev(vx_buffer_h hbuffer, const void* host_ptr, uint64_t dst_offset, uint64_t size);
int vx_copy_from_dev(void* host_ptr, vx_buffer_h hbuffer, uint64_t src_offset, uint64_t size);
int vx_upload_kernel_file(vx_device_h hdevice, const char* filename, vx_buffer_h* hbuffer);
int vx_start(vx_device_h hdevice, vx_buffer_h hkernel, vx_buffer_h harguments);
int vx_ready_wait(vx_device_h hdevice, uint64_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
