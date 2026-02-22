#include <iostream>
#include <vortex.h>
#include <VX_types.h>
#include "common.h"

#define RT_CHECK(_expr) \
   do { \
     int _ret = _expr; \
     if (0 == _ret) break; \
     printf("Error: '%s' returned %d!\n", #_expr, (int)_ret); \
     cleanup(); \
     exit(-1); \
   } while (false)

const char* kernel_file = "kernel.vxbin";
const uint64_t SATP_TEST_VALUE = 0x80001000ULL;

vx_device_h device = nullptr;
vx_buffer_h krnl_buffer = nullptr;
vx_buffer_h args_buffer = nullptr;
vx_buffer_h dst_buffer = nullptr;

void cleanup() {
  if (device) {
    vx_mem_free(krnl_buffer);
    vx_mem_free(args_buffer);
    vx_mem_free(dst_buffer);
    vx_dev_close(device);
  }
}

int main() {
  RT_CHECK(vx_dev_open(&device));

  // Write a known SATP value via DCR so the kernel can read it back via CSR
  RT_CHECK(vx_dcr_write(device, VX_DCR_BASE_SATP0, SATP_TEST_VALUE & 0xffffffff));
  RT_CHECK(vx_dcr_write(device, VX_DCR_BASE_SATP1, SATP_TEST_VALUE >> 32));

  // Allocate output buffer for kernel to write the SATP CSR value into
  RT_CHECK(vx_mem_alloc(device, sizeof(uint64_t), VX_MEM_READ_WRITE, &dst_buffer));

  kernel_arg_t kernel_arg = {};
  RT_CHECK(vx_mem_address(dst_buffer, &kernel_arg.dst_addr));

  // Upload kernel binary and args
  RT_CHECK(vx_upload_kernel_file(device, kernel_file, &krnl_buffer));
  RT_CHECK(vx_upload_bytes(device, &kernel_arg, sizeof(kernel_arg_t), &args_buffer));

  // Run
  RT_CHECK(vx_start(device, krnl_buffer, args_buffer));
  RT_CHECK(vx_ready_wait(device, VX_MAX_TIMEOUT));

  // Read back the SATP value the kernel observed
  uint64_t read_satp = 0;
  RT_CHECK(vx_copy_from_dev(&read_satp, dst_buffer, 0, sizeof(uint64_t)));

  printf("SATP read from kernel: 0x%lx\n", read_satp);
  printf("Expected SATP value:   0x%lx\n", SATP_TEST_VALUE);

  cleanup();

  if (read_satp == SATP_TEST_VALUE) {
    printf("PASSED!\n");
    return 0;
  } else {
    printf("FAILED!\n");
    return 1;
  }
}
