// Copyright © 2019-2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// Zynq-7000 /dev/mem backend for the Vortex runtime (Command Processor
// integration, hw/syn/xilinx/zynq with CP=1).
//
//   * CP register channel: the VX_afu_wrap AXI-Lite window on M_AXI_GP0 —
//     CP regfile at ctrl_base + 0x1000 (host bit-12 split; the CP sees
//     off directly).
//   * CP-visible host memory: a physically reserved, uncached pool mapped
//     through /dev/mem O_SYNC. Boot Linux with mem=240M so the pool
//     [0x0F000000, 0x10000000) stays outside the kernel's allocator; the
//     CP's m_axi_host reaches it through S_AXI_HP0 at the same physical
//     address, and the O_SYNC (Device-type) mapping keeps the CPU view
//     coherent with CP DMA with no explicit sync.
//
// Environment overrides: VX_ZYNQ_CTRL_PHYS, VX_ZYNQ_POOL_PHYS,
// VX_ZYNQ_POOL_SIZE (hex or decimal via strtoull).
// ============================================================================

#include <callbacks.h>
#include <common.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#ifndef DBGPRINT
#define DBGPRINT(...)                                     \
  do {                                                    \
    if (getenv("VXZYNQ_DEBUG"))                           \
      fprintf(stderr, "[vxzynq] " __VA_ARGS__);           \
  } while (0)
#endif

#define CHECK_ERR(_expr, _cleanup)                        \
  do {                                                    \
    auto err = _expr;                                     \
    if (err == 0) break;                                  \
    printf("[VXDRV] Error: '%s' returned %d!\n", #_expr, (int)err); \
    _cleanup                                              \
  } while (false)

static uint64_t env_u64(const char* name, uint64_t dflt) {
  const char* s = getenv(name);
  return s ? strtoull(s, nullptr, 0) : dflt;
}

class vx_device {
public:
  ~vx_device() {
    if (ctrl_)
      munmap((void*)ctrl_, CTRL_SIZE);
    if (pool_)
      munmap(pool_, pool_size_);
    if (fd_ >= 0)
      close(fd_);
  }

  int init() {
    ctrl_phys_ = env_u64("VX_ZYNQ_CTRL_PHYS", 0x43C00000ull);
    pool_phys_ = env_u64("VX_ZYNQ_POOL_PHYS", 0x0F000000ull);
    pool_size_ = env_u64("VX_ZYNQ_POOL_SIZE", 0x01000000ull);

    fd_ = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd_ < 0) {
      perror("[vxzynq] open /dev/mem");
      return -1;
    }
    ctrl_ = (volatile uint32_t*)mmap(nullptr, CTRL_SIZE, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fd_, ctrl_phys_);
    pool_ = (uint8_t*)mmap(nullptr, pool_size_, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd_, pool_phys_);
    if (ctrl_ == MAP_FAILED || pool_ == MAP_FAILED) {
      perror("[vxzynq] mmap");
      return -1;
    }
    pool_next_ = pool_phys_;
    DBGPRINT("init: ctrl@0x%llx pool@0x%llx+0x%llx\n",
             (unsigned long long)ctrl_phys_, (unsigned long long)pool_phys_,
             (unsigned long long)pool_size_);
    return 0;
  }

  // ----- CP register channel (regfile window at +0x1000) -----
  int cp_reg_write(uint32_t off, uint32_t value) {
    // Full barrier on BOTH sides: ring-entry writes (pool_ mapping) and
    // this MMIO doorbell (ctrl_ mapping) are different Device regions —
    // ARM does not order across them without an explicit dmb/dsb.
    __sync_synchronize();
    ctrl_[(CP_WINDOW + off) / 4] = value;
    __sync_synchronize();
    return 0;
  }
  int cp_reg_read(uint32_t off, uint32_t* value) {
    *value = ctrl_[(CP_WINDOW + off) / 4];
    return 0;
  }

  // ----- CP-visible host memory (bump + first-fit free list) -----
  int host_mem_alloc(uint64_t size, void** host_ptr, uint64_t* cp_addr) {
    uint64_t asz = (size + 63) & ~63ull;
    for (auto it = free_.begin(); it != free_.end(); ++it) {
      if (it->second >= asz) {
        uint64_t phys = it->first;
        uint64_t rest = it->second - asz;
        free_.erase(it);
        if (rest)
          free_[phys + asz] = rest;
        used_[phys] = asz;
        *host_ptr = pool_ + (phys - pool_phys_);
        *cp_addr = phys;
        return 0;
      }
    }
    if (pool_next_ + asz > pool_phys_ + pool_size_) {
      fprintf(stderr, "[vxzynq] host pool exhausted (%llu B requested)\n",
              (unsigned long long)size);
      return -1;
    }
    uint64_t phys = pool_next_;
    pool_next_ += asz;
    used_[phys] = asz;
    *host_ptr = pool_ + (phys - pool_phys_);
    *cp_addr = phys;
    DBGPRINT("host_mem_alloc: %llu B @ 0x%llx\n",
             (unsigned long long)size, (unsigned long long)phys);
    return 0;
  }

  int host_mem_free(uint64_t cp_addr) {
    auto it = used_.find(cp_addr);
    if (it == used_.end())
      return -1;
    free_[it->first] = it->second;
    used_.erase(it);
    return 0;
  }

private:
  static constexpr uint64_t CTRL_SIZE = 0x10000;
  static constexpr uint32_t CP_WINDOW = 0x1000;

  int fd_ = -1;
  uint64_t ctrl_phys_ = 0, pool_phys_ = 0, pool_size_ = 0, pool_next_ = 0;
  volatile uint32_t* ctrl_ = nullptr;
  uint8_t* pool_ = nullptr;
  std::map<uint64_t, uint64_t> used_;
  std::map<uint64_t, uint64_t> free_;
};

#include <callbacks.inc>
