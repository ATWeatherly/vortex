// Copyright © 2019-2023
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

#include <common.h>

#ifdef SCOPE
#include "scope.h"
#endif

// XRT includes
#ifdef XRTSIM
#include <xrt_c.h>
#else
#include "experimental/xrt_bo.h"
#include "experimental/xrt_device.h"
#include "experimental/xrt_error.h"
#include "experimental/xrt_ip.h"
#include "experimental/xrt_kernel.h"
#include "experimental/xrt_xclbin.h"
#endif

#include <limits>
#include <stdarg.h>
#include <string>
#include <unordered_map>
#include <util.h>
#include <vector>

using namespace vortex;

#ifndef XRTSIM
#define CPP_API
#endif

// Enabled to match the hardware line-interleave (PLATFORM_MEMORY_INTERLEAVE=1)
// when the U50 uses the un-merged per-bank AXI config (8 masters) in platforms.mk.
// Buffers are striped across all banks so each matmul spans all HBM channels.
// Revert (re-comment) if rebuilding the merged single-master bitstream.
#define BANK_INTERLEAVE

#define MMIO_CTL_ADDR 0x00
#define MMIO_DEV_ADDR 0x10
#define MMIO_ISA_ADDR 0x18
#define MMIO_DCR_ADDR 0x20
#define MMIO_SCP_ADDR 0x28
#define MMIO_MEM_ADDR 0x30

#define CTL_AP_START (1 << 0)
#define CTL_AP_DONE (1 << 1)
#define CTL_AP_IDLE (1 << 2)
#define CTL_AP_READY (1 << 3)
#define CTL_AP_RESET (1 << 4)
#define CTL_AP_RESTART (1 << 7)

#ifdef CPP_API

typedef xrt::device xrt_device_t;
typedef xrt::ip xrt_kernel_t;
typedef xrt::bo xrt_buffer_t;

#else

typedef xrtDeviceHandle xrt_device_t;
typedef xrtKernelHandle xrt_kernel_t;
typedef xrtBufferHandle xrt_buffer_t;

#endif

#define DEFAULT_DEVICE_INDEX 0

#define DEFAULT_XCLBIN_PATH "vortex_afu.xclbin"

#define KERNEL_NAME "vortex_afu"

#define CHECK_HANDLE(handle, _expr, _cleanup)                                  \
  auto handle = _expr;                                                         \
  if (handle == nullptr) {                                                     \
    printf("[VXDRV] Error: '%s' returned NULL!\n", #_expr);                    \
    _cleanup                                                                   \
  }

#ifndef CPP_API
static void dump_xrt_error(xrtDeviceHandle xrtDevice, xrtErrorCode err) {
  size_t len = 0;
  xrtErrorGetString(xrtDevice, err, nullptr, 0, &len);
  std::vector<char> buf(len);
  xrtErrorGetString(xrtDevice, err, buf.data(), buf.size(), nullptr);
  printf("[VXDRV] detail: %s!\n", buf.data());
}
#endif

///////////////////////////////////////////////////////////////////////////////

// ---- launch-path timing instrumentation ----
// Enable by setting VORTEX_RT_TIMING in the environment:
//   VORTEX_RT_TIMING=1  -> print a cumulative breakdown at device teardown
//   VORTEX_RT_TIMING=2  -> also print every upload/download/ready_wait call
static inline uint64_t rt_now_us() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

// RAII helper: adds elapsed wall time (us) to *accum and bumps *count on scope exit.
// Robust to the multiple early-return paths in upload()/download().
struct rt_scoped_timer {
  uint64_t start_us;
  uint64_t *accum_us;
  uint64_t *count;
  rt_scoped_timer(uint64_t *accum, uint64_t *cnt)
    : start_us(rt_now_us()), accum_us(accum), count(cnt) {}
  ~rt_scoped_timer() {
    *accum_us += rt_now_us() - start_us;
    if (count) ++(*count);
  }
};

// Per-bank BO size cap. Some shells/drivers refuse a single device BO equal to a
// full HBM bank (observed on the 8-channel U50 build: 1 GB/bank). XRT then falls
// back to a host userptr BO which the driver denies with EPERM ("failed to
// allocate userptr bo: Operation not permitted"). Capping the per-bank BO to a
// smaller size avoids the oversized device allocation. Safe because allocations
// are striped across banks (get_bank_info), so each bank only holds
// total_used/num_banks bytes — a few MB for small models; xrt-smi validate shows
// 16 MB BOs work and 256 MB == one HBM pseudo-channel. Set VORTEX_BANK_BO_SIZE to
// the cap in bytes (e.g. 268435456 for 256 MB); default 0 = no cap (full bank).
static uint64_t bank_bo_size(uint64_t bank_size) {
  static const uint64_t cap = []{
    const char* e = getenv("VORTEX_BANK_BO_SIZE");
    return (e && e[0]) ? strtoull(e, nullptr, 0) : 0ull;
  }();
  return (cap && cap < bank_size) ? cap : bank_size;
}

///////////////////////////////////////////////////////////////////////////////

class vx_device {
public:
  vx_device()
    : global_mem_(ALLOC_BASE_ADDR,
                  GLOBAL_MEM_SIZE - ALLOC_BASE_ADDR,
                  RAM_PAGE_SIZE,
                  CACHE_BLOCK_SIZE)
  #ifndef CPP_API
    , xrtDevice_(nullptr)
    , xrtKernel_(nullptr)
  #endif
  {}

  ~vx_device() {
    if (rt_timing_ >= 1) {
      uint64_t total = t_upload_us_ + t_download_us_ + t_readywait_us_;
      fprintf(stderr, "[RT-TIME] ===== launch-path breakdown =====\n");
      fprintf(stderr, "[RT-TIME] upload   : %10lu us  calls=%-6lu bytes=%-12lu\n",
              t_upload_us_, n_upload_, up_bytes_);
      fprintf(stderr, "[RT-TIME] download : %10lu us  calls=%-6lu bytes=%-12lu\n",
              t_download_us_, n_download_, dn_bytes_);
      fprintf(stderr, "[RT-TIME] ready_wait: %9lu us  calls=%-6lu polls=%-12lu\n",
              t_readywait_us_, n_readywait_, n_polls_);
      fprintf(stderr, "[RT-TIME] ready_wait avg poll/call=%.1f  (sleep slack lives here)\n",
              n_readywait_ ? (double)n_polls_ / (double)n_readywait_ : 0.0);
      fprintf(stderr, "[RT-TIME] sum(up+dn+wait)=%lu us\n", total);
      fprintf(stderr, "[RT-TIME] ==================================\n");
    }
  #ifdef SCOPE
    vx_scope_stop(this);
  #endif
  #ifndef CPP_API
    for (auto &entry : xrtBuffers_) {
    #ifdef BANK_INTERLEAVE
      xrtBOFree(entry);
    #else
      xrtBOFree(entry.second.xrtBuffer);
    #endif
    }
    if (xrtKernel_) {
      xrtKernelClose(xrtKernel_);
    }
    if (xrtDevice_) {
      xrtDeviceClose(xrtDevice_);
    }
  #endif
  }

  int init() {
    const char *rt_timing_s = getenv("VORTEX_RT_TIMING");
    rt_timing_ = (rt_timing_s != nullptr) ? atoi(rt_timing_s) : 0;

    int device_index = DEFAULT_DEVICE_INDEX;
    const char *device_index_s = getenv("XRT_DEVICE_INDEX");
    if (device_index_s == nullptr || device_index_s[0] == '\0')
      device_index_s = getenv("OMPI_COMM_WORLD_RANK");
    if (device_index_s != nullptr) {
      device_index = atoi(device_index_s);
    }

    const char *xlbin_path_s = getenv("XRT_XCLBIN_PATH");
    if (xlbin_path_s == nullptr) {
      xlbin_path_s = DEFAULT_XCLBIN_PATH;
    }

  #ifdef CPP_API

    auto xrtDevice = xrt::device(device_index);
    auto uuid = xrtDevice.load_xclbin(xlbin_path_s);
    auto xrtKernel = xrt::ip(xrtDevice, uuid, KERNEL_NAME);
    auto xclbin = xrt::xclbin(xlbin_path_s);
    auto device_name = xrtDevice.get_info<xrt::info::device::name>();

  #else

    CHECK_HANDLE(xrtDevice, xrtDeviceOpen(device_index), {
      return -1;
    });

  #ifndef XRTSIM
    CHECK_ERR(xrtDeviceLoadXclbinFile(xrtDevice, xlbin_path_s), {
      dump_xrt_error(xrtDevice, err);
      xrtDeviceClose(xrtDevice);
      return err;
    });

    xuid_t uuid;
    CHECK_ERR(xrtDeviceGetXclbinUUID(xrtDevice, uuid), {
      dump_xrt_error(xrtDevice, err);
      xrtDeviceClose(xrtDevice);
      return err;
    });

    CHECK_HANDLE(xrtKernel, xrtPLKernelOpenExclusive(xrtDevice, uuid, KERNEL_NAME), {
      xrtDeviceClose(xrtDevice);
      return -1;
    });
  #else
    xrtKernelHandle xrtKernel = xrtDevice;
  #endif

    // get device name
    int device_name_size;
    xrtXclbinGetXSAName(xrtDevice, nullptr, 0, &device_name_size);
    std::vector<char> sz_device_name(device_name_size);
    xrtXclbinGetXSAName(xrtDevice, sz_device_name.data(), device_name_size, nullptr);
    std::string device_name(sz_device_name.data(), device_name_size);

  #endif

    xrtDevice_ = xrtDevice;
    xrtKernel_ = xrtKernel;

    CHECK_ERR(this->write_register(MMIO_CTL_ADDR, CTL_AP_RESET), {
      return err;
    });

    CHECK_ERR(this->read_register(MMIO_DEV_ADDR, (uint32_t *)&dev_caps_), {
      return err;
    });

    CHECK_ERR(this->read_register(MMIO_DEV_ADDR + 4, (uint32_t *)&dev_caps_ + 1), {
      return err;
    });

    CHECK_ERR(this->read_register(MMIO_ISA_ADDR, (uint32_t *)&isa_caps_), {
      return err;
    });

    CHECK_ERR(this->read_register(MMIO_ISA_ADDR + 4, (uint32_t *)&isa_caps_ + 1), {
      return err;
    });

    uint64_t num_banks;
    this->get_caps(VX_CAPS_NUM_MEM_BANKS, &num_banks);
    lg2_num_banks_ = log2ceil(num_banks);

    uint64_t bank_size;
    this->get_caps(VX_CAPS_MEM_BANK_SIZE, &bank_size);
    lg2_bank_size_ = log2ceil(bank_size);

    global_mem_size_ = num_banks * bank_size;

    printf("info: device name=%s, memory_capacity=0x%lx bytes, memory_banks=%ld.\n", device_name.c_str(), global_mem_size_, num_banks);

    // ---- BO memory-group probe (VORTEX_BO_PROBE=1) ----
    // Diagnostic for the 8-channel U50 "userptr bo: Operation not permitted" bug:
    // per-bank BOs are allocated with a raw bank index as the XRT memory group,
    // which on this shell resolves to a single 256 MB HBM channel rather than the
    // 1 GB interleaved MBG group, so any BO > 256 MB fails. This probe allocates a
    // throwaway device BO at each candidate memory-group index (largest size first)
    // and reports the largest size each index accepts, so we can find which index
    // (if any) maps to a full 1 GB group. It then exits without touching the device
    // further. Range overridable with VORTEX_BO_PROBE_MAX (default 40 covers
    // 32 HBM channels + PLRAM + HOST + any grouped indices).
    if (const char *probe = getenv("VORTEX_BO_PROBE"); probe && probe[0] == '1') {
      const uint64_t MB = 1ull << 20;
      const uint64_t sizes[] = { 1024 * MB, 512 * MB, 256 * MB, 64 * MB, 16 * MB };
      int max_idx = 40;
      if (const char *mi = getenv("VORTEX_BO_PROBE_MAX")) {
        int v = atoi(mi);
        if (v > 0) max_idx = v;
      }
      fprintf(stderr, "[BO-PROBE] num_banks=%lu bank_size=%lu MB; probing group indices 0..%d\n",
              num_banks, bank_size / MB, max_idx);
      for (int idx = 0; idx <= max_idx; ++idx) {
        uint64_t best = 0;
        for (uint64_t sz : sizes) {
          try {
            xrt::bo b(xrtDevice_, sz, xrt::bo::flags::normal, idx); // freed at scope end
            best = sz;       // sizes descending: first success is the max this index accepts
            break;
          } catch (...) {
            // try the next-smaller size
          }
        }
        if (best)
          fprintf(stderr, "[BO-PROBE] group idx %2d : max BO = %4lu MB%s\n",
                  idx, best / MB, (best >= bank_size) ? "  <-- accepts full bank" : "");
        else
          fprintf(stderr, "[BO-PROBE] group idx %2d : no allocation up to %lu MB\n",
                  idx, sizes[0] / MB);
      }
      fprintf(stderr, "[BO-PROBE] done; exiting. Unset VORTEX_BO_PROBE to use the device normally.\n");
      exit(0);
    }

    // Resolve the per-bank BO memory-group base (see bo_group_base_ note). Prefer an
    // explicit override; otherwise self-calibrate by finding the first memory index
    // that accepts a full bank_size BO. On a multi-channel xclbin the raw indices are
    // individual HBM channels that cap at one channel's size and throw for bank_size,
    // while the interleaved per-port groups (indexed above the channel/PLRAM/HOST
    // entries) accept it. When bank_size already fits one channel this returns 0
    // (legacy raw-index behavior).
    bo_group_base_ = 0;
    if (const char *gb = getenv("VORTEX_BO_GROUP_BASE"); gb && gb[0]) {
      bo_group_base_ = (uint32_t)strtoul(gb, nullptr, 0);
    } else {
    #ifdef CPP_API
      for (uint32_t idx = 0; idx < 256; ++idx) {
        try {
          xrt::bo probe(xrtDevice_, bank_size, xrt::bo::flags::normal, idx); // freed at scope end
          bo_group_base_ = idx;
          break;
        } catch (...) { /* index too small or invalid; keep scanning */ }
      }
    #endif
    }
    fprintf(stderr, "[VXDRV] per-bank BO memory-group base = %u (bank i -> group idx %u+i)\n",
            bo_group_base_, bo_group_base_);

  #ifdef BANK_INTERLEAVE
    xrtBuffers_.reserve(num_banks);
    uint64_t bo_size = bank_bo_size(bank_size);
    for (uint32_t i = 0; i < num_banks; ++i) {
      uint32_t grp = bo_group_base_ + i;
    #ifdef CPP_API
      xrtBuffers_.emplace_back(xrtDevice_, bo_size, xrt::bo::flags::normal, grp);
    #else
      CHECK_HANDLE(xrtBuffer, xrtBOAlloc(xrtDevice_, bo_size, XRT_BO_FLAGS_NONE, grp), {
         return -1;
      });
      xrtBuffers_.push_back(xrtBuffer);
    #endif
      printf("*** allocated bank%u/%lu, size=%lu, group=%u\n", i, num_banks, bo_size, grp);
    }
  #endif

  #ifdef SCOPE
    {
      scope_callback_t callback;
      callback.registerWrite = [](vx_device_h hdevice, uint64_t value) -> int {
        auto device = (vx_device *)hdevice;
        uint32_t value_lo = (uint32_t)(value);
        uint32_t value_hi = (uint32_t)(value >> 32);
        CHECK_ERR(device->write_register(MMIO_SCP_ADDR, value_lo), {
          return err;
        });
        CHECK_ERR(device->write_register(MMIO_SCP_ADDR + 4, value_hi), {
          return err;
        });
        return 0;
      };
      callback.registerRead = [](vx_device_h hdevice, uint64_t *value) -> int {
        auto device = (vx_device *)hdevice;
        uint32_t value_lo, value_hi;
        CHECK_ERR(device->read_register(MMIO_SCP_ADDR, &value_lo), {
          return err;
        });
        CHECK_ERR(device->read_register(MMIO_SCP_ADDR + 4, &value_hi), {
          return err;
        });
        *value = (((uint64_t)value_hi) << 32) | value_lo;
        return 0;
      };
      CHECK_ERR(vx_scope_start(&callback, this, -1, -1), {
        return err;
      });
    }
  #endif

  #ifdef CHIPSCOPE
    std::cout << "\nPress ENTER to continue after setting up ILA trigger..." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  #endif

    return 0;
  }

  int get_caps(uint32_t caps_id, uint64_t *value) {
    uint64_t _value;

    switch (caps_id) {
    case VX_CAPS_VERSION:
      _value = (dev_caps_ >> 0) & 0xff;
      break;
    case VX_CAPS_NUM_THREADS:
      _value = (dev_caps_ >> 8) & 0xff;
      break;
    case VX_CAPS_NUM_WARPS:
      _value = (dev_caps_ >> 16) & 0xff;
      break;
    case VX_CAPS_NUM_CORES:
      _value = (dev_caps_ >> 24) & 0xffff;
      break;
    case VX_CAPS_CACHE_LINE_SIZE:
      _value = CACHE_BLOCK_SIZE;
      break;
    case VX_CAPS_GLOBAL_MEM_SIZE:
      _value = global_mem_size_;
      break;
    case VX_CAPS_LOCAL_MEM_SIZE:
      _value = 1ull << ((dev_caps_ >> 40) & 0xff);
      break;
    case VX_CAPS_ISA_FLAGS:
      _value = isa_caps_;
      break;
    case VX_CAPS_NUM_MEM_BANKS:
      _value = 1 << ((dev_caps_ >> 48) & 0x7);
      break;
    case VX_CAPS_MEM_BANK_SIZE:
      _value = 1ull << (20 + ((dev_caps_ >> 51) & 0x1f));
      break;
    default:
      fprintf(stderr, "[VXDRV] Error: invalid caps id: %d\n", caps_id);
      std::abort();
      return -1;
    }

    *value = _value;

    return 0;
  }

  int mem_alloc(uint64_t size, int flags, uint64_t *dev_addr) {
    uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
    uint64_t addr;
    CHECK_ERR(global_mem_.allocate(asize, &addr), {
      return err;
    });
  #ifndef BANK_INTERLEAVE
    // Register BOs for every bank spanned by this allocation so reference
    // counts stay correct even when an allocation crosses a bank boundary.
    uint32_t first_bank = (uint32_t)(addr >> lg2_bank_size_);
    uint32_t last_bank  = (uint32_t)((addr + asize - 1) >> lg2_bank_size_);
    for (uint32_t b = first_bank; b <= last_bank; ++b) {
      CHECK_ERR(get_buffer(b, nullptr), {
        global_mem_.release(addr);
        return err;
      });
    }
    alloc_size_map_[addr] = asize;
  #endif
    CHECK_ERR(this->mem_access(addr, size, flags), {
      global_mem_.release(addr);
      return err;
    });
    *dev_addr = addr;
    return 0;
  }

  int mem_reserve(uint64_t dev_addr, uint64_t size, int flags) {
    uint64_t asize = aligned_size(size, RAM_PAGE_SIZE);
    CHECK_ERR(global_mem_.reserve(dev_addr, size), {
      return err;
    });
  #ifndef BANK_INTERLEAVE
    uint32_t first_bank = (uint32_t)(dev_addr >> lg2_bank_size_);
    uint32_t last_bank  = (uint32_t)((dev_addr + asize - 1) >> lg2_bank_size_);
    for (uint32_t b = first_bank; b <= last_bank; ++b) {
      CHECK_ERR(get_buffer(b, nullptr), {
        global_mem_.release(dev_addr);
        return err;
      });
    }
    alloc_size_map_[dev_addr] = asize;
  #endif
    CHECK_ERR(this->mem_access(dev_addr, size, flags), {
      global_mem_.release(dev_addr);
      return err;
    });
    return 0;
  }

  int mem_free(uint64_t dev_addr) {
    CHECK_ERR(global_mem_.release(dev_addr), {
      return err;
    });
  #ifdef BANK_INTERLEAVE
    // NOTE: per-bank BOs are allocated once in init() and indexed directly by
    // get_buffer(); they are NOT recreated on mem_alloc(). Freeing/clearing them
    // here when allocation hits zero would leave get_buffer() reading an empty
    // vector if the program allocates again after a full free (e.g. llama2's
    // teardown allocs after freeing) -> std::out_of_range crash. Keep the BOs for
    // the device lifetime; the destructor releases them. (allocated()==0 path
    // intentionally left as a no-op.)
  #else
    auto sz_it = alloc_size_map_.find(dev_addr);
    if (sz_it == alloc_size_map_.end()) {
      fprintf(stderr, "[VXDRV] Error: invalid device memory address: 0x%lx\n",
              dev_addr);
      return -1;
    }
    uint64_t asize = sz_it->second;
    alloc_size_map_.erase(sz_it);

    uint32_t first_bank = (uint32_t)(dev_addr >> lg2_bank_size_);
    uint32_t last_bank  = (uint32_t)((dev_addr + asize - 1) >> lg2_bank_size_);
    for (uint32_t b = first_bank; b <= last_bank; ++b) {
      auto it = xrtBuffers_.find(b);
      if (it == xrtBuffers_.end()) {
        fprintf(stderr, "[VXDRV] Error: missing BO for bank %u (addr=0x%lx)\n",
                b, dev_addr);
        return -1;
      }
      auto count = --it->second.count;
      if (0 == count) {
        printf("freeing bank%u...\n", b);
      #ifndef CPP_API
        xrtBOFree(it->second.xrtBuffer);
      #endif
        xrtBuffers_.erase(it);
      }
    }
  #endif
    return 0;
  }

  int mem_access(uint64_t /*dev_addr*/, uint64_t /*size*/, int /*flags*/) {
    return 0;
  }

  int mem_info(uint64_t *mem_free, uint64_t *mem_used) const {
    if (mem_free)
      *mem_free = global_mem_.free();
    if (mem_used)
      *mem_used = global_mem_.allocated();
    return 0;
  }

  int write_register(uint32_t addr, uint32_t value) {
  #ifdef CPP_API
    xrtKernel_.write_register(addr, value);
  #else
    CHECK_ERR(xrtKernelWriteRegister(xrtKernel_, addr, value), {
      dump_xrt_error(xrtDevice_, err);
      return err;
    });
  #endif
    return 0;
  }

  int read_register(uint32_t addr, uint32_t *value) {
  #ifdef CPP_API
    *value = xrtKernel_.read_register(addr);
  #else
    CHECK_ERR(xrtKernelReadRegister(xrtKernel_, addr, value), {
      dump_xrt_error(xrtDevice_, err);
      return err;
    });
  #endif
    return 0;
  }

  int copy(uint64_t dest_addr, uint64_t src_addr, uint64_t size) {
    if (dest_addr == src_addr) {
      return 0;
    }

    // bound checking
    if (dest_addr + size > global_mem_size_ ||
        src_addr + size > global_mem_size_)
      return -1;

    uint64_t offset = 0;
    while (offset < size) {
      uint64_t curr_src = src_addr + offset;
      uint64_t curr_dest = dest_addr + offset;

      uint64_t src_rem = CACHE_BLOCK_SIZE - (curr_src % CACHE_BLOCK_SIZE);
      uint64_t dest_rem = CACHE_BLOCK_SIZE - (curr_dest % CACHE_BLOCK_SIZE);

      uint64_t chunk_size = (src_rem < dest_rem) ? src_rem : dest_rem;
      if (chunk_size > size - offset) {
        chunk_size = size - offset;
      }

      uint32_t src_bo_idx, dst_bo_idx;
      uint64_t src_bo_off, dst_bo_off;
      xrt_buffer_t src_buf, dst_buf;

      CHECK_ERR(this->get_bank_info(curr_src, &src_bo_idx, &src_bo_off), {
        return err;
      });
#ifdef BANK_INTERLEAVE
      src_bo_off += (curr_src % CACHE_BLOCK_SIZE);
#endif

      CHECK_ERR(this->get_buffer(src_bo_idx, &src_buf), {
        return err;
      });

      CHECK_ERR(this->get_bank_info(curr_dest, &dst_bo_idx, &dst_bo_off), {
        return err;
      });
#ifdef BANK_INTERLEAVE
      dst_bo_off += (curr_dest % CACHE_BLOCK_SIZE);
#endif

      CHECK_ERR(this->get_buffer(dst_bo_idx, &dst_buf), {
        return err;
      });

#ifdef CPP_API
      dst_buf.copy(src_buf, chunk_size, src_bo_off, dst_bo_off);
#else
      CHECK_ERR(xrtBOCopy(dst_buf, src_buf, chunk_size, src_bo_off, dst_bo_off), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
#endif

      offset += chunk_size;
    }

    return 0;
  }

  int upload(uint64_t dev_addr, const void *src, uint64_t size) {
    rt_scoped_timer _rt(&t_upload_us_, &n_upload_);
    up_bytes_ += size;
    if (rt_timing_ >= 2)
      fprintf(stderr, "[RT-TIME] upload  dev=0x%lx size=%lu\n", dev_addr, size);
    auto host_ptr = (const uint8_t *)src;

    // check alignment
    if (!is_aligned(dev_addr, CACHE_BLOCK_SIZE))
      return -1;

    auto asize = aligned_size(size, CACHE_BLOCK_SIZE);

    // bound checking
    if (dev_addr + asize > global_mem_size_)
      return -1;

#ifdef BANK_INTERLEAVE
    for (uint64_t end = dev_addr + asize; dev_addr < end;
         dev_addr += CACHE_BLOCK_SIZE, host_ptr += CACHE_BLOCK_SIZE) {
      asize = CACHE_BLOCK_SIZE;
      uint32_t bo_index;
      uint64_t bo_offset;
      xrt_buffer_t xrtBuffer;
      CHECK_ERR(this->get_bank_info(dev_addr, &bo_index, &bo_offset), {
        return err;
      });
      CHECK_ERR(this->get_buffer(bo_index, &xrtBuffer), {
        return err;
      });
    #ifdef CPP_API
      xrtBuffer.write(host_ptr, size, bo_offset);
      xrtBuffer.sync(XCL_BO_SYNC_BO_TO_DEVICE, size, bo_offset);
    #else
      CHECK_ERR(xrtBOWrite(xrtBuffer, host_ptr, size, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
      CHECK_ERR(xrtBOSync(xrtBuffer, XCL_BO_SYNC_BO_TO_DEVICE, size, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
    #endif
    }
#else
    for (uint64_t done = 0; done < size; ) {
      uint32_t bo_index;
      uint64_t bo_offset;
      xrt_buffer_t xrtBuffer;
      CHECK_ERR(this->get_bank_info(dev_addr + done, &bo_index, &bo_offset), {
        return err;
      });
      CHECK_ERR(this->get_buffer(bo_index, &xrtBuffer), {
        return err;
      });
      uint64_t bank_remain = (1ull << lg2_bank_size_) - bo_offset;
      uint64_t chunk = (size - done < bank_remain) ? (size - done) : bank_remain;
    #ifdef CPP_API
      xrtBuffer.write(host_ptr + done, chunk, bo_offset);
      xrtBuffer.sync(XCL_BO_SYNC_BO_TO_DEVICE, chunk, bo_offset);
    #else
      CHECK_ERR(xrtBOWrite(xrtBuffer, host_ptr + done, chunk, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
      CHECK_ERR(xrtBOSync(xrtBuffer, XCL_BO_SYNC_BO_TO_DEVICE, chunk, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
    #endif
      done += chunk;
    }
#endif
    return 0;
  }

  int download(void *dest, uint64_t dev_addr, uint64_t size) {
    rt_scoped_timer _rt(&t_download_us_, &n_download_);
    dn_bytes_ += size;
    if (rt_timing_ >= 2)
      fprintf(stderr, "[RT-TIME] download dev=0x%lx size=%lu\n", dev_addr, size);
    auto host_ptr = (uint8_t *)dest;

    // check alignment
    if (!is_aligned(dev_addr, CACHE_BLOCK_SIZE))
      return -1;

    auto asize = aligned_size(size, CACHE_BLOCK_SIZE);

    // bound checking
    if (dev_addr + asize > global_mem_size_)
      return -1;

#ifdef BANK_INTERLEAVE
    for (uint64_t end = dev_addr + asize; dev_addr < end;
         dev_addr += CACHE_BLOCK_SIZE, host_ptr += CACHE_BLOCK_SIZE) {
      asize = CACHE_BLOCK_SIZE;
      uint32_t bo_index;
      uint64_t bo_offset;
      xrt_buffer_t xrtBuffer;
      CHECK_ERR(this->get_bank_info(dev_addr, &bo_index, &bo_offset), {
        return err;
      });
      CHECK_ERR(this->get_buffer(bo_index, &xrtBuffer), {
        return err;
      });
    #ifdef CPP_API
      xrtBuffer.sync(XCL_BO_SYNC_BO_FROM_DEVICE, size, bo_offset);
      xrtBuffer.read(host_ptr, size, bo_offset);
    #else
      CHECK_ERR(xrtBOSync(xrtBuffer, XCL_BO_SYNC_BO_FROM_DEVICE, size, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
      CHECK_ERR(xrtBORead(xrtBuffer, host_ptr, size, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
    #endif
    }
#else
    for (uint64_t done = 0; done < size; ) {
      uint32_t bo_index;
      uint64_t bo_offset;
      xrt_buffer_t xrtBuffer;
      CHECK_ERR(this->get_bank_info(dev_addr + done, &bo_index, &bo_offset), {
        return err;
      });
      CHECK_ERR(this->get_buffer(bo_index, &xrtBuffer), {
        return err;
      });
      uint64_t bank_remain = (1ull << lg2_bank_size_) - bo_offset;
      uint64_t chunk = (size - done < bank_remain) ? (size - done) : bank_remain;
    #ifdef CPP_API
      xrtBuffer.sync(XCL_BO_SYNC_BO_FROM_DEVICE, chunk, bo_offset);
      xrtBuffer.read(host_ptr + done, chunk, bo_offset);
    #else
      CHECK_ERR(xrtBOSync(xrtBuffer, XCL_BO_SYNC_BO_FROM_DEVICE, chunk, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
      CHECK_ERR(xrtBORead(xrtBuffer, host_ptr + done, chunk, bo_offset), {
        dump_xrt_error(xrtDevice_, err);
        return err;
      });
    #endif
      done += chunk;
    }
#endif
    return 0;
  }

  int start(uint64_t krnl_addr, uint64_t args_addr) {
    // set kernel info
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ADDR0, krnl_addr & 0xffffffff), {
      return err;
    });
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ADDR1, krnl_addr >> 32), {
      return err;
    });
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ARG0, args_addr & 0xffffffff), {
      return err;
    });
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ARG1, args_addr >> 32), {
      return err;
    });

    // start execution
    CHECK_ERR(this->write_register(MMIO_CTL_ADDR, CTL_AP_START), {
      return err;
    });

    // clear mpm cache
    mpm_cache_.clear();

    return 0;
  }

  int ready_wait(uint64_t timeout) {
    rt_scoped_timer _rt(&t_readywait_us_, &n_readywait_);
    uint64_t polls = 0;
    struct timespec sleep_time;
  #ifndef NDEBUG
    sleep_time.tv_sec = 1;
    sleep_time.tv_nsec = 0;
  #else
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 100000;   // 0.1 ms (diagnostic: lowered completion-poll interval; still > light-kernel runtime)
  #endif

    // to milliseconds (ceil so a sub-1ms interval still decrements the timeout)
    uint64_t sleep_time_ms = (sleep_time.tv_sec * 1000)
                           + ((sleep_time.tv_nsec + 999999) / 1000000);

    for (;;) {
      ++polls;
      uint32_t status = 0;
      CHECK_ERR(this->read_register(MMIO_CTL_ADDR, &status), {
        n_polls_ += polls;
        return err;
      });
      bool is_done = (status & CTL_AP_DONE) == CTL_AP_DONE;
      if (is_done)
        break;
      if (0 == timeout) {
        n_polls_ += polls;
        return -1;
      }
      nanosleep(&sleep_time, nullptr);
      timeout -= sleep_time_ms;
    };

    n_polls_ += polls;
    if (rt_timing_ >= 2)
      fprintf(stderr, "[RT-TIME] ready_wait polls=%lu\n", polls);

    return 0;
  }

  int dcr_write(uint32_t addr, uint32_t value) {
    CHECK_ERR(this->write_register(MMIO_DCR_ADDR, addr), {
      return err;
    });
    CHECK_ERR(this->write_register(MMIO_DCR_ADDR + 4, value), {
      return err;
    });
    dcrs_.write(addr, value);
    return 0;
  }

  int dcr_read(uint32_t addr, uint32_t *value) const {
    return dcrs_.read(addr, value);
  }

  int mpm_query(uint32_t addr, uint32_t core_id, uint64_t *value) {
    uint32_t offset = addr - VX_CSR_MPM_BASE;
    if (offset > 31)
      return -1;
    if (mpm_cache_.count(core_id) == 0) {
      uint64_t mpm_mem_addr = IO_MPM_ADDR + core_id * 32 * sizeof(uint64_t);
      CHECK_ERR(this->download(mpm_cache_[core_id].data(), mpm_mem_addr, 32 * sizeof(uint64_t)), {
        return err;
      });
    }
    *value = mpm_cache_.at(core_id).at(offset);
    return 0;
  }

private:

  MemoryAllocator global_mem_;
  xrt_device_t xrtDevice_;
  xrt_kernel_t xrtKernel_;
  uint64_t dev_caps_;
  uint64_t isa_caps_;
  uint64_t global_mem_size_;
  DeviceConfig dcrs_;
  std::unordered_map<uint32_t, std::array<uint64_t, 32>> mpm_cache_;
  uint32_t lg2_num_banks_;
  uint32_t lg2_bank_size_;
  // Memory-group base index for per-bank BOs. On a multi-channel xclbin the raw
  // bank indices 0..N-1 hit individual HBM channels (capped at one channel's size),
  // while the interleaved per-port groups the hardware actually uses are indexed
  // ABOVE the MEM_TOPOLOGY entries (e.g. 32 HBM + 4 PLRAM + 1 HOST = 37 -> groups
  // at 37..44 on the 8-channel U50). bank i -> group bo_group_base_ + i.
  uint32_t bo_group_base_ = 0;

  // launch-path timing accumulators (us)
  int      rt_timing_ = 0;     // 0=off, 1=summary, >=2=per-call
  uint64_t t_upload_us_ = 0,    n_upload_ = 0,   up_bytes_ = 0;
  uint64_t t_download_us_ = 0,  n_download_ = 0, dn_bytes_ = 0;
  uint64_t t_readywait_us_ = 0, n_readywait_ = 0, n_polls_ = 0;

#ifndef BANK_INTERLEAVE
  std::unordered_map<uint64_t, uint64_t> alloc_size_map_;
#endif

#ifdef BANK_INTERLEAVE

  std::vector<xrt_buffer_t> xrtBuffers_;

  int get_bank_info(uint64_t addr, uint32_t *pIdx, uint64_t *pOff) {
    uint32_t num_banks = 1 << lg2_num_banks_;
    uint64_t block_addr = addr / CACHE_BLOCK_SIZE;
    uint32_t index = block_addr & (num_banks - 1);
    uint64_t offset = (block_addr >> lg2_num_banks_) * CACHE_BLOCK_SIZE;
    if (pIdx) {
      *pIdx = index;
    }
    if (pOff) {
      *pOff = offset;
    }
    //printf("get_bank_info(addr=0x%lx, bank=%d, offset=0x%lx\n", addr, index, offset);
    return 0;
  }

  int get_buffer(uint32_t bank_id, xrt_buffer_t *pBuf) {
    if (pBuf) {
      *pBuf = xrtBuffers_.at(bank_id);
    }
    return 0;
  }

#else

  struct buf_cnt_t {
    xrt_buffer_t xrtBuffer;
    uint32_t count;
  };

  std::unordered_map<uint32_t, buf_cnt_t> xrtBuffers_;

  int get_bank_info(uint64_t addr, uint32_t *pIdx, uint64_t *pOff) {
    uint32_t num_banks = 1 << lg2_num_banks_;
    uint64_t bank_size = 1ull << lg2_bank_size_;
    uint32_t index = addr >> lg2_bank_size_;
    uint64_t offset = addr & (bank_size - 1);
    if (index > num_banks) {
      fprintf(stderr, "[VXDRV] Error: address out of range: 0x%lx\n", addr);
      return -1;
    }
    if (pIdx) {
      *pIdx = index;
    }
    if (pOff) {
      *pOff = offset;
    }
    //printf("get_bank_info(addr=0x%lx, bank=%d, offset=0x%lx\n", addr, index, offset);
    return 0;
  }

  int get_buffer(uint32_t bank_id, xrt_buffer_t *pBuf) {
    auto it = xrtBuffers_.find(bank_id);
    if (it != xrtBuffers_.end()) {
      if (pBuf) {
        *pBuf = it->second.xrtBuffer;
      } else {
        printf("reusing bank%d...\n", bank_id);
        ++it->second.count;
      }
    } else {
      printf("allocating bank%d...\n", bank_id);
      uint64_t bank_size = bank_bo_size(1ull << lg2_bank_size_);
    #ifdef CPP_API
      // Try the matching XRT memory group; fall back to group 0 for merged
      // memory interfaces (e.g. U50 PLATFORM_MERGED_MEMORY_INTERFACE) where
      // all HBM banks are exposed through a single group.  BOs are allocated
      // sequentially in group 0, so bank N lands at physical N*bank_size.
      uint32_t grp = bo_group_base_ + bank_id;
      auto xrtBuffer = [&]() -> xrt::bo {
        try {
          return xrt::bo(xrtDevice_, bank_size, xrt::bo::flags::normal, grp);
        } catch (...) {
          return xrt::bo(xrtDevice_, bank_size, xrt::bo::flags::normal, 0);
        }
      }();
    #else
      uint32_t grp = bo_group_base_ + bank_id;
      xrt_buffer_t xrtBuffer = xrtBOAlloc(xrtDevice_, bank_size, XRT_BO_FLAGS_NONE, grp);
      if (xrtBuffer == nullptr) {
        xrtBuffer = xrtBOAlloc(xrtDevice_, bank_size, XRT_BO_FLAGS_NONE, 0);
      }
      if (xrtBuffer == nullptr) {
        printf("[VXDRV] Error: xrtBOAlloc failed for bank %d\n", bank_id);
        return -1;
      }
    #endif
      xrtBuffers_.insert({bank_id, {xrtBuffer, 1}});
      if (pBuf) {
        *pBuf = xrtBuffer;
      }
    }
    return 0;
  }

#endif
};

#include <callbacks.inc>