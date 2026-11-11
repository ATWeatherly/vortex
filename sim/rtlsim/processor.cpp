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

#include <VX_types.h>
#include "processor.h"

#include "Vrtlsim_shim.h"

#ifdef VCD_OUTPUT
#include <verilated_vcd_c.h>
#endif

#ifdef SAIF_OUTPUT
#include <verilated_saif_c.h>
#endif

#if defined(VCD_OUTPUT) && defined(SAIF_OUTPUT)
#error "VCD_OUTPUT and SAIF_OUTPUT cannot both be defined"
#endif


#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mem.h>
#include <host_monitor.h>
#include <cout_drainer.h>

#include <VX_config.h>
#include <ostream>
#include <list>
#include <queue>
#include <vector>
#include <sstream>
#include <unordered_map>

#include <dram_sim.h>
#include <util.h>


#ifndef TRACE_START_TIME
#define TRACE_START_TIME 0ull
#endif

#ifndef TRACE_STOP_TIME
#define TRACE_STOP_TIME -1ull
#endif

#ifndef VERILATOR_RESET_VALUE
#define VERILATOR_RESET_VALUE 2
#endif

#if (VX_CFG_XLEN == 32)
typedef uint32_t Word;
#elif (VX_CFG_XLEN == 64)
typedef uint64_t Word;
#else
#error unsupported VX_CFG_XLEN
#endif

#define VL_WDATA_GETW(lwp, i, n, w) \
  VL_SEL_IWII(0, n * w, 0, 0, lwp, i * w, w)

using namespace vortex;

static uint32_t g_mem_bank_addr_width = (VX_CFG_PLATFORM_MEMORY_ADDR_WIDTH - log2ceil(VX_CFG_PLATFORM_MEMORY_NUM_BANKS));

#ifdef VX_CFG_TSU_ENABLE
// Tile-stream masters (mirrors VX_gpu_pkg VX_STR_PORTS).
#define TSU_NUM_SOCKETS ((VX_CFG_NUM_CORES / VX_CFG_SOCKET_SIZE) > 0 ? (VX_CFG_NUM_CORES / VX_CFG_SOCKET_SIZE) : 1)
#define TSU_STR_PORTS (VX_CFG_NUM_TSU_CORES * VX_CFG_TSU_MASTERS_PER_WORKER * TSU_NUM_SOCKETS * VX_CFG_NUM_CLUSTERS)
#define TSU_BEAT_SIZE VX_CFG_PLATFORM_MEMORY_DATA_SIZE
#endif

static uint64_t timestamp = 0;

double sc_time_stamp() {
  return timestamp;
}

///////////////////////////////////////////////////////////////////////////////

static uint64_t trace_start_time = TRACE_START_TIME;
static uint64_t trace_stop_time  = TRACE_STOP_TIME;

bool sim_trace_enabled() {
  if (timestamp >= trace_start_time
   && timestamp < trace_stop_time)
    return true;
  return false;
}

///////////////////////////////////////////////////////////////////////////////

class Processor::Impl {
public:
  Impl() : dram_sim_(DramSim::platform_config()) {
    // restart sim time so a re-created model registers at time zero
    timestamp = 0;

    // A context of our own rather than the process-wide default. A model
    // registers a trace callback on its context at construction, and
    // VerilatedContext::addModel has no counterpart -- the registration lives as
    // long as the context does. On the shared default context those entries
    // accumulate across device opens and keep naming models that have since been
    // deleted, so tracing a second open walks the list and dereferences freed
    // memory. Per-context, the list dies with the models it names. Reset values,
    // the seed and the assertion window are per-context settings and move with it.
    context_ = new VerilatedContext();

    // force random values for uninitialized signals
    context_->randReset(VERILATOR_RESET_VALUE);
    context_->randSeed(50);

    // turn off assertion before reset
    context_->assertOn(false);

    // create device instance
    device_ = new Vrtlsim_shim(context_);

    if (const char* e = std::getenv("VORTEX_MEM_MAX_WR")) throttle_max_wr_ = std::atoi(e);
    if (const char* e = std::getenv("VORTEX_MEM_MAX_RD")) throttle_max_rd_ = std::atoi(e);
    if (const char* e = std::getenv("VORTEX_MEM_RAW_STALL")) throttle_raw_ = (std::atoi(e) != 0);
    if (const char* e = std::getenv("VORTEX_MEM_PIPE_LATENCY")) throttle_lat_ = std::atoll(e);
    if (const char* e = std::getenv("VORTEX_MEM_COUPLED")) throttle_coupled_ = (std::atoi(e) != 0);
  #ifdef VX_CFG_TSU_ENABLE
    // Stream-port model: VORTEX_STR_MAX_RD outstanding bursts per port (the
    // U50 platform accepts 16), first beat after VORTEX_STR_LATENCY cycles
    // (default: the main-memory floor, i.e. VORTEX_MEM_PIPE_LATENCY/2 if set,
    // else 30 cycles ~ the rtlsim DRAM model), then one beat per cycle.
    if (const char* e = std::getenv("VORTEX_STR_MAX_RD")) str_max_rd_ = std::atoi(e);
    str_lat_ticks_ = (throttle_lat_ ? throttle_lat_ : 60);
    if (const char* e = std::getenv("VORTEX_STR_LATENCY")) str_lat_ticks_ = 2 * std::atoll(e);
    printf("[sim] stream ports: %d, max_rd=%u, latency=%lu cycles\n", TSU_STR_PORTS, str_max_rd_, (unsigned long)(str_lat_ticks_ / 2));
  #endif
    if (throttle_on()) {
      printf("[sim] AXI throttle: max_wr=%u max_rd=%u raw_stall=%d pipe_latency=%lu coupled=%d\n", throttle_max_wr_, throttle_max_rd_, (int)throttle_raw_, (unsigned long)throttle_lat_, (int)throttle_coupled_);
    }

  #ifdef VCD_OUTPUT
    context_->traceEverOn(true);
    tfp_ = new VerilatedVcdC();
    device_->trace(tfp_, 99);
    const char* vcd_file = std::getenv("VCD_FILE");
    tfp_->open(vcd_file ? vcd_file : "trace.vcd");
  #endif

  #ifdef SAIF_OUTPUT
    context_->traceEverOn(true);
    sfp_ = new VerilatedSaifC();
    device_->trace(sfp_, 99);
    const char* saif_file = std::getenv("SAIF_FILE");
    sfp_->open(saif_file ? saif_file : "trace.saif");
  #endif

    ram_ = nullptr;

    // reset the device
    this->reset();

    // Turn on assertion after reset
    context_->assertOn(true);
  }

  ~Impl() {
    this->cout_flush();

  #ifdef VCD_OUTPUT
    tfp_->close();
    delete tfp_;
  #endif
  #ifdef SAIF_OUTPUT
    sfp_->close();
    delete sfp_;
  #endif

    // Model before context: the model holds a reference to the context it was
    // built with, so releasing the context first would leave that reference
    // dangling through the model's own destruction.
    // Run SystemVerilog final blocks (end-of-run perf summaries) first.
    device_->final();
    delete device_;
    delete context_;
  }

  void cout_flush() {
    for (auto& buf : print_bufs_) {
      auto str = buf.second.str();
      if (!str.empty()) {
        std::cout << "#" << buf.first << ": " << str << std::endl;
      }
    }
  }

  void attach_ram(RAM* ram) {
    ram_ = ram;
  }

  void run(HostMonitor* monitor, CoutDrainer* cout_drainer) {
  #ifndef NDEBUG
    std::cout << std::dec << timestamp << ": [sim] run()" << std::endl;
  #endif
    // pulse start for one cycle
    device_->start = 1;
    this->tick();
    device_->start = 0;

    // Upper bound on the post-start wait for busy (see below); a no-work frame
    // never asserts busy, so the wait must not be unbounded.
    constexpr uint32_t NO_WORK_TIMEOUT = 100000;

    // wait for device to go busy. A frame may legitimately have no work — e.g. a
    // fully-culled graphics draw whose rasterizer emits zero fragment waves — and
    // then busy never re-asserts after the start pulse. Bound the wait so such a
    // frame drains immediately instead of spinning forever. Real work asserts
    // busy within a few cycles of start, far below this window.
    for (uint32_t i = 0; !device_->busy && i < NO_WORK_TIMEOUT; ++i) {
      this->tick();
    }

    // wait for device to go idle, or for the HTIF tohost write.
    //
    // Standalone rtlsim has no host runtime, so a kernel that uses
    // vx_printf would back-pressure on a full COUT ring forever unless
    // it's drained here (cf. Device::drain_cout). Drainer is only wired
    // by sim/rtlsim/main.cpp — the runtime path leaves it null so the
    // runtime's own drainer is the sole COUT consumer.
    while (device_->busy) {
      this->tick();
      if (cout_drainer) cout_drainer->tick();
      if (monitor && monitor->enabled() && monitor->tick(*ram_))
        break;
    }
    if (cout_drainer) cout_drainer->tick(); // final flush

    this->cout_flush();
  }

  int dcr_write(uint32_t addr, uint32_t value) {
    device_->dcr_req_valid = 1;
    device_->dcr_req_rw    = 1;
    device_->dcr_req_addr  = addr;
    device_->dcr_req_data  = value;
    this->tick();
    device_->dcr_req_valid = 0;
    this->tick();
    return 0;
  }

  int dcr_read(uint32_t addr, uint32_t tag, uint32_t* value) {
    device_->dcr_req_valid = 1;
    device_->dcr_req_rw    = 0;
    device_->dcr_req_addr  = addr;
    device_->dcr_req_data  = tag;
    this->tick();
    device_->dcr_req_valid = 0;
    this->tick();
    // READ response is returned when dcr_rsp_valid is high
    while (!device_->dcr_rsp_valid) {
      this->tick();
    }
    *value = device_->dcr_rsp_data;
    return 0;
  }

private:

  void reset() {
    this->mem_bus_reset();
    this->dcr_bus_reset();

    print_bufs_.clear();

    for (auto& reqs : pending_mem_reqs_) {
      reqs.clear();
    }

    for (int b = 0; b < VX_CFG_PLATFORM_MEMORY_NUM_BANKS; ++b) {
      std::queue<mem_req_t*> empty;
      std::swap(dram_queue_[b], empty);
    }

    device_->start = 0;
    device_->reset = 1;

    // Hold reset high until all internal pipeline state are initialized.
    for (int i = 0; i < VX_CFG_RESET_DELAY; ++i) {
      device_->clk = 0;
      this->eval();
      device_->clk = 1;
      this->eval();
    }

    device_->reset = 0;

    // Pump clocks after reset drops to allow internal pipeline states to settle.
    for (int i = 0; i < VX_CFG_RESET_DELAY; ++i) {
      device_->clk = 0;
      this->eval();
      device_->clk = 1;
      this->eval();
    }

    for (int b = 0; b < VX_CFG_PLATFORM_MEMORY_NUM_BANKS; ++b) {
      device_->mem_req_ready[b] = 1;
    }
  #ifdef VX_CFG_TSU_ENABLE
    for (int p = 0; p < TSU_STR_PORTS; ++p) {
      device_->str_req_ready[p] = 1;
    }
  #endif
  }

  void tick() {
    device_->clk = 0;
    this->eval();

    this->mem_bus_eval(0);
  #ifdef VX_CFG_TSU_ENABLE
    this->str_bus_eval(0);
  #endif

    device_->clk = 1;
    this->eval();

    this->mem_bus_eval(1);
  #ifdef VX_CFG_TSU_ENABLE
    this->str_bus_eval(1);
  #endif

    dram_sim_.tick();

    for (int b = 0; b < VX_CFG_PLATFORM_MEMORY_NUM_BANKS; ++b) {
      if (!dram_queue_[b].empty() && dram_sim_.can_accept(b)) {
        auto mem_req = dram_queue_[b].front();
        dram_sim_.send_request(b, mem_req->bank_addr, mem_req->write, [](void* arg)->bool {
          // mark completed request as ready
          auto orig_req = reinterpret_cast<mem_req_t*>(arg);
          orig_req->ready = true;
          return true;
        }, mem_req);
        dram_queue_[b].pop();
      }
    }

  #ifndef NDEBUG
    fflush(stdout);
  #endif
  }

  void eval() {
    device_->eval();
  #ifdef VCD_OUTPUT
    if (sim_trace_enabled()) {
      tfp_->dump(timestamp);
    }
  #endif
  #ifdef SAIF_OUTPUT
    if (sim_trace_enabled()) {
      sfp_->dump(timestamp);
    }
  #endif
    ++timestamp;
  }

  void mem_bus_reset() {
    for (int b = 0; b < VX_CFG_PLATFORM_MEMORY_NUM_BANKS; ++b) {
      device_->mem_req_ready[b] = 0;
      device_->mem_rsp_valid[b] = 0;
    }
  #ifdef VX_CFG_TSU_ENABLE
    for (int p = 0; p < TSU_STR_PORTS; ++p) {
      device_->str_req_ready[p] = 0;
      device_->str_rsp_valid[p] = 0;
      str_reqs_[p].clear();
    }
  #endif
  }

#ifdef VX_CFG_TSU_ENABLE
  // One AXI read master per stream port: a request is one burst over the
  // contiguous wanted beats of a granule; beats return in order, one per cycle,
  // the first after str_lat_ticks_; bursts are served in issue order.
  void str_bus_eval(bool clk) {
    if (!clk) {
      for (int p = 0; p < TSU_STR_PORTS; ++p) {
        str_rsp_ready_[p] = device_->str_rsp_ready[p];
      }
      return;
    }
    for (int p = 0; p < TSU_STR_PORTS; ++p) {
      auto& q = str_reqs_[p];
      // retire the presented beat
      if (device_->str_rsp_valid[p] && str_rsp_ready_[p]) {
        device_->str_rsp_valid[p] = 0;
        auto& r = q.front();
        if (++r.next_beat > r.last_beat) {
          q.pop_front();
        }
      }
      // present the next beat
      if (!device_->str_rsp_valid[p] && !q.empty()) {
        auto& r = q.front();
        if (timestamp >= r.t_ready) {
          uint64_t baddr = r.byte_addr + uint64_t(r.next_beat) * TSU_BEAT_SIZE;
          ram_->read(VDataCast<void*, TSU_BEAT_SIZE>::get(device_->str_rsp_data[p]), baddr, TSU_BEAT_SIZE);
          device_->str_rsp_beat[p] = r.next_beat;
          device_->str_rsp_tag[p]  = r.tag;
          device_->str_rsp_valid[p] = 1;
        }
      }
      // accept a request
      if (device_->str_req_valid[p] && device_->str_req_ready[p]) {
        str_req_t r;
        uint32_t beaten = uint32_t(device_->str_req_beaten[p]);
        uint32_t first = 0, last = 0;
        bool found = false;
        for (uint32_t b = 0; b < 32; ++b) {
          if ((beaten >> b) & 1) { if (!found) { first = b; found = true; } last = b; }
        }
        r.byte_addr = uint64_t(device_->str_req_addr[p]) * VX_CFG_TSU_BURST_BYTES;
        r.next_beat = first;
        r.last_beat = last;
        r.tag       = uint64_t(device_->str_req_tag[p]);
        r.t_ready   = timestamp + str_lat_ticks_;
        q.push_back(r);
        ++str_bursts_;
      }
      device_->str_req_ready[p] = (str_max_rd_ == 0) || (q.size() < str_max_rd_);
    }
  }
#endif

  void mem_bus_eval(bool clk) {
    if (!clk) {
      for (int b = 0; b < VX_CFG_PLATFORM_MEMORY_NUM_BANKS; ++b) {
        mem_rd_rsp_ready_[b] = device_->mem_rsp_ready[b];
      }
      return;
    }

    for (int b = 0; b < VX_CFG_PLATFORM_MEMORY_NUM_BANKS; ++b) {
      // process memory responses
      if (device_->mem_rsp_valid[b] && mem_rd_rsp_ready_[b]) {
        device_->mem_rsp_valid[b] = 0;
      }
      if (throttle_on() && !throttle_coupled_) {
        // AXI mode: writes complete on their own (B channel), independently of
        // read-data delivery, and reads return out of order (distinct IDs).
        for (auto it = pending_mem_reqs_[b].begin(); it != pending_mem_reqs_[b].end(); ) {
          if ((*it)->write && req_done(*it)) {
            if (n_pending_wr_[b]) --n_pending_wr_[b];
            delete *it;
            it = pending_mem_reqs_[b].erase(it);
          } else {
            ++it;
          }
        }
        if (device_->mem_rsp_valid[b] == 0) {
          for (auto it = pending_mem_reqs_[b].begin(); it != pending_mem_reqs_[b].end(); ++it) {
            auto mem_rsp = *it;
            if (!mem_rsp->write && req_done(mem_rsp)) {
              device_->mem_rsp_valid[b] = 1;
              memcpy(VDataCast<void*, VX_CFG_PLATFORM_MEMORY_DATA_SIZE>::get(device_->mem_rsp_data[b]), mem_rsp->data.data(), VX_CFG_PLATFORM_MEMORY_DATA_SIZE);
              device_->mem_rsp_tag[b] = mem_rsp->tag;
              if (n_pending_rd_[b]) --n_pending_rd_[b];
              pending_mem_reqs_[b].erase(it);
              delete mem_rsp;
              break;
            }
          }
        }
      } else
      if (device_->mem_rsp_valid[b] == 0) {
        if (!pending_mem_reqs_[b].empty()) {
          auto mem_rsp_it = pending_mem_reqs_[b].begin();
          auto mem_rsp = *mem_rsp_it;
          if (req_done(mem_rsp)) {
            if (!mem_rsp->write) {
              // return read responses
              device_->mem_rsp_valid[b] = 1;
              memcpy(VDataCast<void*, VX_CFG_PLATFORM_MEMORY_DATA_SIZE>::get(device_->mem_rsp_data[b]), mem_rsp->data.data(), VX_CFG_PLATFORM_MEMORY_DATA_SIZE);
              device_->mem_rsp_tag[b] = mem_rsp->tag;
            }
            // delete the request
            if (mem_rsp->write) { if (n_pending_wr_[b]) --n_pending_wr_[b]; }
            else                { if (n_pending_rd_[b]) --n_pending_rd_[b]; }
            pending_mem_reqs_[b].erase(mem_rsp_it);
            delete mem_rsp;
          }
        }
      }

      // process memory requests
      if (device_->mem_req_valid[b] && device_->mem_req_ready[b]) {
      #if VX_CFG_PLATFORM_MEMORY_INTERLEAVE == 1
        uint64_t byte_addr = (uint64_t(device_->mem_req_addr[b]) * VX_CFG_PLATFORM_MEMORY_NUM_BANKS + b) * VX_CFG_PLATFORM_MEMORY_DATA_SIZE;
      #else
        uint64_t byte_addr = (uint64_t(device_->mem_req_addr[b]) + (b << g_mem_bank_addr_width)) * VX_CFG_PLATFORM_MEMORY_DATA_SIZE;
      #endif
        // check read/write
        if (device_->mem_req_rw[b]) {
          auto byteen = device_->mem_req_byteen[b];
          auto data = VDataCast<uint8_t*, VX_CFG_PLATFORM_MEMORY_DATA_SIZE>::get(device_->mem_req_data[b]);
          // Console output (VX_MEM_IO_COUT range) is now ordinary device
          // memory — Option C: the host runtime drains it. No tap here.
          {
            // process memory writes
            for (int i = 0; i < VX_CFG_PLATFORM_MEMORY_DATA_SIZE; i++) {
              if ((byteen >> i) & 0x1) {
                (*ram_)[byte_addr + i] = data[i];
              }
            }

            auto mem_req = new mem_req_t();
            mem_req->tag   = device_->mem_req_tag[b];
            mem_req->addr  = byte_addr;
            mem_req->bank_addr = uint64_t(device_->mem_req_addr[b]) * VX_CFG_PLATFORM_MEMORY_DATA_SIZE;
            mem_req->write = true;
            mem_req->ready = false;
            mem_req->t_issue = timestamp;

            // enqueue dram request
            dram_queue_[b].push(mem_req);

            // add to pending list
            pending_mem_reqs_[b].emplace_back(mem_req);
            ++n_pending_wr_[b];
          }
        } else {
          // process memory reads
          auto mem_req = new mem_req_t();
          mem_req->tag   = device_->mem_req_tag[b];
          mem_req->addr  = byte_addr;
          mem_req->bank_addr = uint64_t(device_->mem_req_addr[b]) * VX_CFG_PLATFORM_MEMORY_DATA_SIZE;
          mem_req->write = false;
          mem_req->ready = false;
          mem_req->t_issue = timestamp;
          ram_->read(mem_req->data.data(), byte_addr, VX_CFG_PLATFORM_MEMORY_DATA_SIZE);

          // enqueue dram request
          dram_queue_[b].push(mem_req);

          // add to pending list
          pending_mem_reqs_[b].emplace_back(mem_req);
          ++n_pending_rd_[b];
        }
      }

      // AXI-boundary throttle: decide ready for the next edge from the request
      // the device is presenting now (valid is held until accepted).
      if (throttle_on()) {
        bool rdy = true;
        if (device_->mem_req_valid[b]) {
          bool rw = device_->mem_req_rw[b];
        #if VX_CFG_PLATFORM_MEMORY_INTERLEAVE == 1
          uint64_t nxt_addr = (uint64_t(device_->mem_req_addr[b]) * VX_CFG_PLATFORM_MEMORY_NUM_BANKS + b) * VX_CFG_PLATFORM_MEMORY_DATA_SIZE;
        #else
          uint64_t nxt_addr = (uint64_t(device_->mem_req_addr[b]) + (b << g_mem_bank_addr_width)) * VX_CFG_PLATFORM_MEMORY_DATA_SIZE;
        #endif
          if (rw && throttle_max_wr_ && n_pending_wr_[b] >= throttle_max_wr_) rdy = false;
          if (!rw && throttle_max_rd_ && n_pending_rd_[b] >= throttle_max_rd_) rdy = false;
          if (!rw && throttle_raw_) {
            for (auto r : pending_mem_reqs_[b]) {
              if (r->write && r->addr == nxt_addr) { rdy = false; break; }
            }
          }
        }
        device_->mem_req_ready[b] = rdy;
      }
    }
  }

  void dcr_bus_reset() {
    device_->dcr_req_valid = 0;
    device_->dcr_req_rw    = 0;
  }

  void wait(uint32_t cycles) {
    for (int i = 0; i < cycles; ++i) {
      this->tick();
    }
  }

private:

  typedef struct {
    Vrtlsim_shim* device;
    std::array<uint8_t, VX_CFG_PLATFORM_MEMORY_DATA_SIZE> data;
    uint64_t addr;
    uint64_t bank_addr;
    uint64_t tag;
    bool write;
    bool ready;
    uint64_t t_issue;
  } mem_req_t;

  std::unordered_map<int, std::stringstream> print_bufs_;

  std::list<mem_req_t*> pending_mem_reqs_[VX_CFG_PLATFORM_MEMORY_NUM_BANKS];

  std::queue<mem_req_t*> dram_queue_[VX_CFG_PLATFORM_MEMORY_NUM_BANKS];

  // AXI-boundary throttle (env-gated, default off = legacy behaviour):
  //   VORTEX_MEM_MAX_WR=<n>   max writes outstanding per bank (VX_mem_to_axi write tracker)
  //   VORTEX_MEM_MAX_RD=<n>   max reads outstanding per bank
  //   VORTEX_MEM_RAW_STALL=1  a read at the head stalls while a write to the same
  //                           line is outstanding (the store->fill ordering stall)
  //   VORTEX_MEM_PIPE_LATENCY=<cycles>  extra fixed request->response latency
  uint32_t throttle_max_wr_ = 0;
  uint32_t throttle_max_rd_ = 0;
  bool     throttle_raw_    = false;
  uint64_t throttle_lat_    = 0;
  uint32_t n_pending_wr_[VX_CFG_PLATFORM_MEMORY_NUM_BANKS] = {};
  uint32_t n_pending_rd_[VX_CFG_PLATFORM_MEMORY_NUM_BANKS] = {};
  //   VORTEX_MEM_COUPLED=1    writes retire only in order behind read responses (a DRAM
  //                           controller that cannot make progress while read data is refused)
  bool     throttle_coupled_ = false;
  bool throttle_on() const { return throttle_max_wr_ || throttle_max_rd_ || throttle_raw_ || throttle_lat_; }
  bool req_done(const mem_req_t* r) const { return r->ready && (timestamp - r->t_issue) >= throttle_lat_; }

  std::array<bool, VX_CFG_PLATFORM_MEMORY_NUM_BANKS> mem_rd_rsp_ready_;

#ifdef VX_CFG_TSU_ENABLE
  typedef struct {
    uint64_t byte_addr;
    uint32_t next_beat;
    uint32_t last_beat;
    uint64_t tag;
    uint64_t t_ready;
  } str_req_t;
  std::deque<str_req_t> str_reqs_[TSU_STR_PORTS];
  std::array<bool, TSU_STR_PORTS> str_rsp_ready_ = {};
  uint32_t str_max_rd_    = 16;
  uint64_t str_lat_ticks_ = 60;
  uint64_t str_bursts_    = 0;
#endif

  DramSim dram_sim_;

  VerilatedContext* context_;
  Vrtlsim_shim* device_;

  RAM* ram_;

#ifdef VCD_OUTPUT
  VerilatedVcdC *tfp_;
#endif
#ifdef SAIF_OUTPUT
  VerilatedSaifC *sfp_;
#endif
};

///////////////////////////////////////////////////////////////////////////////

Processor::Processor()
  : impl_(new Impl())
{}

Processor::~Processor() {
  delete impl_;
}

void Processor::attach_ram(RAM* mem) {
  impl_->attach_ram(mem);
}

void Processor::run(HostMonitor* monitor, CoutDrainer* cout_drainer) {
  impl_->run(monitor, cout_drainer);
}

int Processor::dcr_write(uint32_t addr, uint32_t value) {
  return impl_->dcr_write(addr, value);
}

int Processor::dcr_read(uint32_t addr, uint32_t tag, uint32_t* value) {
  return impl_->dcr_read(addr, tag, value);
}