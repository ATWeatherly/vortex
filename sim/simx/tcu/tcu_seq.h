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

// TCU-side sequencer golden model (VX_CFG_TCU_SEQ_ENABLE). Normative spec: bench docs/spec/tgemm_isa.md.
//
// Functional effects of tbuf.ldx / tbuf.ldw / tgemm.i8 / tacc.st are applied when the command is accepted
// (issue order per warp, so per-warp program order is preserved); timing is a per-block / per-loader
// timeline that also models the slot / tile interlocks of ISA §4. Each warp's pending count drops when a
// command's modelled end time passes; twait / tfence hold the warp until it reaches zero.

#pragma once

#ifdef VX_CFG_TCU_SEQ_ENABLE

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <vector>

#ifndef VX_CFG_NUM_TCU_SEQ_BLOCKS
#define VX_CFG_NUM_TCU_SEQ_BLOCKS 4
#endif

namespace vortex {

class TcuSeq {
public:
  static constexpr uint32_t kBlocks    = VX_CFG_NUM_TCU_SEQ_BLOCKS;
  static constexpr uint32_t kXSlots    = 8;
  static constexpr uint32_t kXRows     = 8;
  static constexpr uint32_t kXRowBytes = 2048;
  static constexpr uint32_t kWSlots    = 4;                 // per block
  static constexpr uint32_t kWSlotBytes = 128 * 1024;
  static constexpr uint32_t kAccTiles  = 16;                // per block
  static constexpr uint32_t kAccElems  = 512;
  static constexpr uint32_t kQueueDepth = 2;
  static constexpr uint32_t kGemmFill  = 80;
  static constexpr uint32_t kGlobalLat = 200;
  static constexpr uint32_t kFenceDrain = 64;

  // WSCALE / XSCALE / LIMITS: tacc.st fp32 dequant (ctl bit 9, RTL VX_CFG_TCU_SEQ_STF)
  enum CfgField : uint32_t { KT = 0, X_ROWS, X_STRIDE, W_ROWS, W_LAYOUT, W_STRIDE, ACC_STRIDE, NOTIFY_BAR,
                             WSCALE, XSCALE, LIMITS, NUM_CFG };

  struct PerfStats {
    uint64_t gemms = 0, gemm_k = 0, loads_x = 0, loads_w = 0, acc_st = 0, busy = 0, landings = 0;
  };

  // Memory access callbacks bound by the owner (LMEM window vs global backing store).
  struct MemIf {
    std::function<bool(uint64_t)> is_lmem;
    std::function<void(uint64_t, void*, uint64_t)> read;
    std::function<void(uint64_t, const void*, uint64_t)> write;
    std::function<void(uint32_t)> notify;     // raw barrier id
  };

  TcuSeq() { reset(); }

  void bind(const MemIf& mem) { mem_ = mem; }

  void reset() {
    for (auto& s : x_) s.assign(kXRows * kXRowBytes, 0);
    w_.assign(kBlocks, std::vector<std::vector<int8_t>>(kWSlots, std::vector<int8_t>(kWSlotBytes, 0)));
    acc_.assign(kBlocks, std::vector<std::vector<int32_t>>(kAccTiles, std::vector<int32_t>(kAccElems, 0)));
    w_rows_.assign(kBlocks, std::vector<uint32_t>(kWSlots, 0));
    w_kt_.assign(kBlocks, std::vector<uint32_t>(kWSlots, 0));
    for (auto& c : cfg_) default_cfg(c);
    pending_.fill(0);
    fence_due_.fill(0);
    events_.clear();
    blk_free_.fill(0); ldw_free_.fill(0); acc_st_free_.fill(0);
    ldx_free_ = 0;
    w_ready_.assign(kBlocks, std::vector<uint64_t>(kWSlots, 0));
    w_read_end_.assign(kBlocks, std::vector<uint64_t>(kWSlots, 0));
    acc_ready_.assign(kBlocks, std::vector<uint64_t>(kAccTiles, 0));
    x_ready_.fill(0); x_read_end_.fill(0);
    staging_.assign(kBlocks, std::vector<std::deque<Landing>>(kWSlots));
    expect_.assign(kBlocks, std::vector<std::deque<uint32_t>>(kWSlots));
  }

  // ---- phase-4 tile-stream landing (TSU -> W slot) ----
  // The DXA/TSU model delivers a transfer's bytes into a per-slot staging image (landings into one slot arrive in
  // issue order: the RTL steers them to one TSU worker); land_done() closes it. A tbuf.ldws expectation is matched
  // with the oldest closed image in tick(): the image becomes the slot's contents (functional effect) and the
  // expectation retires (pending--). Because the ldws is accepted after every earlier tgemm on the slot (in-order
  // issue), overwriting the slot then is safe; a tgemm on a slot with an unmatched expectation is held.
  void land(uint32_t tag, uint64_t off, const void* data, uint32_t size) {
    uint32_t lg = (tag >> 4) & 0xfu, b = (tag >> 2) & 3u, s = tag & 3u;
    assert(b < kBlocks && lg >= 6 && lg <= 11 && off + size <= kWSlotBytes);
    auto& q = staging_.at(b).at(s);
    if (q.empty() || q.back().closed) q.push_back(Landing{std::vector<int8_t>(kWSlotBytes, 0), 1u << lg, false, 0});
    if ((tag >> 8) & 1u) {   // tile-local SIMT8 image: byte o = chunk o >> 9, row (o >> 3) & 63, byte o & 7
      const uint32_t kt = 1u << lg;
      for (uint32_t i = 0; i < size; ++i) {
        const uint64_t o = off + i, k = (o >> 9) * 8 + (o & 7), n = (o >> 3) & 63;
        assert(k < kt);
        q.back().img[n * kt + k] = static_cast<const int8_t*>(data)[i];
      }
      return;
    }
    std::memcpy(q.back().img.data() + off, data, size);
  }
  void land_done(uint32_t tag, uint64_t now) {
    uint32_t b = (tag >> 2) & 3u, s = tag & 3u;
    auto& q = staging_.at(b).at(s);
    if (q.empty() || q.back().closed) q.push_back(Landing{std::vector<int8_t>(kWSlotBytes, 0), 1u << ((tag >> 4) & 0xfu), false, 0});
    q.back().closed = true;
    q.back().at = now;
    ++perf_.landings;
  }

  // Retire commands whose modelled end time has passed.
  void tick(uint64_t now) {
    for (uint32_t b = 0; b < kBlocks; ++b) {
      for (uint32_t s = 0; s < kWSlots; ++s) {
        auto& eq = expect_.at(b).at(s);
        auto& sq = staging_.at(b).at(s);
        while (!eq.empty() && !sq.empty() && sq.front().closed && sq.front().at <= now) {
          w_.at(b).at(s).swap(sq.front().img);
          w_rows_.at(b).at(s) = 64;
          w_kt_.at(b).at(s) = sq.front().kt;
          w_ready_.at(b).at(s) = now;
          --pending_.at(eq.front());
          eq.pop_front();
          sq.pop_front();
        }
      }
    }
    while (!events_.empty()) {
      bool any = false;
      for (auto it = events_.begin(); it != events_.end(); ) {
        if (it->end <= now) {
          --pending_.at(it->wid);
          if (it->notify && mem_.notify) mem_.notify(it->bar);
          it = events_.erase(it);
          any = true;
        } else {
          ++it;
        }
      }
      if (!any) break;
    }
    if (!events_.empty()) ++perf_.busy;
  }

  // Try to accept one sequencer instruction. Returns false to hold it this cycle.
  bool issue(TcuType type, uint32_t wid, uint32_t rs1, uint32_t rs2, uint64_t now) {
    auto& c = cfg_.at(wid);
    switch (type) {
    case TcuType::TCFG:
      if (rs1 >= NUM_CFG) { std::cerr << "*** tcfg: bad field " << rs1 << std::endl; std::abort(); }
      c[rs1] = rs2;
      return true;
    case TcuType::TWAIT:
      return pending_.at(wid) == 0;
    case TcuType::TFENCE:
      if (pending_.at(wid) != 0) return false;
      if (fence_due_.at(wid) == 0) { fence_due_.at(wid) = now + kFenceDrain; return false; }
      if (now < fence_due_.at(wid)) return false;
      fence_due_.at(wid) = 0;
      return true;
    case TcuType::TBUF_LDX: return ldx(wid, rs1, rs2, now);
    case TcuType::TBUF_LDW: return ldw(wid, rs1, rs2, now);
    case TcuType::TBUF_LDWS: {
      uint32_t b = (rs2 >> 2) & 3u, s = rs2 & 3u;
      assert(b < kBlocks);
      auto& eq = expect_.at(b).at(s);
      if (eq.size() >= kQueueDepth) return false;
      eq.push_back(wid);
      ++pending_.at(wid);
      return true;
    }
    case TcuType::TGEMM:    return gemm(wid, rs1, rs2, now);
    case TcuType::TACC_ST:  return acc_st(wid, rs1, rs2, now);
    default: std::abort();
    }
  }

  const PerfStats& perf() const { return perf_; }

private:
  struct Event { uint64_t end; uint32_t wid; bool notify; uint32_t bar; uint32_t queue; };

  static void default_cfg(std::array<uint32_t, NUM_CFG>& c) {
    c.fill(0);
    c[KT] = 2048; c[X_ROWS] = 8; c[X_STRIDE] = 2048; c[W_ROWS] = 64; c[W_LAYOUT] = 0; c[W_STRIDE] = 2048;
    c[LIMITS] = 0x840;
  }

  static uint32_t rows_of(uint32_t mode) { return 8u >> mode; }
  static uint32_t cols_of(uint32_t mode) { return 64u << mode; }

  // Queue ids: 0..kBlocks-1 = blocks, kBlocks = core X loader.
  uint32_t outstanding(uint32_t queue, uint64_t now) const {
    uint32_t n = 0;
    for (auto& e : events_) if (e.queue == queue && e.end > now) ++n;
    return n;
  }

  void push(uint64_t end, uint32_t wid, uint32_t queue, bool notify = false, uint32_t bar = 0) {
    events_.push_back({end, wid, notify, bar, queue});
    ++pending_.at(wid);
  }

  void read_mem(uint64_t addr, void* dst, uint64_t size) { mem_.read(addr, dst, size); }

  bool ldx(uint32_t wid, uint32_t src, uint32_t rs2, uint64_t now) {
    auto& c = cfg_.at(wid);
    uint32_t slot = rs2 & 7u;
    if (outstanding(kBlocks, now) >= kQueueDepth) return false;
    uint32_t rows = c[X_ROWS], kt = c[KT];
    assert(rows >= 1 && rows <= kXRows && kt % 16 == 0 && kt <= kXRowBytes);
    auto& s = x_.at(slot);
    std::fill(s.begin(), s.end(), 0);
    for (uint32_t m = 0; m < rows; ++m)
      read_mem((uint64_t)src + (uint64_t)m * c[X_STRIDE], s.data() + m * kXRowBytes, kt);
    bool lm = mem_.is_lmem(src);
    uint64_t dur = lm ? (uint64_t)rows * kt / 32 : kGlobalLat + (uint64_t)rows * kt / 64;
    uint64_t start = std::max({now + 1, ldx_free_, x_read_end_.at(slot)});
    ldx_free_ = start + (lm ? dur : dur - kGlobalLat);
    x_ready_.at(slot) = start + dur;
    push(start + dur, wid, kBlocks);
    ++perf_.loads_x;
    return true;
  }

  uint64_t w_addr(const std::array<uint32_t, NUM_CFG>& c, uint32_t src, uint32_t n, uint32_t k) const {
    return c[W_LAYOUT] ? (uint64_t)src + (uint64_t)(k / 8) * c[W_STRIDE] + (uint64_t)n * 8 + (k % 8)
                       : (uint64_t)src + (uint64_t)n * c[W_STRIDE] + k;
  }

  bool ldw(uint32_t wid, uint32_t src, uint32_t rs2, uint64_t now) {
    auto& c = cfg_.at(wid);
    uint32_t b = (rs2 >> 2) & 3u, slot = rs2 & 3u;
    assert(b < kBlocks);
    if (outstanding(b, now) >= kQueueDepth) return false;
    uint32_t rows = c[W_ROWS], kt = c[KT];
    assert(rows >= 1 && kt % 16 == 0 && (uint64_t)rows * kt <= kWSlotBytes);
    auto& s = w_.at(b).at(slot);
    std::fill(s.begin(), s.begin() + std::min<uint64_t>(kWSlotBytes, (uint64_t)256 * kt), 0);
    if (c[W_LAYOUT]) {
      // SIMT8: (k/8)*S + n*8 + k%8 -- rows of one 8-k chunk are contiguous (rows*8 bytes)
      std::vector<int8_t> chunk((size_t)rows * 8);
      for (uint32_t k8 = 0; k8 < kt; k8 += 8) {
        read_mem((uint64_t)src + (uint64_t)(k8 / 8) * c[W_STRIDE], chunk.data(), chunk.size());
        for (uint32_t n = 0; n < rows; ++n)
          std::memcpy(s.data() + (size_t)n * kt + k8, chunk.data() + (size_t)n * 8, 8);
      }
    } else {
      for (uint32_t n = 0; n < rows; ++n)
        read_mem((uint64_t)src + (uint64_t)n * c[W_STRIDE], s.data() + (size_t)n * kt, kt);
    }
    w_rows_.at(b).at(slot) = rows;
    w_kt_.at(b).at(slot) = kt;
    bool lm = mem_.is_lmem(src);
    uint64_t dur = lm ? (uint64_t)rows * kt / 32 : kGlobalLat + (uint64_t)rows * kt / 128;
    uint64_t start = std::max({now + 1, ldw_free_.at(b), w_read_end_.at(b).at(slot)});
    ldw_free_.at(b) = start + (lm ? dur : dur - kGlobalLat);
    w_ready_.at(b).at(slot) = start + dur;
    push(start + dur, wid, b);
    ++perf_.loads_w;
    return true;
  }

  bool gemm(uint32_t wid, uint32_t K, uint32_t ctl, uint64_t now) {
    auto& c = cfg_.at(wid);
    uint32_t acc_tile = ctl & 0x3fu, b = acc_tile >> 4, t = acc_tile & 0xfu;
    uint32_t xs = (ctl >> 6) & 7u, ws = (ctl >> 9) & 0xfu, wb = ws >> 2, wsl = ws & 3u;
    uint32_t mode = (ctl >> 13) & 3u;
    bool accumulate = (ctl >> 15) & 1u, notify = (ctl >> 16) & 1u;
    uint32_t koff = ((ctl >> 17) & 0x7ffu) * 16;
    if (b >= kBlocks || wb != b || mode > 2 || K % 16 != 0) {
      std::cerr << "*** tgemm: illegal control 0x" << std::hex << ctl << std::dec << " K=" << K << std::endl;
      std::abort();
    }
    if (outstanding(b, now) >= kQueueDepth) return false;
    if (!expect_.at(b).at(wsl).empty()) return false;   // W slot still waiting for its stream landing
    const uint32_t rows = rows_of(mode), cols = cols_of(mode);
    const uint32_t wkt = w_kt_.at(b).at(wsl);
    assert(koff + K <= wkt && koff + K <= kXRowBytes);
    assert((uint64_t)cols * wkt <= kWSlotBytes);
    const auto& X = x_.at(xs);
    const auto& W = w_.at(b).at(wsl);
    auto& D = acc_.at(b).at(t);
    for (uint32_t m = 0; m < rows; ++m) {
      const int8_t* xr = X.data() + (size_t)m * kXRowBytes + koff;
      for (uint32_t n = 0; n < cols; ++n) {
        const int8_t* wr = W.data() + (size_t)n * wkt + koff;
        int64_t s = 0;
        for (uint32_t k = 0; k < K; ++k) s += (int32_t)xr[k] * (int32_t)wr[k];
        uint32_t prev = accumulate ? (uint32_t)D[m * cols + n] : 0u;
        D[m * cols + n] = (int32_t)(prev + (uint32_t)(int32_t)s);
      }
    }
    uint64_t start = std::max({now + 1, blk_free_.at(b), w_ready_.at(b).at(wsl), x_ready_.at(xs), acc_ready_.at(b).at(t)});
    blk_free_.at(b) = start + K;
    uint64_t end = start + K + kGemmFill;
    acc_ready_.at(b).at(t) = end;
    w_read_end_.at(b).at(wsl) = std::max(w_read_end_.at(b).at(wsl), end);
    x_read_end_.at(xs) = std::max(x_read_end_.at(xs), end);
    push(end, wid, b, notify, c[NOTIFY_BAR]);
    ++perf_.gemms; perf_.gemm_k += K;
    return true;
  }

  bool acc_st(uint32_t wid, uint32_t dst, uint32_t ctl, uint64_t now) {
    auto& c = cfg_.at(wid);
    uint32_t acc_tile = ctl & 0x3fu, b = acc_tile >> 4, t = acc_tile & 0xfu;
    uint32_t mode = (ctl >> 6) & 3u;
    bool notify = (ctl >> 8) & 1u;
    if (((ctl >> 9) & 1u) && b < kBlocks) return acc_st_fp(wid, dst, ctl, now);
    if (b >= kBlocks || mode > 2 || !mem_.is_lmem(dst) || (dst & 3u)) {
      std::cerr << "*** tacc.st: illegal dst 0x" << std::hex << dst << " ctl 0x" << ctl << std::dec << std::endl;
      std::abort();
    }
    if (outstanding(b, now) >= kQueueDepth) return false;
    const uint32_t rows = rows_of(mode), cols = cols_of(mode);
    const uint32_t stride = c[ACC_STRIDE] ? c[ACC_STRIDE] : cols * 4;
    const auto& D = acc_.at(b).at(t);
    for (uint32_t m = 0; m < rows; ++m)
      mem_.write((uint64_t)dst + (uint64_t)m * stride, D.data() + m * cols, cols * 4);
    uint64_t dur = (uint64_t)rows * cols * 4 / 32;
    uint64_t start = std::max({now + 1, acc_ready_.at(b).at(t), acc_st_free_.at(b)});
    acc_st_free_.at(b) = start + dur;
    push(start + dur, wid, b, notify, c[NOTIFY_BAR]);
    ++perf_.acc_st;
    return true;
  }

  // tacc.st fp: C[dst + n*ACC_STRIDE + 4m] = 0.0f + ((float)acc[m][n] * ws[n]) * xs[m], n < nlim, m < mlim (fp32 RNE,
  // each operation rounded: the host's rescale order). Timing: scale fetch + 2 values per cycle (RTL dequant unit).
  bool acc_st_fp(uint32_t wid, uint32_t dst, uint32_t ctl, uint64_t now) {
    auto& c = cfg_.at(wid);
    uint32_t acc_tile = ctl & 0x3fu, b = acc_tile >> 4, t = acc_tile & 0xfu, mode = (ctl >> 6) & 3u;
    const uint32_t nlim = c[LIMITS] & 0x7fu, mlim = (c[LIMITS] >> 8) & 0xfu, stride = c[ACC_STRIDE];
    if (mode != 0 || mem_.is_lmem(dst) || (dst & 3u) || (stride & 3u) || stride == 0 || nlim == 0 || nlim > 64
        || mlim == 0 || mlim > 8 || ((ctl >> 8) & 1u)) {
      std::cerr << "*** tacc.st fp: illegal dst 0x" << std::hex << dst << " ctl 0x" << ctl << " limits 0x" << c[LIMITS]
                << std::dec << " stride " << stride << std::endl;
      std::abort();
    }
    if (outstanding(b, now) >= kQueueDepth) return false;
    float ws[64], xs[8];
    mem_.read(c[WSCALE], ws, (uint64_t)nlim * 4);
    mem_.read(c[XSCALE], xs, (uint64_t)mlim * 4);
    const auto& D = acc_.at(b).at(t);
    for (uint32_t n = 0; n < nlim; ++n) {
      float row[8];
      for (uint32_t m = 0; m < mlim; ++m) {
        volatile float f  = (float)D[m * 64 + n];
        volatile float p1 = f * ws[n];
        volatile float p2 = p1 * xs[m];
        row[m] = 0.0f + p2;
      }
      mem_.write((uint64_t)dst + (uint64_t)n * stride, row, (uint64_t)mlim * 4);
    }
    uint64_t dur = kGlobalLat + 64 * 8 / 2;
    uint64_t start = std::max({now + 1, acc_ready_.at(b).at(t), acc_st_free_.at(b)});
    acc_st_free_.at(b) = start + 64 * 8 / 2;
    push(start + dur, wid, b);
    ++perf_.acc_st;
    return true;
  }

  struct Landing { std::vector<int8_t> img; uint32_t kt; bool closed; uint64_t at; };
  std::vector<std::vector<std::deque<Landing>>> staging_;
  std::vector<std::vector<std::deque<uint32_t>>> expect_;   // tbuf.ldws expectations: issuing wid

  MemIf mem_;
  std::array<std::vector<int8_t>, kXSlots> x_;
  std::vector<std::vector<std::vector<int8_t>>> w_;
  std::vector<std::vector<std::vector<int32_t>>> acc_;
  std::vector<std::vector<uint32_t>> w_rows_, w_kt_;
  std::array<std::array<uint32_t, NUM_CFG>, VX_CFG_NUM_WARPS> cfg_;
  std::array<uint32_t, VX_CFG_NUM_WARPS> pending_;
  std::array<uint64_t, VX_CFG_NUM_WARPS> fence_due_;
  std::deque<Event> events_;
  std::array<uint64_t, kBlocks> blk_free_, ldw_free_, acc_st_free_;
  uint64_t ldx_free_ = 0;
  std::vector<std::vector<uint64_t>> w_ready_, w_read_end_, acc_ready_;
  std::array<uint64_t, kXSlots> x_ready_, x_read_end_;
  PerfStats perf_;
};

} // namespace vortex

#endif // VX_CFG_TCU_SEQ_ENABLE
