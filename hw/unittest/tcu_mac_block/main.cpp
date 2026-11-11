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

// Self-checking test of VX_tcu_mac_block (512-MAC int8 tensor block, spec phase 3 track B).
// Drives spans of random / worst-case int8 and uint8 operands over random K
// (multiples of 4 and not, 1..4096), with random input bubbles and output
// back-pressure, and checks every drained int32 against an exact reference.

#include "VVX_tcu_mac_block.h"
#include <verilated.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <type_traits>
#include <vector>

#ifndef ROWS
#define ROWS 8
#endif
#ifndef COLS
#define COLS 16
#endif
#ifndef OUT_COLS
#define OUT_COLS 2
#endif

static constexpr int BEATS = COLS / OUT_COLS;

double sc_time_stamp() { return 0; }

// --- bit access on Verilator port types (QData or VlWide) -------------------
template <typename T> static void put8(T& sig, int idx, uint8_t v) {
  if constexpr (std::is_integral_v<T>) {
    sig &= ~(T(0xff) << (idx * 8));
    sig |= T(v) << (idx * 8);
  } else {
    int w = idx / 4, s = (idx % 4) * 8;
    sig[w] = (sig[w] & ~(0xffu << s)) | (uint32_t(v) << s);
  }
}
template <typename T> static uint32_t get32(const T& sig, int idx) {
  if constexpr (std::is_integral_v<T>) {
    return uint32_t(sig >> (idx * 32));
  } else {
    return sig[idx];
  }
}

struct Span {
  int K;
  bool sgn;
  std::vector<uint8_t> a;   // [k][ROWS]
  std::vector<uint8_t> b;   // [k][COLS]
  std::vector<uint32_t> d;  // [ROWS][COLS] expected
  const char* name;
};

static std::mt19937_64 rng(12345);

static Span make_span(int K, bool sgn, int pat, const char* name) {
  Span s;
  s.K = K; s.sgn = sgn; s.name = name;
  s.a.resize(size_t(K) * ROWS);
  s.b.resize(size_t(K) * COLS);
  auto gen = [&](size_t i, int k) -> uint8_t {
    switch (pat) {
    case 1: return sgn ? 0x80 : 0xff;                         // all min (signed) / all max (unsigned)
    case 2: return sgn ? 0x7f : 0xff;                         // all max
    case 3: return ((i + k) & 1) ? (sgn ? 0x80 : 0xff) : (sgn ? 0x7f : 0x00);  // alternating extremes
    case 4: return (rng() & 1) ? (sgn ? 0x80 : 0xff) : (sgn ? 0x7f : 0xff);    // random extremes
    default: return uint8_t(rng());
    }
  };
  for (int k = 0; k < K; ++k) {
    for (int r = 0; r < ROWS; ++r) s.a[size_t(k) * ROWS + r] = gen(r, k);
    for (int c = 0; c < COLS; ++c) s.b[size_t(k) * COLS + c] = gen(c + 1, k);
  }
  s.d.assign(ROWS * COLS, 0);
  for (int r = 0; r < ROWS; ++r) {
    for (int c = 0; c < COLS; ++c) {
      int64_t acc = 0;
      for (int k = 0; k < K; ++k) {
        int32_t x = sgn ? int32_t(int8_t(s.a[size_t(k) * ROWS + r])) : int32_t(s.a[size_t(k) * ROWS + r]);
        int32_t w = sgn ? int32_t(int8_t(s.b[size_t(k) * COLS + c])) : int32_t(s.b[size_t(k) * COLS + c]);
        acc += int64_t(x) * w;
      }
      s.d[r * COLS + c] = uint32_t(acc);
    }
  }
  return s;
}

int main(int argc, char** argv) {
  Verilated::commandArgs(argc, argv);
  if (const char* e = std::getenv("TEST_SEED")) rng.seed(std::strtoull(e, nullptr, 10));
  auto* top = new VVX_tcu_mac_block;

  // ---- test list -----------------------------------------------------------
  std::vector<Span> spans;
  const int ks[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 15, 16, 17, 31, 32, 33, 36, 63, 64, 65, 100,
                    255, 256, 257, 1023, 1024, 4095, 4096};
  for (int K : ks) {
    spans.push_back(make_span(K, true,  0, "rand-i8"));
    spans.push_back(make_span(K, false, 0, "rand-u8"));
  }
  for (int pat = 1; pat <= 4; ++pat) {
    for (int K : {1, 3, 4, 5, 64, 4096}) {
      spans.push_back(make_span(K, true,  pat, "extreme-i8"));
      spans.push_back(make_span(K, false, pat, "extreme-u8"));
    }
  }
  for (int i = 0; i < 40; ++i) {
    int K = 1 + int(rng() % 600);
    spans.push_back(make_span(K, (rng() & 1) != 0, (rng() % 5 == 0) ? 4 : 0, "rand-mix"));
  }

  // ---- simulation ------------------------------------------------------------
  auto tick = [&]() {
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
  };
  top->reset = 1; top->in_valid = 0; top->out_ready = 0;
  for (int i = 0; i < 4; ++i) tick();
  top->reset = 0;

  size_t send_span = 0, send_k = 0;     // next beat to present
  size_t chk_span = 0; int chk_beat = 0;
  uint64_t cycles = 0, accepted = 0, stall_ready = 0, errors = 0, checked = 0;
  // phase A (first half of the spans): random bubbles + back-pressure; phase B: full rate
  const size_t phaseB = spans.size() / 2;
  bool have_beat = false;

  while (chk_span < spans.size()) {
    if (++cycles > 100000000ull) { printf("TIMEOUT\n"); return 1; }
    bool full_rate = send_span >= phaseB;

    // present a beat (hold it until accepted: valid/ready)
    if (!have_beat && send_span < spans.size()) {
      bool bubble = !full_rate && (rng() % 8 == 0);
      have_beat = !bubble;
    }
    if (have_beat) {
      const Span& s = spans[send_span];
      top->in_valid  = 1;
      top->in_first  = (send_k == 0);
      top->in_last   = (send_k + 1 == size_t(s.K));
      top->in_signed = s.sgn;
      for (int r = 0; r < ROWS; ++r) put8(top->in_a, r, s.a[send_k * ROWS + r]);
      for (int c = 0; c < COLS; ++c) put8(top->in_b, c, s.b[send_k * COLS + c]);
    } else {
      top->in_valid = 0;
    }
    top->out_ready = full_rate ? 1 : ((rng() % 5) != 0);

    top->clk = 0; top->eval();       // combinational in_ready / outputs for this edge
    bool acc = top->in_valid && top->in_ready;
    if (top->in_valid && !top->in_ready) ++stall_ready;
    bool out_fire = top->out_valid && top->out_ready;
    if (out_fire) {
      const Span& s = spans[chk_span];
      if (int(top->out_col) != chk_beat) {
        printf("ERROR span %zu (%s K=%d): out_col %d expected %d\n", chk_span, s.name, s.K, int(top->out_col), chk_beat);
        ++errors;
      }
      for (int j = 0; j < OUT_COLS; ++j) {
        int col = j * BEATS + chk_beat;
        for (int r = 0; r < ROWS; ++r) {
          uint32_t got = get32(top->out_data, j * ROWS + r);
          uint32_t exp = s.d[r * COLS + col];
          ++checked;
          if (got != exp) {
            if (errors < 20)
              printf("ERROR span %zu (%s K=%d sgn=%d) row %d col %d: got %d expected %d\n",
                     chk_span, s.name, s.K, s.sgn, r, col, int32_t(got), int32_t(exp));
            ++errors;
          }
        }
      }
      bool last = top->out_last;
      if (last != (chk_beat == BEATS - 1)) { printf("ERROR out_last at beat %d\n", chk_beat); ++errors; }
      if (++chk_beat == BEATS) { chk_beat = 0; ++chk_span; }
    }
    top->clk = 1; top->eval();       // posedge
    if (acc) {
      ++accepted;
      have_beat = false;
      if (++send_k == size_t(spans[send_span].K)) { send_k = 0; ++send_span; }
    }
  }

  // full-rate throughput on the phase-B spans is reported for information
  uint64_t total_k = 0;
  for (auto& s : spans) total_k += s.K;
  printf("ROWS=%d COLS=%d OUT_COLS=%d: %zu spans, %llu k-steps, %llu int32 checked, %llu cycles, %llu ready-stalls\n",
         ROWS, COLS, OUT_COLS, spans.size(), (unsigned long long)total_k, (unsigned long long)checked,
         (unsigned long long)cycles, (unsigned long long)stall_ready);
  if (errors) {
    printf("FAILED: %llu mismatches\n", (unsigned long long)errors);
    return 1;
  }
  printf("PASSED\n");
  top->final();
  delete top;
  return 0;
}
