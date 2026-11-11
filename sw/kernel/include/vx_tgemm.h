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

// TCU-side sequencer intrinsics (VX_CFG_TCU_SEQ_ENABLE). Normative ISA: bench docs/spec/tgemm_isa.md.
// CUSTOM0 (0x0B), funct7 = 5. All values are warp-uniform scalars.

#ifndef __VX_TGEMM_H__
#define __VX_TGEMM_H__

#include <stdint.h>
#include <vx_intrinsics.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VX_TGEMM_FUNCT7 5

// tcfg field ids (ISA §2.3)
#define VX_TCFG_KT          0
#define VX_TCFG_X_ROWS      1
#define VX_TCFG_X_STRIDE    2
#define VX_TCFG_W_ROWS      3
#define VX_TCFG_W_LAYOUT    4
#define VX_TCFG_W_STRIDE    5
#define VX_TCFG_ACC_STRIDE  6
#define VX_TCFG_NOTIFY_BAR  7

#define VX_TCFG_W_ROWMAJOR  0
#define VX_TCFG_W_SIMT8     1

// n_mode (ISA §3)
#define VX_TGEMM_M8N64   0
#define VX_TGEMM_M4N128  1
#define VX_TGEMM_M2N256  2

static inline uint32_t vx_tgemm_ctl(uint32_t acc_tile, uint32_t x_slot, uint32_t w_slot, uint32_t n_mode,
                                    uint32_t accumulate, uint32_t notify, uint32_t k_off16) {
  return (acc_tile & 0x3f) | ((x_slot & 7) << 6) | ((w_slot & 0xf) << 9) | ((n_mode & 3) << 13)
       | ((accumulate & 1) << 15) | ((notify & 1) << 16) | ((k_off16 & 0x7ff) << 17);
}

static inline uint32_t vx_tacc_ctl(uint32_t acc_tile, uint32_t n_mode, uint32_t notify) {
  return (acc_tile & 0x3f) | ((n_mode & 3) << 6) | ((notify & 1) << 8);
}

static inline void vx_tcfg(uint32_t field, uint32_t value) {
  __asm__ volatile (".insn r %0, 5, 5, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(field), "r"(value) : "memory");
}

static inline void vx_tbuf_ldx(const void* src, uint32_t x_slot) {
  __asm__ volatile (".insn r %0, 3, 5, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(src), "r"(x_slot) : "memory");
}

static inline void vx_tbuf_ldw(const void* src, uint32_t w_slot) {
  __asm__ volatile (".insn r %0, 4, 5, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(src), "r"(w_slot) : "memory");
}

static inline void vx_tgemm_i8(uint32_t K, uint32_t ctl) {
  __asm__ volatile (".insn r %0, 0, 5, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(K), "r"(ctl) : "memory");
}

static inline void vx_tacc_st(void* lmem_dst, uint32_t ctl) {
  __asm__ volatile (".insn r %0, 1, 5, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(lmem_dst), "r"(ctl) : "memory");
}

static inline void vx_twait(void) {
  __asm__ volatile (".insn r %0, 2, 5, x0, x0, x0" :: "i"(RISCV_CUSTOM0) : "memory");
}

static inline void vx_tfence(void) {
  __asm__ volatile (".insn r %0, 6, 5, x0, x0, x0" :: "i"(RISCV_CUSTOM0) : "memory");
}

#ifdef __cplusplus
}
#endif

#endif // __VX_TGEMM_H__
