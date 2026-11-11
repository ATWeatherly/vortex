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

// DXA descriptor-meta field widths and bit offsets, shared by the host encoder
// (dxa.h) and the SimX decoder (sim/simx/dxa). The offsets are the running sum
// of the field widths, so they live beside the encoder/decoder instead of being
// exported as generated macros. The RTL mirror lives in VX_dxa_pkg.

#pragma once

#define DXA_DESC_META_DIM_BITS        3
#define DXA_DESC_META_ELEMSZ_BITS     2
#define DXA_DESC_META_LAYOUT_BITS     2
#define DXA_DESC_META_SWIZZLE_BITS    2
#define DXA_DESC_META_INTERLEAVE_BITS 2
#define DXA_DESC_META_L2PROMO_BITS    2

#define DXA_DESC_META_DIM_LSB        0
#define DXA_DESC_META_ELEMSZ_LSB     (DXA_DESC_META_DIM_LSB + DXA_DESC_META_DIM_BITS)
#define DXA_DESC_META_LAYOUT_LSB     (DXA_DESC_META_ELEMSZ_LSB + DXA_DESC_META_ELEMSZ_BITS)
#define DXA_DESC_META_SWIZZLE_LSB    (DXA_DESC_META_LAYOUT_LSB + DXA_DESC_META_LAYOUT_BITS)
#define DXA_DESC_META_INTERLEAVE_LSB (DXA_DESC_META_SWIZZLE_LSB + DXA_DESC_META_SWIZZLE_BITS)
#define DXA_DESC_META_L2PROMO_LSB    (DXA_DESC_META_INTERLEAVE_LSB + DXA_DESC_META_INTERLEAVE_BITS)
#define DXA_DESC_META_TOTAL_BITS     (DXA_DESC_META_L2PROMO_LSB + DXA_DESC_META_L2PROMO_BITS)

// Tile-stream-unit fields above the running-sum budget (RTL: VX_dxa_pkg).
//   DEST_KIND: 0 = LMEM, 1 = TILEBUF_B, 2 = TILEBUF_A (only LMEM in phase 1)
//   SLOT     : tile-buffer slot (phase 4)
//   ENGINE   : 0 = DXA line engine, 1 = TSU burst engine
#define DXA_DESC_META_DEST_KIND_BITS 2
#define DXA_DESC_META_SLOT_BITS      4
#define DXA_DESC_META_ENGINE_BITS    1
#define DXA_DESC_META_DEST_KIND_LSB  DXA_DESC_META_TOTAL_BITS
#define DXA_DESC_META_SLOT_LSB       (DXA_DESC_META_DEST_KIND_LSB + DXA_DESC_META_DEST_KIND_BITS)
#define DXA_DESC_META_ENGINE_LSB     (DXA_DESC_META_SLOT_LSB + DXA_DESC_META_SLOT_BITS)
