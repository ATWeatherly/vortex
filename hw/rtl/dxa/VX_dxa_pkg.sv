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

`ifndef VX_DXA_PKG_VH
`define VX_DXA_PKG_VH

`include "VX_define.vh"

package VX_dxa_pkg;

    import VX_gpu_pkg::*;

    // DXA local-memory path: dimensioned to cover all banks in one request.
    localparam DXA_LMEM_WORD_SIZE = `VX_CFG_LMEM_NUM_BANKS * (`VX_CFG_XLEN / 8);
    localparam DXA_LMEM_ADDR_W = LMEM_DMA_ADDR_WIDTH;

    // LMEM byte-address width: an LMEM-relative byte offset, same as
    // VX_local_mem's ADDR_WIDTH = `CLOG2(SIZE) = `VX_CFG_LMEM_LOG_SIZE.
    // Phase-4 W-slot landing (TSU + TCU sequencer): the destination of a TILEBUF_B transfer is a dense byte
    // offset inside one 64-row x kt W slot (<= 128 KB), so the byte-address field is widened to 17 bits there.
    // LMEM transfers are unaffected: every LMEM-side use truncates explicitly to the LMEM word address.
`ifdef VX_CFG_TSU_ENABLE
`ifdef VX_CFG_TCU_SEQ_ENABLE
    localparam DXA_WLAND_OFF_W = 17;
    localparam DXA_SMEM_ADDR_W = (`VX_CFG_LMEM_LOG_SIZE > DXA_WLAND_OFF_W) ? `VX_CFG_LMEM_LOG_SIZE : DXA_WLAND_OFF_W;
`else
    localparam DXA_SMEM_ADDR_W = `VX_CFG_LMEM_LOG_SIZE;
`endif
`else
    localparam DXA_SMEM_ADDR_W = `VX_CFG_LMEM_LOG_SIZE;
`endif

    localparam DXA_DESC_SLOT_BITS = `CLOG2(`VX_DCR_DXA_DESC_COUNT);
    localparam DXA_DESC_SLOT_W    = `UP(DXA_DESC_SLOT_BITS);

    // Descriptor-meta field widths and bit offsets (offsets = running sum of the
    // widths). RTL-owned; the host/SimX encoder-decoder derive the same locally.
    localparam DXA_DESC_META_DIM_BITS        = 3;
    localparam DXA_DESC_META_ELEMSZ_BITS     = 2;
    localparam DXA_DESC_META_LAYOUT_BITS     = 2;
    localparam DXA_DESC_META_SWIZZLE_BITS    = 2;
    localparam DXA_DESC_META_INTERLEAVE_BITS = 2;
    localparam DXA_DESC_META_L2PROMO_BITS    = 2;
    localparam DXA_DESC_META_DIM_LSB        = 0;
    localparam DXA_DESC_META_ELEMSZ_LSB     = DXA_DESC_META_DIM_LSB + DXA_DESC_META_DIM_BITS;
    localparam DXA_DESC_META_LAYOUT_LSB     = DXA_DESC_META_ELEMSZ_LSB + DXA_DESC_META_ELEMSZ_BITS;
    localparam DXA_DESC_META_SWIZZLE_LSB    = DXA_DESC_META_LAYOUT_LSB + DXA_DESC_META_LAYOUT_BITS;
    localparam DXA_DESC_META_INTERLEAVE_LSB = DXA_DESC_META_SWIZZLE_LSB + DXA_DESC_META_SWIZZLE_BITS;
    localparam DXA_DESC_META_L2PROMO_LSB    = DXA_DESC_META_INTERLEAVE_LSB + DXA_DESC_META_INTERLEAVE_BITS;
    // Total descriptor-meta bit budget (running sum through the last field); the
    // encoder packs it into the 32-bit meta ABI word, checked in VX_dxa_setup.
    localparam DXA_DESC_META_TOTAL_BITS     = DXA_DESC_META_L2PROMO_LSB + DXA_DESC_META_L2PROMO_BITS;

    // Tile-stream-unit fields, placed above the running-sum budget so the
    // existing encoder/decoder pairs are untouched (mirrored in sw/common/
    // dxa_meta.h and sim/simx/dxa/dxa_core.cpp):
    //   DEST_KIND [14:13]: 0 = LMEM (the only destination in phase 1),
    //                      1 = TILEBUF_B, 2 = TILEBUF_A (reserved for phase 4)
    //   SLOT      [18:15]: tile-buffer slot (phase 4)
    //   ENGINE    [19]   : 0 = DXA line engine, 1 = TSU burst engine
    /* verilator lint_off UNUSEDPARAM */
    localparam DXA_DESC_META_DEST_KIND_BITS = 2;
    localparam DXA_DESC_META_SLOT_BITS      = 4;
    localparam DXA_DESC_META_ENGINE_BITS    = 1;
    localparam DXA_DESC_META_DEST_KIND_LSB  = DXA_DESC_META_TOTAL_BITS;
    localparam DXA_DESC_META_SLOT_LSB       = DXA_DESC_META_DEST_KIND_LSB + DXA_DESC_META_DEST_KIND_BITS;
    localparam DXA_DESC_META_ENGINE_LSB     = DXA_DESC_META_SLOT_LSB + DXA_DESC_META_SLOT_BITS;
    localparam DXA_DEST_KIND_LMEM      = 2'd0;
    localparam DXA_DEST_KIND_TILEBUF_B = 2'd1;
    localparam DXA_DEST_KIND_TILEBUF_A = 2'd2;
    localparam DXA_ENGINE_DXA = 1'b0;
    localparam DXA_ENGINE_TSU = 1'b1;

    // TSU geometry (see VX_config.toml [dxa]).
    localparam TSU_BURST_BYTES  = `VX_CFG_TSU_BURST_BYTES;
    localparam TSU_BEAT_BYTES   = `VX_CFG_PLATFORM_MEMORY_DATA_SIZE;
    localparam TSU_NUM_BEATS    = TSU_BURST_BYTES / TSU_BEAT_BYTES;
    localparam TSU_MAX_INFLIGHT = `VX_CFG_TSU_MAX_INFLIGHT;
    localparam TSU_TAG_W        = `CLOG2(TSU_MAX_INFLIGHT);
    localparam TSU_STR_TAG_W    = UUID_WIDTH + TSU_TAG_W;
    localparam TSU_STR_ADDR_W   = `VX_CFG_MEM_ADDR_WIDTH - `CLOG2(TSU_BURST_BYTES);
    localparam TSU_NUM_WORKERS  = `VX_CFG_NUM_TSU_CORES;
    localparam TSU_MASTERS_PER_WORKER = `VX_CFG_TSU_MASTERS_PER_WORKER;
    // Master select inside a worker: byte-address bits [SEL_LSB +: log2(M)],
    // expressed in granule-address units.
    localparam TSU_SEL_LSB_GRANULE = `VX_CFG_TSU_SEL_LSB - `CLOG2(TSU_BURST_BYTES);
    /* verilator lint_on UNUSEDPARAM */

    // DXA request data — core → DXA control path.
    typedef struct packed {
        logic [NC_WIDTH-1:0]      core_id;
        logic [UUID_WIDTH-1:0]    uuid;
        logic [NW_WIDTH-1:0]      wid;
        logic [DXA_SMEM_ADDR_W-1:0]      smem_addr;   // from lane 0 rs1; LMEM byte address
        logic [31:0]                     meta;        // from lane 1 rs1 (desc[3:0], bar[30:4], 1[31]); 32-bit ABI word
        logic [4:0][31:0]                coords;      // [0]=lane2.rs1,[1]=lane3.rs1,[2]=lane0.rs2,[3]=lane1.rs2,[4]=lane2.rs2; element indices, 32-bit ABI
        logic [`VX_CFG_NUM_WARPS-1:0]    cta_mask;     // from rs2 lane 3
    } dxa_req_data_t;

    // Descriptor table read output — all fields for one slot.
    typedef struct packed {
        logic [`VX_CFG_MEM_ADDR_WIDTH-1:0] base_addr;
        logic [31:0] meta;
        logic [31:0] tile01;
        logic [31:0] tile23;
        logic [31:0] tile4;
        logic [31:0] cfill;
        logic [31:0] size0;
        logic [31:0] size1;
        logic [31:0] size2;
        logic [31:0] size3;
        logic [31:0] size4;
        logic [31:0] stride0;
        logic [31:0] stride1;
        logic [31:0] stride2;
        logic [31:0] stride3;
        logic [31:0] smem_stride;
        logic [31:0] estride2;    // WGMMA tile geometry (tcN) for Flat/BlockMajor
    } dxa_desc_t;

    // Destination SMEM layout (2-bit LAYOUT meta field). Mirrors dxa.h::Layout
    // and sim/simx/dxa/dxa_core.cpp::DestLayout.
    localparam DXA_DEST_ROWMAJOR   = 2'd0;
    localparam DXA_DEST_KMAJOR     = 2'd1;
    localparam DXA_DEST_FLAT       = 2'd2;  // sparse B candidate-pair (b_sp_flat_idx)
    localparam DXA_DEST_BLOCKMAJOR = 2'd3;  // dense B block-major (b_blockmajor_idx)

    // ── Line-granularity types ──────────────────────────────────────────

    // Maximum outer dimensions for tile iteration (dims 1..4 for up to 5D).
    localparam DXA_MAX_OUTER_DIMS = 4;

    // Setup parameters: precomputed constants for addr_gen, rd_ctrl, cl2smem, wr_ctrl.
    // All multiplies happen during setup; fast path uses additions only.
    typedef struct packed {
        logic [`VX_CFG_MEM_ADDR_WIDTH-1:0]  initial_gmem_base;
        logic [DXA_SMEM_ADDR_W-1:0]         initial_smem_base;
        logic [31:0]                row_len_bytes;
        // Rolling-cursor deltas applied at each outer-dim step:
        //   delta[0]: dim-0 step                = stride[0]
        //   delta[d>0]: dim (d-1)→d wrap event  = stride[d] - (tile[d-1]-1)*stride[d-1]
        logic [DXA_MAX_OUTER_DIMS-1:0][31:0] delta;
        logic [DXA_MAX_OUTER_DIMS-1:0][31:0] dim_tiles;  // per-dim tile limits (tile1..tile4)
        logic [DXA_MAX_OUTER_DIMS-1:0][31:0] oob_limit;
        logic [31:0]                cfill;
        // K-major destination layout. When set, smem_wr
        // scatters one element per beat at +per_lane_stride_bytes per element,
        // producing smem[i0 * tile1 + i1 * ... ] instead of the default
        // row-major smem[i1 * tile0 + i0 * ...] layout. rank ≤ 2 only.
        logic                       dest_kmajor;
        logic [15:0]                per_lane_stride_bytes;  // = tile1 * elem_bytes
        logic [3:0]                 elem_bytes;             // 1, 2, 4, or 8
        // Tiled (Flat/BlockMajor) scatter: the per-element SMEM destination is
        // the bbuf-native index (vx_tensor.h::b_sp_flat_idx / b_blockmajor_idx).
        // All tile divisors are powers of two → log shift amounts. Derived once
        // in setup from the tile geometry (estride2) and elem_bytes.
        logic [1:0]                 dest_mode;   // DXA_DEST_*
        logic [3:0]                 lg_ratio;    // log2(32-bit-word / elem) = 2 - log2(elem_bytes)
        logic [3:0]                 lg_tcN;      // log2(tcN)  (= log2(tcK))
        logic [3:0]                 lg_nsteps;   // log2(xtileN / tcN)
        logic [3:0]                 lg_bkK;      // log2(dense B block K extent, in elements)
        // Phase-4 W-slot landing (TSU worker only): descriptor DEST_KIND == TILEBUF_B. wl_tag comes from the
        // issue meta's barrier field (bits [11:4]): {lg_kt[3:0], blk[1:0], slot[1:0]}; the transfer is a dense
        // 64-row x 2^lg_kt-byte image landed straight into W slot `slot` of sequencer block `blk`.
        logic                       wl_en;
        logic [7:0]                 wl_tag;
    } dxa_setup_params_t;

    // W-slot landing beat, TSU worker -> socket -> core -> TCU sequencer (phase 4).
    typedef struct packed {
        logic [1:0]   blk;
        logic [1:0]   slot;
        logic [5:0]   row;     // weight row n in the slot (0..63)
        logic [4:0]   seg;     // 64 B segment of the row: k = seg * 64 .. seg * 64 + 63
        logic [511:0] data;
    } dxa_wland_beat_t;
    typedef struct packed {
        logic [1:0]   blk;
        logic [1:0]   slot;
    } dxa_wland_done_t;

    // SMEM byte destination for B element (k = K-row, n = N-col) under the
    // bbuf-native Flat / BlockMajor layouts. Mirrors vx_tensor.h::b_sp_flat_idx
    // and b_blockmajor_idx; tcK == tcN for canonical WGMMA configs. Every
    // divisor is a power of two, so the body is shifts/masks/adds (no DSP).
    function automatic [DXA_SMEM_ADDR_W-1:0] dxa_tiled_dest_byte(
        input logic [1:0]  mode,
        input logic [15:0] k,
        input logic [15:0] n,
        input logic [3:0]  lg_ratio,
        input logic [3:0]  lg_tcN,
        input logic [3:0]  lg_nsteps,
        input logic [3:0]  lg_bkK,
        input logic [3:0]  esize       // log2(elem_bytes)
    );
        logic [15:0] tcN_mask, ratio_mask, ktck_mask, kw_mask;
        logic [15:0] k_word, elem, k_blk, kw_in, n_blk, n_in, r_in;
        logic [31:0] word_off, dest_elem;
        tcN_mask   = (16'd1 << lg_tcN) - 16'd1;
        ratio_mask = (16'd1 << lg_ratio) - 16'd1;
        n_blk      = n >> lg_tcN;
        n_in       = n & tcN_mask;
        if (mode == DXA_DEST_FLAT) begin
            // word_off = (k_blk*n_steps + n_blk)*blk_words + kw_in*tcN + n_in,
            // blk_words = tcN*(tcN*2); dest = word_off*ratio + elem.
            ktck_mask = (16'd1 << (lg_tcN + 4'd1)) - 16'd1;   // (tcN*2) - 1
            k_word    = k >> lg_ratio;
            elem      = k & ratio_mask;
            k_blk     = k_word >> (lg_tcN + 4'd1);
            kw_in     = k_word & ktck_mask;
            word_off  = ((((32'(k_blk) << lg_nsteps) + 32'(n_blk)) << (lg_tcN + lg_tcN + 4'd1))
                         + (32'(kw_in) << lg_tcN) + 32'(n_in));
            dest_elem = (word_off << lg_ratio) + 32'(elem);
        end else begin
            // BlockMajor: dest = (k_blk*n_steps + n_blk)*b_blk_elems + n_in*bkK + r_in,
            // bkK = fedpK*ratio (conveyed by the descriptor), b_blk_elems = bkK*tcN.
            kw_mask   = (16'd1 << lg_bkK) - 16'd1;
            k_blk     = k >> lg_bkK;
            r_in      = k & kw_mask;
            dest_elem = ((((32'(k_blk) << lg_nsteps) + 32'(n_blk)) << (lg_tcN + lg_bkK))
                         + (32'(n_in) << lg_bkK) + 32'(r_in));
        end
        dxa_tiled_dest_byte = DXA_SMEM_ADDR_W'(dest_elem << esize);
    endfunction

    task automatic trace_ex_op(input int level,
                     input [INST_OP_BITS-1:0] op_type,
                     input op_args_t op_args
    );
        `UNUSED_VAR (op_type)
        `UNUSED_VAR (op_args)
        `TRACE(level, ("DXA.ISSUE"))
    endtask

endpackage

`endif // VX_DXA_PKG_VH
