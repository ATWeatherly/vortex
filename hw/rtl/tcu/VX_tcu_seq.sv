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

`include "VX_define.vh"

`ifdef VX_CFG_TCU_SEQ_ENABLE

// TCU-side sequencer (balanced-design spec phase 3, track C). Executes the tgemm ISA
// (bench docs/spec/tgemm_isa.md, normative; SimX golden model sim/simx/tcu/tcu_seq.h):
//   tgemm.i8 / tacc.st / twait / tbuf.ldx / tbuf.ldw / tcfg / tfence (CUSTOM0 funct7=5, funct3 0..6).
//
// Structure (per core):
//   front    : round-robin over the issue slices, one command per cycle. tcfg updates the per-warp config;
//              async commands (ldx, ldw, tgemm, tacc.st) are placed into a 2-deep engine queue, counted in the
//              warp's pending count and committed immediately (wb = 0); twait / tfence are parked per warp
//              (the decoder wstalls the warp) and committed + unlocked once the warp's pending count is 0
//              (tfence: and the core's stores have drained).
//   X loader : one per core; fills the 8 shared activation slots through the LSU client port.
//   per block (VX_CFG_NUM_TCU_SEQ_BLOCKS): a W loader (4 weight slots, LSU client port), a MAC engine driving one
//              VX_tcu_mac_block (M8 x N64 x K1), a drain that read-modify-writes the drained beats into the
//              block's 16 int32 accumulator tiles, and a store engine (tacc.st) writing tiles to LMEM over the
//              TCU LMEM DMA port.
//   ordering : per-slot / per-tile issue and done generation counters captured at accept implement the ISA §4
//              interlocks (RAW load->gemm, WAR gemm->load, gemm->tacc.st, tacc.st->gemm on the tile).
//
// Slot storage: 8 banks of 64-bit chunks (8 consecutive k bytes of one row), bank = (row + chunk) mod 8, so
// a 32-byte LSU response (4 chunks of one row, or one chunk of 4 rows for the SIMT8 weight pack) lands in 4
// distinct banks in one cycle, and the MAC fill reads one chunk of every row in 1 (X, 8 rows) or 8 (W, 64
// rows) cycles. Rows >= the loaded row count read as 0.
//
// v1 limits (documented in the bench perf_log entry): n_mode 0 (M8/N64) only; notify (barrier transaction) is
// not implemented (asserted); tfence (b) waits for the core-wide store drain (LSU schedulers + mem unit empty),
// which covers the LSU/L1 path used by tbuf.ld* here but NOT the phase-1/4 stream ports.

module VX_tcu_seq import VX_gpu_pkg::*, VX_tcu_pkg::*; #(
    parameter `STRING INSTANCE_ID = ""
) (
    input wire              clk,
    input wire              reset,

    // sequencer instructions, per issue slice
    input  wire [`VX_CFG_ISSUE_WIDTH-1:0] cmd_valid,
    input  tcu_execute_t                  cmd_data [`VX_CFG_ISSUE_WIDTH],
    output wire [`VX_CFG_ISSUE_WIDTH-1:0] cmd_ready,

    // commit packets, per issue slice
    output wire [`VX_CFG_ISSUE_WIDTH-1:0] res_valid,
    output tcu_result_t                   res_data [`VX_CFG_ISSUE_WIDTH],
    input  wire [`VX_CFG_ISSUE_WIDTH-1:0] res_ready,

    // twait / tfence release
    output wire                     unlock_valid,
    output wire [NW_WIDTH-1:0]      unlock_wid,

    // core store drain (tfence)
    input wire                      store_drained,

`ifdef VX_CFG_TSU_ENABLE
    // phase-4 W-slot landing from the TSU (one aligned 64 B row segment per beat; done = transfer landed)
    input  wire [VX_dxa_pkg::TSU_WL_LANES-1:0] wl_valid,
    output wire [VX_dxa_pkg::TSU_WL_LANES-1:0] wl_ready,
    input  VX_dxa_pkg::dxa_wland_beat_t [VX_dxa_pkg::TSU_WL_LANES-1:0] wl_beat,
    input  wire                         wl_done_valid,
    input  VX_dxa_pkg::dxa_wland_done_t wl_done,
`endif

    // slot loads (LSU client port)
    VX_lsu_sched_if.master          mem_if,

    // tacc.st (LMEM DMA port)
    VX_mem_bus_if.master            lmem_if
);
    `UNUSED_SPARAM (INSTANCE_ID)

    localparam NS      = `VX_CFG_ISSUE_WIDTH;
    localparam NB      = `VX_CFG_NUM_TCU_SEQ_BLOCKS;
    localparam NW      = `VX_CFG_NUM_WARPS;
    localparam NT      = `VX_CFG_NUM_TCU_LANES;
    localparam NL      = `VX_CFG_NUM_LSU_LANES;
`ifdef VX_CFG_TCU_SEQ_KT_MAX
    localparam KT_MAX  = `VX_CFG_TCU_SEQ_KT_MAX;
`else
    localparam KT_MAX  = 2048;
`endif
    localparam NXS     = 8;              // X slots per core
    localparam NWS     = 4;              // W slots per block
    localparam NTILE   = 16;             // acc tiles per block
    localparam XROWS   = 8;
    localparam WROWS   = 64;
    localparam CX      = KT_MAX / 64;    // bank entries per row (8 banks x 8 bytes per entry)
    localparam XD      = NXS * XROWS * CX;
    localparam WD      = NWS * WROWS * CX;
    localparam XA_W    = `CLOG2(XD);
    localparam WA_W    = `CLOG2(WD);
    localparam KB_W    = `CLOG2(KT_MAX + 1);     // byte count / offset within a row
    localparam KW_W    = `CLOG2(KT_MAX / 4);     // 32-bit word index within a row
    localparam CH_W    = `CLOG2(KT_MAX / 8);     // chunk index within a row
    localparam NLD     = NB + 1;                 // loaders: 0 = X, 1..NB = W of block b-1
    localparam LID_W   = `CLOG2(NLD);
    localparam GEN_W   = 8;
    localparam ACC_D   = NTILE * 32;             // 32 drained beats per tile
    localparam ACC_W   = 2 * 8 * 32;             // one beat: 2 columns x 8 rows x int32
    localparam QD      = 2;                      // engine queue depth (ISA §4)
    localparam LMEM_ROW_W = `VX_CFG_LMEM_LOG_SIZE - 5;
    localparam DMA_TAG_VW = TCU_LMEM_WG_TAG_W - UUID_WIDTH;
    localparam BK_W    = `UP(`CLOG2(NB));

    `STATIC_ASSERT ((KT_MAX % 64) == 0, ("KT_MAX must be a multiple of 64"))
    `STATIC_ASSERT (NL == 8, ("VX_tcu_seq assumes 8 LSU lanes (32-byte requests)"))
    `STATIC_ASSERT (LSU_WORD_SIZE == 4, ("VX_tcu_seq assumes XLEN 32"))
    `STATIC_ASSERT (`VX_CFG_LMEM_NUM_BANKS == 8, ("VX_tcu_seq assumes a 32-byte LMEM DMA row"))
    `STATIC_ASSERT (LSU_CLIENT_TAG_WIDTH >= (LID_W + 8 + KW_W), ("LSU client tag too narrow for the sequencer"))
    `STATIC_ASSERT (DMA_TAG_VW >= BK_W, ("TCU LMEM tag too narrow for the sequencer store engines"))
    `STATIC_ASSERT (NB <= 4, ("acc_tile[5:4] addresses at most 4 blocks"))

    // ===================================================================================================
    // Front: slice arbitration
    // ===================================================================================================
    localparam NS_W = `UP(`CLOG2(NS));
    reg  [NS_W-1:0] rr_slice_r;
    wire [NS_W-1:0] fsel;
    wire            fvalid;
    begin : g_front_pick
        logic [NS_W-1:0] sel;
        logic            any;
        always_comb begin
            sel = rr_slice_r;
            any = 1'b0;
            for (integer i = 0; i < NS; ++i) begin
                automatic logic [NS_W-1:0] s = NS_W'((32'(rr_slice_r) + i) % NS);
                if (!any && cmd_valid[s]) begin
                    sel = s;
                    any = 1'b1;
                end
            end
        end
        assign fsel   = sel;
        assign fvalid = any;
    end

    tcu_execute_t fcmd;
    assign fcmd = cmd_data[fsel];
    `UNUSED_VAR (fcmd)

    // first active lane carries the warp-uniform scalar operands
    logic [`CLOG2(NT)-1:0] flane;
    always_comb begin
        flane = '0;
        for (integer i = NT-1; i >= 0; --i) begin
            if (fcmd.header.tmask[i]) flane = `CLOG2(NT)'(i);
        end
    end
    wire [31:0] f_rs1 = 32'(fcmd.rs1_data[flane]);
    wire [31:0] f_rs2 = 32'(fcmd.rs2_data[flane]);
    wire [2:0]  f_op  = fcmd.op_args.tcu.fmt_s[2:0];
    wire [NW_WIDTH-1:0] f_wid = fcmd.header.wid;

    localparam OP_GEMM = 3'd0, OP_ACCST = 3'd1, OP_WAIT = 3'd2, OP_LDX = 3'd3, OP_LDW = 3'd4, OP_CFG = 3'd5, OP_FENCE = 3'd6;
    localparam OP_LDWS = 3'd7;   // tbuf.ldws: expect one TSU stream landing into a W slot (phase 4, TSU builds)

    // ===================================================================================================
    // Per-warp configuration (tcfg)
    // ===================================================================================================
    reg [31:0] cfg_r [NW][8];
    localparam CFG_KT = 0, CFG_XROWS = 1, CFG_XSTRIDE = 2, CFG_WROWS = 3, CFG_WLAYOUT = 4, CFG_WSTRIDE = 5,
               CFG_ACCSTRIDE = 6, CFG_NOTIFY = 7;
    wire [31:0] fc_kt      = cfg_r[f_wid][CFG_KT];
    wire [31:0] fc_xrows   = cfg_r[f_wid][CFG_XROWS];
    wire [31:0] fc_xstride = cfg_r[f_wid][CFG_XSTRIDE];
    wire [31:0] fc_wrows   = cfg_r[f_wid][CFG_WROWS];
    wire [31:0] fc_wlayout = cfg_r[f_wid][CFG_WLAYOUT];
    wire [31:0] fc_wstride = cfg_r[f_wid][CFG_WSTRIDE];
    wire [31:0] fc_accstr  = cfg_r[f_wid][CFG_ACCSTRIDE];
    `UNUSED_VAR (fc_wlayout[31:1])

    // ===================================================================================================
    // Generation counters (ISA §4 interlocks)
    // ===================================================================================================
    reg [GEN_W-1:0] x_ld_iss [NXS], x_ld_done [NXS], x_rd_iss [NXS], x_rd_done [NXS];
    reg [GEN_W-1:0] w_ld_iss [NB][NWS], w_ld_done [NB][NWS], w_rd_iss [NB][NWS], w_rd_done [NB][NWS];
    reg [GEN_W-1:0] a_gm_iss [NB][NTILE], a_gm_done [NB][NTILE], a_st_iss [NB][NTILE], a_st_done [NB][NTILE];

    function automatic logic gen_ok(input logic [GEN_W-1:0] done, input logic [GEN_W-1:0] cap);
        logic [GEN_W-1:0] d;
        d = done - cap;
        gen_ok = ~d[GEN_W-1];
    endfunction

    // ===================================================================================================
    // Engine queue entries
    // ===================================================================================================
    typedef struct packed {
        logic [NW_WIDTH-1:0]   wid;
        logic [31:0]           src;
        logic [2:0]            slot;
        logic [3:0]            rows;      // 1..8
        logic [KB_W-1:0]       kt;
        logic [31:0]           stride;
        logic [GEN_W-1:0]      cap;
    } ldx_cmd_t;

    typedef struct packed {
        logic [NW_WIDTH-1:0]   wid;
        logic [31:0]           src;
        logic [1:0]            slot;
        logic [6:0]            rows;      // 1..64
        logic [KB_W-1:0]       kt;
        logic                  layout;
        logic [31:0]           stride;
        logic [GEN_W-1:0]      cap;
    } ldw_cmd_t;

    typedef struct packed {
        logic [NW_WIDTH-1:0]   wid;
        logic [KB_W-1:0]       k;
        logic [KB_W-1:0]       koff;
        logic [2:0]            xs;
        logic [1:0]            ws;
        logic [3:0]            tile;
        logic                  accum;
        logic [GEN_W-1:0]      capx;
        logic [GEN_W-1:0]      capw;
        logic [GEN_W-1:0]      capa;
    } gemm_cmd_t;

    typedef struct packed {
        logic [NW_WIDTH-1:0]   wid;
        logic [31:0]           dst;
        logic [31:0]           stride;
        logic [3:0]            tile;
        logic [GEN_W-1:0]      cap;
    } st_cmd_t;

    // ----- queues (2-deep) -----
    ldx_cmd_t  ldx_q [QD];   reg [1:0] ldx_cnt;   reg ldx_wp, ldx_rp;
    ldw_cmd_t  ldw_q [NB][QD]; reg [1:0] ldw_cnt [NB]; reg ldw_wp [NB], ldw_rp [NB];
    gemm_cmd_t gm_q  [NB][QD]; reg [1:0] gm_cnt  [NB]; reg gm_wp  [NB], gm_rp  [NB];
    st_cmd_t   st_q  [NB][QD]; reg [1:0] st_cnt  [NB]; reg st_wp  [NB], st_rp  [NB];

    wire        ldx_pop;
    wire [NB-1:0] ldw_pop, gm_pop, st_pop;

    // ----- stream-landing expectations (tbuf.ldws), per W slot, 2-deep, in program order -----
    typedef struct packed {
        logic [NW_WIDTH-1:0]   wid;
        logic [GEN_W-1:0]      cap;      // WAR: landing may write once w_rd_done reaches cap
    } lws_ent_t;
    /* verilator lint_off UNUSEDSIGNAL */   // read only by the TSU landing path (VX_CFG_TSU_ENABLE)
    lws_ent_t  lws_q [NB][NWS][QD];
    /* verilator lint_on UNUSEDSIGNAL */
    reg [1:0]  lws_cnt [NB][NWS];
    reg        lws_wp [NB][NWS], lws_rp [NB][NWS];
    // landing completion (one transfer = one expectation)
    wire                 lws_done_ev;
    wire [1:0]           lws_done_blk, lws_done_slot;
    lws_ent_t            lws_done_head;
    `UNUSED_VAR (lws_done_head.cap)

    // ===================================================================================================
    // Front decode / accept
    // ===================================================================================================
    wire [1:0] f_ldw_blk = f_rs2[3:2];
    wire [1:0] f_gm_blk  = f_rs2[5:4];
    wire [1:0] f_st_blk  = f_rs2[5:4];

    // result FIFO (one entry per cycle at most)
    localparam RQ = 4;
    typedef struct packed {
        logic [NS_W-1:0]  slice;
        tcu_header_t      hdr;
    } res_ent_t;
    res_ent_t res_q [RQ];
    reg [`CLOG2(RQ+1)-1:0] res_cnt;
    reg [`CLOG2(RQ)-1:0]   res_wp, res_rp;
    wire res_full = (res_cnt == (`CLOG2(RQ+1))'(RQ));

    // parked twait / tfence per warp
    reg [NW-1:0]  wait_v, wait_fence;
    tcu_header_t  wait_hdr [NW];
    reg [15:0]    pending [NW];

    // wait release (one per cycle)
    logic                 wrel_v;
    logic [NW_WIDTH-1:0]  wrel_w;
    always_comb begin
        wrel_v = 1'b0;
        wrel_w = '0;
        for (integer w = NW-1; w >= 0; --w) begin
            if (wait_v[w] && (pending[w] == 16'd0) && (~wait_fence[w] || store_drained)) begin
                wrel_v = 1'b1;
                wrel_w = NW_WIDTH'(w);
            end
        end
    end
    wire wrel_fire = wrel_v && ~res_full;

    logic f_can;
    always_comb begin
        case (f_op)
        OP_LDX:   f_can = (ldx_cnt != 2'(QD));
        OP_LDW:   f_can = (ldw_cnt[f_ldw_blk] != 2'(QD)) && (32'(f_ldw_blk) < NB);
        OP_GEMM:  f_can = (gm_cnt[f_gm_blk] != 2'(QD)) && (32'(f_gm_blk) < NB);
        OP_ACCST: f_can = (st_cnt[f_st_blk] != 2'(QD)) && (32'(f_st_blk) < NB);
        OP_LDWS:  f_can = (32'(f_ldw_blk) < NB) && (lws_cnt[f_ldw_blk][f_rs2[1:0]] != 2'(QD));
        default:  f_can = 1'b1;
        endcase
    end
    wire f_parks  = (f_op == OP_WAIT) || (f_op == OP_FENCE);
    wire f_fire   = fvalid && f_can && ~wrel_fire && (f_parks || ~res_full);
    wire f_async  = (f_op == OP_LDX) || (f_op == OP_LDW) || (f_op == OP_GEMM) || (f_op == OP_ACCST) || (f_op == OP_LDWS);

    for (genvar s = 0; s < NS; ++s) begin : g_cmd_ready
        assign cmd_ready[s] = f_fire && (fsel == NS_W'(s));
    end

    // gemm control word fields
    wire [5:0]  gc_tile_full = f_rs2[5:0];
    wire [2:0]  gc_xs    = f_rs2[8:6];
    wire [3:0]  gc_ws    = f_rs2[12:9];
    wire [1:0]  gc_mode  = f_rs2[14:13];
    wire        gc_acc   = f_rs2[15];
    wire        gc_ntfy  = f_rs2[16];
    wire [10:0] gc_koff  = f_rs2[27:17];
    wire [1:0]  sc_mode  = f_rs2[7:6];
    wire        sc_ntfy  = f_rs2[8];
    `UNUSED_VAR ({f_rs2[31:28], gc_tile_full})
    `RUNTIME_ASSERT(~(f_fire && (f_op == OP_GEMM)) || ((gc_mode == 2'd0) && (gc_ws[3:2] == f_gm_blk) && ~gc_ntfy),
        ("%t: *** %s tgemm: unsupported control 0x%0h (v1: n_mode 0, notify 0, w_slot block == acc block)", $time, INSTANCE_ID, f_rs2))
    `RUNTIME_ASSERT(~(f_fire && (f_op == OP_ACCST)) || ((sc_mode == 2'd0) && ~sc_ntfy),
        ("%t: *** %s tacc.st: unsupported control 0x%0h (v1: n_mode 0, notify 0)", $time, INSTANCE_ID, f_rs2))
    `RUNTIME_ASSERT(~(f_fire && (f_op == OP_CFG)) || (f_rs1 < 32'd8),
        ("%t: *** %s tcfg: bad field %0d", $time, INSTANCE_ID, f_rs1))
`ifndef VX_CFG_TSU_ENABLE
    `RUNTIME_ASSERT(~(f_fire && (f_op == OP_LDWS)),
        ("%t: *** %s tbuf.ldws needs a TSU build (VX_CFG_TSU_ENABLE)", $time, INSTANCE_ID))
`endif

    // ===================================================================================================
    // Completion events (decrement pending, bump done counters)
    // ===================================================================================================
    wire                  ldx_done_ev;
    wire [NW_WIDTH-1:0]   ldx_done_wid;
    wire [2:0]            ldx_done_slot;
    wire [NB-1:0]         ldw_done_ev;
    wire [NW_WIDTH-1:0]   ldw_done_wid [NB];
    wire [1:0]            ldw_done_slot [NB];
    wire [NB-1:0]         gm_done_ev;
    wire [NW_WIDTH-1:0]   gm_done_wid [NB];
    wire [2:0]            gm_done_xs [NB];
    wire [1:0]            gm_done_ws [NB];
    wire [3:0]            gm_done_tile [NB];
    wire [NB-1:0]         st_done_ev;
    wire [NW_WIDTH-1:0]   st_done_wid [NB];
    wire [3:0]            st_done_tile [NB];

    // ===================================================================================================
    // Front state update
    // ===================================================================================================
    always_ff @(posedge clk) begin
        if (reset) begin
            rr_slice_r <= '0;
            for (integer w = 0; w < NW; ++w) begin
                cfg_r[w][CFG_KT]        <= 32'd2048;
                cfg_r[w][CFG_XROWS]     <= 32'd8;
                cfg_r[w][CFG_XSTRIDE]   <= 32'd2048;
                cfg_r[w][CFG_WROWS]     <= 32'd64;
                cfg_r[w][CFG_WLAYOUT]   <= 32'd0;
                cfg_r[w][CFG_WSTRIDE]   <= 32'd2048;
                cfg_r[w][CFG_ACCSTRIDE] <= 32'd0;
                cfg_r[w][CFG_NOTIFY]    <= 32'd0;
            end
            wait_v     <= '0;
            wait_fence <= '0;
        end else begin
            if (f_fire) begin
                rr_slice_r <= NS_W'((32'(fsel) + 1) % NS);
            end
            if (f_fire && (f_op == OP_CFG)) begin
                cfg_r[f_wid][f_rs1[2:0]] <= f_rs2;
            end
            if (f_fire && f_parks) begin
                wait_v[f_wid]     <= 1'b1;
                wait_fence[f_wid] <= (f_op == OP_FENCE);
                wait_hdr[f_wid]   <= fcmd.header;
            end
            if (wrel_fire) begin
                wait_v[wrel_w] <= 1'b0;
            end
        end
    end

    // pending counts
    always_ff @(posedge clk) begin
        if (reset) begin
            for (integer w = 0; w < NW; ++w) pending[w] <= '0;
        end else begin
            for (integer w = 0; w < NW; ++w) begin
                automatic logic [15:0] inc = 16'(f_fire && f_async && (f_wid == NW_WIDTH'(w)));
                automatic logic [15:0] dec = 16'(ldx_done_ev && (ldx_done_wid == NW_WIDTH'(w)))
                                           + 16'(lws_done_ev && (lws_done_head.wid == NW_WIDTH'(w)));
                for (integer b = 0; b < NB; ++b) begin
                    dec = dec + 16'(ldw_done_ev[b] && (ldw_done_wid[b] == NW_WIDTH'(w)))
                              + 16'(gm_done_ev[b]  && (gm_done_wid[b]  == NW_WIDTH'(w)))
                              + 16'(st_done_ev[b]  && (st_done_wid[b]  == NW_WIDTH'(w)));
                end
                pending[w] <= pending[w] + inc - dec;
            end
        end
    end

    // generation counters
    always_ff @(posedge clk) begin
        if (reset) begin
            for (integer i = 0; i < NXS; ++i) begin
                x_ld_iss[i] <= '0; x_ld_done[i] <= '0; x_rd_iss[i] <= '0; x_rd_done[i] <= '0;
            end
            for (integer b = 0; b < NB; ++b) begin
                for (integer i = 0; i < NWS; ++i) begin
                    w_ld_iss[b][i] <= '0; w_ld_done[b][i] <= '0; w_rd_iss[b][i] <= '0; w_rd_done[b][i] <= '0;
                end
                for (integer i = 0; i < NTILE; ++i) begin
                    a_gm_iss[b][i] <= '0; a_gm_done[b][i] <= '0; a_st_iss[b][i] <= '0; a_st_done[b][i] <= '0;
                end
            end
        end else begin
            // issue side (front)
            if (f_fire && (f_op == OP_LDX)) x_ld_iss[f_rs2[2:0]] <= x_ld_iss[f_rs2[2:0]] + GEN_W'(1);
            if (f_fire && ((f_op == OP_LDW) || (f_op == OP_LDWS))) w_ld_iss[f_ldw_blk][f_rs2[1:0]] <= w_ld_iss[f_ldw_blk][f_rs2[1:0]] + GEN_W'(1);
            if (f_fire && (f_op == OP_GEMM)) begin
                x_rd_iss[gc_xs] <= x_rd_iss[gc_xs] + GEN_W'(1);
                w_rd_iss[f_gm_blk][gc_ws[1:0]] <= w_rd_iss[f_gm_blk][gc_ws[1:0]] + GEN_W'(1);
                a_gm_iss[f_gm_blk][f_rs2[3:0]] <= a_gm_iss[f_gm_blk][f_rs2[3:0]] + GEN_W'(1);
            end
            if (f_fire && (f_op == OP_ACCST)) a_st_iss[f_st_blk][f_rs2[3:0]] <= a_st_iss[f_st_blk][f_rs2[3:0]] + GEN_W'(1);
            // done side (engines)
            if (ldx_done_ev) x_ld_done[ldx_done_slot] <= x_ld_done[ldx_done_slot] + GEN_W'(1);
            for (integer i = 0; i < NXS; ++i) begin
                automatic logic [GEN_W-1:0] n = '0;
                for (integer b = 0; b < NB; ++b) n = n + GEN_W'(gm_done_ev[b] && (gm_done_xs[b] == 3'(i)));
                x_rd_done[i] <= x_rd_done[i] + n;
            end
            for (integer b = 0; b < NB; ++b) begin
                for (integer i = 0; i < NWS; ++i) begin
                    w_ld_done[b][i] <= w_ld_done[b][i]
                        + GEN_W'(ldw_done_ev[b] && (ldw_done_slot[b] == 2'(i)))
                        + GEN_W'(lws_done_ev && (lws_done_blk == 2'(b)) && (lws_done_slot == 2'(i)));
                end
                if (gm_done_ev[b]) begin
                    w_rd_done[b][gm_done_ws[b]]   <= w_rd_done[b][gm_done_ws[b]] + GEN_W'(1);
                    a_gm_done[b][gm_done_tile[b]] <= a_gm_done[b][gm_done_tile[b]] + GEN_W'(1);
                end
                if (st_done_ev[b]) a_st_done[b][st_done_tile[b]] <= a_st_done[b][st_done_tile[b]] + GEN_W'(1);
            end
        end
    end

    // ===================================================================================================
    // Queues
    // ===================================================================================================
    ldx_cmd_t f_ldx;
    always_comb begin
        f_ldx.wid    = f_wid;
        f_ldx.src    = f_rs1;
        f_ldx.slot   = f_rs2[2:0];
        f_ldx.rows   = 4'(fc_xrows);
        f_ldx.kt     = KB_W'(fc_kt);
        f_ldx.stride = fc_xstride;
        f_ldx.cap    = x_rd_iss[f_rs2[2:0]];
    end
    ldw_cmd_t f_ldw;
    always_comb begin
        f_ldw.wid    = f_wid;
        f_ldw.src    = f_rs1;
        f_ldw.slot   = f_rs2[1:0];
        f_ldw.rows   = 7'(fc_wrows);
        f_ldw.kt     = KB_W'(fc_kt);
        f_ldw.layout = fc_wlayout[0];
        f_ldw.stride = fc_wstride;
        f_ldw.cap    = w_rd_iss[f_ldw_blk][f_rs2[1:0]];
    end
    gemm_cmd_t f_gm;
    always_comb begin
        f_gm.wid   = f_wid;
        f_gm.k     = KB_W'(f_rs1);
        f_gm.koff  = KB_W'({gc_koff, 4'b0});
        f_gm.xs    = gc_xs;
        f_gm.ws    = gc_ws[1:0];
        f_gm.tile  = f_rs2[3:0];
        f_gm.accum = gc_acc;
        f_gm.capx  = x_ld_iss[gc_xs];
        f_gm.capw  = w_ld_iss[f_gm_blk][gc_ws[1:0]];
        f_gm.capa  = a_st_iss[f_gm_blk][f_rs2[3:0]];
    end
    st_cmd_t f_st;
    always_comb begin
        f_st.wid    = f_wid;
        f_st.dst    = f_rs1;
        f_st.stride = (fc_accstr == 32'd0) ? 32'd256 : fc_accstr;
        f_st.tile   = f_rs2[3:0];
        f_st.cap    = a_gm_iss[f_st_blk][f_rs2[3:0]];
    end
    `RUNTIME_ASSERT(~(f_fire && (f_op == OP_LDX)) || ((fc_kt[3:0] == 4'd0) && (fc_kt <= 32'(KT_MAX)) && (fc_xrows >= 32'd1) && (fc_xrows <= 32'd8) && (f_rs1[1:0] == 2'd0) && (fc_xstride[1:0] == 2'd0)),
        ("%t: *** %s tbuf.ldx: illegal (src=0x%0h kt=%0d rows=%0d stride=%0d)", $time, INSTANCE_ID, f_rs1, fc_kt, fc_xrows, fc_xstride))
    `RUNTIME_ASSERT(~(f_fire && (f_op == OP_LDW)) || ((fc_kt[3:0] == 4'd0) && (fc_kt <= 32'(KT_MAX)) && (fc_wrows >= 32'd1) && (fc_wrows <= 32'd64) && (f_rs1[1:0] == 2'd0) && (fc_wstride[1:0] == 2'd0)),
        ("%t: *** %s tbuf.ldw: illegal (src=0x%0h kt=%0d rows=%0d stride=%0d; v1: rows <= 64)", $time, INSTANCE_ID, f_rs1, fc_kt, fc_wrows, fc_wstride))
    `RUNTIME_ASSERT(~(f_fire && (f_op == OP_ACCST)) || ((f_rs1[1:0] == 2'd0) && (fc_accstr[1:0] == 2'd0)),
        ("%t: *** %s tacc.st: unaligned dst 0x%0h / stride %0d", $time, INSTANCE_ID, f_rs1, fc_accstr))

    always_ff @(posedge clk) begin
        if (reset) begin
            ldx_cnt <= '0; ldx_wp <= 1'b0; ldx_rp <= 1'b0;
            for (integer b = 0; b < NB; ++b) begin
                ldw_cnt[b] <= '0; ldw_wp[b] <= 1'b0; ldw_rp[b] <= 1'b0;
                gm_cnt[b]  <= '0; gm_wp[b]  <= 1'b0; gm_rp[b]  <= 1'b0;
                st_cnt[b]  <= '0; st_wp[b]  <= 1'b0; st_rp[b]  <= 1'b0;
            end
        end else begin
            begin
                automatic logic push = f_fire && (f_op == OP_LDX);
                if (push) begin ldx_q[ldx_wp] <= f_ldx; ldx_wp <= ~ldx_wp; end
                if (ldx_pop) ldx_rp <= ~ldx_rp;
                ldx_cnt <= ldx_cnt + 2'(push) - 2'(ldx_pop);
            end
            for (integer b = 0; b < NB; ++b) begin
                automatic logic pw = f_fire && (f_op == OP_LDW)  && (f_ldw_blk == 2'(b));
                automatic logic pg = f_fire && (f_op == OP_GEMM) && (f_gm_blk  == 2'(b));
                automatic logic ps = f_fire && (f_op == OP_ACCST) && (f_st_blk == 2'(b));
                if (pw) begin ldw_q[b][ldw_wp[b]] <= f_ldw; ldw_wp[b] <= ~ldw_wp[b]; end
                if (ldw_pop[b]) ldw_rp[b] <= ~ldw_rp[b];
                ldw_cnt[b] <= ldw_cnt[b] + 2'(pw) - 2'(ldw_pop[b]);
                if (pg) begin gm_q[b][gm_wp[b]] <= f_gm; gm_wp[b] <= ~gm_wp[b]; end
                if (gm_pop[b]) gm_rp[b] <= ~gm_rp[b];
                gm_cnt[b] <= gm_cnt[b] + 2'(pg) - 2'(gm_pop[b]);
                if (ps) begin st_q[b][st_wp[b]] <= f_st; st_wp[b] <= ~st_wp[b]; end
                if (st_pop[b]) st_rp[b] <= ~st_rp[b];
                st_cnt[b] <= st_cnt[b] + 2'(ps) - 2'(st_pop[b]);
            end
        end
    end

    // ===================================================================================================
    // Result FIFO + per-slice output
    // ===================================================================================================
    wire res_push = (f_fire && ~f_parks) || wrel_fire;
    res_ent_t res_in;
    always_comb begin
        res_in = '0;
        if (wrel_fire) begin
            res_in.hdr   = wait_hdr[wrel_w];
            res_in.slice = NS_W'(wid_to_isw(wrel_w));
        end else begin
            res_in.hdr   = fcmd.header;
            res_in.slice = fsel;
        end
        res_in.hdr.wb       = 1'b0;
        res_in.hdr.wr_xregs = '0;
        res_in.hdr.sop      = 1'b1;
        res_in.hdr.eop      = 1'b1;
    end
    wire res_head_v = (res_cnt != 0);
    res_ent_t res_head;
    assign res_head = res_q[res_rp];
    wire res_pop = res_head_v && res_ready[res_head.slice];
    always_ff @(posedge clk) begin
        if (reset) begin
            res_cnt <= '0; res_wp <= '0; res_rp <= '0;
        end else begin
            if (res_push) begin res_q[res_wp] <= res_in; res_wp <= res_wp + 1'b1; end
            if (res_pop) res_rp <= res_rp + 1'b1;
            res_cnt <= res_cnt + (`CLOG2(RQ+1))'(res_push) - (`CLOG2(RQ+1))'(res_pop);
        end
    end
    for (genvar s = 0; s < NS; ++s) begin : g_res
        assign res_valid[s]       = res_head_v && (res_head.slice == NS_W'(s));
        assign res_data[s].header = res_head.hdr;
        assign res_data[s].data   = '0;
    end
    assign unlock_valid = wrel_fire;
    assign unlock_wid   = wrel_w;

    // ===================================================================================================
    // Slot storage: X (per core) and W (per block), 8 banks x 64-bit chunks
    // ===================================================================================================
    // X banks: write port from the LSU response, read port shared by the NB MAC fills (one block per cycle)
    logic [7:0]            xb_we;
    logic [7:0][XA_W-1:0]  xb_wa;
    logic [7:0][63:0]      xb_wd;
    logic [7:0][7:0]       xb_wbe;
    logic                  xb_re;
    logic [7:0][XA_W-1:0]  xb_ra;
    logic [7:0][63:0]      xb_rd;
    for (genvar bk = 0; bk < 8; ++bk) begin : g_xbank
        (* ram_style = "block" *) reg [63:0] mem [XD];
        reg [63:0] rd_r;
        always_ff @(posedge clk) begin
            if (xb_we[bk]) begin
                for (integer j = 0; j < 8; ++j) begin
                    if (xb_wbe[bk][j]) mem[xb_wa[bk]][j*8 +: 8] <= xb_wd[bk][j*8 +: 8];
                end
            end
            if (xb_re) rd_r <= mem[xb_ra[bk]];
        end
        assign xb_rd[bk] = rd_r;
    end

    // ===================================================================================================
    // Phase-4 stream landing (TSU -> W slot): expectation FIFOs and beat acceptance
    // ===================================================================================================
    // A beat for (blk, slot) is accepted when that slot has an outstanding tbuf.ldws whose WAR cap is met
    // (every tgemm issued before it that reads the slot has finished) and the block's W banks are not taking
    // an LSU-loader write this cycle. The transfer's completion pulse retires the expectation exactly like a
    // tbuf.ldw completion (w_ld_done++, the issuing warp's pending count--), so tgemm's RAW interlock and
    // twait need no change.
    wire [NB-1:0] wb_lsu_busy;
`ifdef VX_CFG_TSU_ENABLE
    // One lane per TSU worker (phase-4 feed): lane l only carries blocks b with b mod WLL == l, so the lanes
    // write disjoint blocks and are accepted independently in the same cycle.
    localparam WLL = VX_dxa_pkg::TSU_WL_LANES;
    wire [WLL-1:0][1:0] wl_blk;
    wire [WLL-1:0][1:0] wl_slot;
    wire [WLL-1:0]      wl_fire;
    for (genvar l = 0; l < WLL; ++l) begin : g_wl_lane
        lws_ent_t wl_head;
        assign wl_blk[l]  = wl_beat[l].blk;
        assign wl_slot[l] = wl_beat[l].slot;
        assign wl_head    = lws_q[wl_blk[l]][wl_slot[l]][lws_rp[wl_blk[l]][wl_slot[l]]];
        assign wl_ready[l] = (32'(wl_blk[l]) < NB) && (lws_cnt[wl_blk[l]][wl_slot[l]] != 2'd0)
                          && gen_ok(w_rd_done[wl_blk[l]][wl_slot[l]], wl_head.cap) && ~wb_lsu_busy[wl_blk[l]];
        assign wl_fire[l] = wl_valid[l] && wl_ready[l];
        `UNUSED_VAR (wl_head.wid)
        `RUNTIME_ASSERT(~wl_valid[l] || ((32'(wl_blk[l]) % WLL) == l),
            ("%t: *** %s landing lane %0d carries block %0d (lanes own blocks b mod %0d)", $time, INSTANCE_ID, l, wl_blk[l], WLL))
        `RUNTIME_ASSERT(~wl_fire[l] || ((wl_beat[l].simt ? (32'(wl_beat[l].seg) >> 3) : 32'(wl_beat[l].seg)) < CX),
            ("%t: *** %s stream landing segment %0d beyond KT_MAX", $time, INSTANCE_ID, wl_beat[l].seg))
    end
    assign lws_done_ev   = wl_done_valid;
    assign lws_done_blk  = wl_done.blk;
    assign lws_done_slot = wl_done.slot;
    assign lws_done_head = lws_q[wl_done.blk][wl_done.slot][lws_rp[wl_done.blk][wl_done.slot]];
    `RUNTIME_ASSERT(~wl_done_valid || (lws_cnt[wl_done.blk][wl_done.slot] != 2'd0),
        ("%t: *** %s stream landing done without a tbuf.ldws (blk=%0d slot=%0d)", $time, INSTANCE_ID, wl_done.blk, wl_done.slot))
`else
    wire       wl_fire = 1'b0;
    `UNUSED_VAR (wl_fire)
    `UNUSED_VAR (wb_lsu_busy)
    assign lws_done_ev   = 1'b0;
    assign lws_done_blk  = '0;
    assign lws_done_slot = '0;
    assign lws_done_head = '0;
`endif
    always_ff @(posedge clk) begin
        if (reset) begin
            for (integer b = 0; b < NB; ++b) for (integer i = 0; i < NWS; ++i) begin
                lws_cnt[b][i] <= '0; lws_wp[b][i] <= 1'b0; lws_rp[b][i] <= 1'b0;
            end
        end else begin
            for (integer b = 0; b < NB; ++b) for (integer i = 0; i < NWS; ++i) begin
                automatic logic push = f_fire && (f_op == OP_LDWS) && (f_ldw_blk == 2'(b)) && (f_rs2[1:0] == 2'(i));
                automatic logic pop  = lws_done_ev && (lws_done_blk == 2'(b)) && (lws_done_slot == 2'(i));
                if (push) begin
                    lws_q[b][i][lws_wp[b][i]] <= '{wid: f_wid, cap: w_rd_iss[b][i]};
                    lws_wp[b][i] <= ~lws_wp[b][i];
                end
                if (pop) lws_rp[b][i] <= ~lws_rp[b][i];
                lws_cnt[b][i] <= lws_cnt[b][i] + 2'(push) - 2'(pop);
            end
        end
    end

    // ---------------------------------------------------------------------------------------------------
    // Landed watermark (G4 feed): a tgemm may start on a W slot whose stream landing is still in flight and
    // consume it k-group by k-group as it lands. A k-group is 64 k bytes of all 64 rows = 64 beats in both
    // landing layouts (SIMT8: 8 k-chunks x 8 row-octets; row-major: one 64 B segment of each of the 64 rows).
    // Per block, one tracker follows the landing currently writing the block (one TSU worker per block, one
    // transfer at a time): a beat counter per k-group (beats may arrive in any order: masters interleave
    // bursts) and wm = the number of leading complete groups. The tracker is cleared by the landing's done.
    // A beat for a different slot while the tracker is busy is landed but not tracked (its tgemm falls back
    // to the whole-transfer wait), so the watermark only ever claims groups whose 64 beats were all written.
    // ---------------------------------------------------------------------------------------------------
    localparam LG_N   = KT_MAX / 64;              // k-groups per slot row
    localparam LWM_W  = `CLOG2(LG_N + 1);
    wire [NB-1:0]              trk_v;
    wire [1:0]                 trk_slot [NB];
    wire [LWM_W-1:0]           trk_wm   [NB];
    wire [NB-1:0]              lws_beat_ev;       // a beat landed in block b this cycle
    wire [1:0]                 lws_beat_slot [NB];
`ifdef VX_CFG_TSU_ENABLE
`ifndef VX_CFG_TCU_SEQ_NO_LAND_WM
    localparam LG_W   = `UP(`CLOG2(LG_N));
    for (genvar b = 0; b < NB; ++b) begin : g_trk
        localparam LN = b % WLL;
        wire        bt_fire = wl_fire[LN] && (wl_blk[LN] == 2'(b));
        wire [1:0]  bt_slot = wl_slot[LN];
        wire [LG_W-1:0] bt_grp = wl_beat[LN].simt ? LG_W'(32'(wl_beat[LN].seg) >> 3) : LG_W'(wl_beat[LN].seg);
        wire        dn = lws_done_ev && (lws_done_blk == 2'(b));
        reg         v_r;
        reg  [1:0]  slot_r;
        reg  [6:0]  cnt_r [LG_N];
        reg  [LWM_W-1:0] wm_r;
        // the tracked landing ends this cycle: a beat in the same cycle starts a new one
        wire        cur_v  = v_r && ~(dn && (lws_done_slot == slot_r));
        wire        start  = bt_fire && ~cur_v;
        wire        count  = bt_fire && cur_v && (bt_slot == slot_r);
        wire        wm_adv = cur_v && (32'(wm_r) < LG_N) && (cnt_r[LG_W'(wm_r)] == 7'd64);
        always_ff @(posedge clk) begin
            if (reset) begin
                v_r <= 1'b0;
            end else begin
                if (start) begin
                    v_r    <= 1'b1;
                    slot_r <= bt_slot;
                    wm_r   <= '0;
                    for (integer g = 0; g < LG_N; ++g) cnt_r[g] <= (LG_W'(g) == bt_grp) ? 7'd1 : 7'd0;
                end else begin
                    if (~cur_v) v_r <= 1'b0;
                    if (count) cnt_r[bt_grp] <= cnt_r[bt_grp] + 7'd1;
                    if (wm_adv) wm_r <= wm_r + LWM_W'(1);
                end
            end
        end
        `RUNTIME_ASSERT(~count || (cnt_r[bt_grp] != 7'd64),
            ("%t: *** %s blk%0d landing k-group %0d overflows (a second landing reached slot %0d before the first closed)", $time, INSTANCE_ID, b, bt_grp, slot_r))
        assign trk_v[b]         = v_r;
        assign trk_slot[b]      = slot_r;
        assign trk_wm[b]        = wm_r;
        assign lws_beat_ev[b]   = bt_fire;
        assign lws_beat_slot[b] = bt_slot;
    end
`else
    for (genvar b = 0; b < NB; ++b) begin : g_trk
        localparam LN = b % WLL;
        assign trk_v[b] = 1'b0; assign trk_slot[b] = '0; assign trk_wm[b] = '0;
        assign lws_beat_ev[b]   = wl_fire[LN] && (wl_blk[LN] == 2'(b));
        assign lws_beat_slot[b] = wl_slot[LN];
    end
`endif
`else
    for (genvar b = 0; b < NB; ++b) begin : g_trk
        assign trk_v[b] = 1'b0; assign trk_slot[b] = '0; assign trk_wm[b] = '0;
        assign lws_beat_ev[b] = 1'b0; assign lws_beat_slot[b] = '0;
    end
`endif

    // W banks per block
    logic [NB-1:0][7:0]            wb_we;
    logic [NB-1:0][7:0][WA_W-1:0]  wb_wa;
    logic [NB-1:0][7:0][63:0]      wb_wd;
    logic [NB-1:0][7:0][7:0]       wb_wbe;
    logic [NB-1:0]                 wb_re;
    logic [NB-1:0][7:0][WA_W-1:0]  wb_ra;
    logic [NB-1:0][7:0][63:0]      wb_rd;
    for (genvar b = 0; b < NB; ++b) begin : g_wbanks
        for (genvar bk = 0; bk < 8; ++bk) begin : g_wbank
            (* ram_style = "ultra" *) reg [63:0] mem [WD];
            reg [63:0] rd_r;
            always_ff @(posedge clk) begin
                if (wb_we[b][bk]) begin
                    for (integer j = 0; j < 8; ++j) begin
                        if (wb_wbe[b][bk][j]) mem[wb_wa[b][bk]][j*8 +: 8] <= wb_wd[b][bk][j*8 +: 8];
                    end
                end
                if (wb_re[b]) rd_r <= mem[wb_ra[b][bk]];
            end
            assign wb_rd[b][bk] = rd_r;
        end
    end

    reg [3:0] x_rows_r [NXS];
    reg [6:0] w_rows_r [NB][NWS];

    // ===================================================================================================
    // Loaders (LSU client)
    // ===================================================================================================
    // Per loader: one command at a time. Requests: 8 lanes x 4 B, lane l at word (base + l) of the run.
    //   X / row-major W: run = (row, k .. k+31) of one row.  SIMT8 W: run = chunk c of rows n0 .. n0+3.
    typedef struct packed {
        logic [LID_W-1:0] lid;
        logic [7:0]       row;
        logic [KW_W-1:0]  kw;
    } ld_tag_t;

    logic [NLD-1:0]          lq_valid;
    lsu_req_data_t           lq_req [NLD];
    wire  [NLD-1:0]          lq_grant;

    // loader state (driven inside g_ld_seq)
    wire [NLD-1:0]           ld_act;
    wire [NLD-1:0]           ld_reqs_done;
    wire [7:0]               ld_row  [NLD];   // X: m, W row-major: n, W simt: n0
    wire [KB_W-1:0]          ld_k    [NLD];   // byte offset (row-major) or chunk*8 (simt)

    // current command per loader
    wire [31:0]       lc_src    [NLD];
    wire [7:0]        lc_rows   [NLD];
    wire [KB_W-1:0]   lc_kt     [NLD];
    wire              lc_layout [NLD];
    wire [31:0]       lc_stride [NLD];
    wire              lc_ready  [NLD];   // command present and dependencies met
    ldx_cmd_t lx_head;
    assign lx_head = ldx_q[ldx_rp];
    assign lc_src[0]    = lx_head.src;
    assign lc_rows[0]   = 8'(lx_head.rows);
    assign lc_kt[0]     = lx_head.kt;
    assign lc_layout[0] = 1'b0;
    assign lc_stride[0] = lx_head.stride;
    assign lc_ready[0]  = (ldx_cnt != 0) && gen_ok(x_rd_done[lx_head.slot], lx_head.cap);
    ldw_cmd_t lw_head [NB];
    for (genvar b = 0; b < NB; ++b) begin : g_lw_head
        assign lw_head[b]       = ldw_q[b][ldw_rp[b]];
        assign lc_src[b+1]      = lw_head[b].src;
        assign lc_rows[b+1]     = 8'(lw_head[b].rows);
        assign lc_kt[b+1]       = lw_head[b].kt;
        assign lc_layout[b+1]   = lw_head[b].layout;
        assign lc_stride[b+1]   = lw_head[b].stride;
        assign lc_ready[b+1]    = (ldw_cnt[b] != 0) && gen_ok(w_rd_done[b][lw_head[b].slot], lw_head[b].cap);
    end

`ifdef VX_CFG_LMEM_ENABLE
    localparam LMEM_WORD_LSB = `VX_CFG_LMEM_LOG_SIZE - 2;
    localparam [LSU_ADDR_WIDTH-1:0] LMEM_WORD_BASE = LSU_ADDR_WIDTH'(`VX_CFG_XLEN'(`VX_MEM_LMEM_BASE_ADDR) >> 2);
`endif

    for (genvar l = 0; l < NLD; ++l) begin : g_ld_req
        // request for the current position
        always_comb begin
            automatic logic [31:0] base;
            automatic logic [3:0]  nlanes;
            lq_req[l] = '0;
            lq_req[l].rw = 1'b0;
            if (lc_layout[l]) begin
                // SIMT8: chunk c = ld_k/8, rows n0..n0+3, 8 B each
                base   = lc_src[l] + (32'(ld_k[l]) >> 3) * lc_stride[l] + 32'(ld_row[l]) * 32'd8;
                nlanes = ((32'(lc_rows[l]) - 32'(ld_row[l])) >= 32'd4) ? 4'd8 : 4'(2 * (32'(lc_rows[l]) - 32'(ld_row[l])));
            end else begin
                base   = lc_src[l] + 32'(ld_row[l]) * lc_stride[l] + 32'(ld_k[l]);
                nlanes = ((32'(lc_kt[l]) - 32'(ld_k[l])) >= 32'd32) ? 4'd8 : 4'((32'(lc_kt[l]) - 32'(ld_k[l])) >> 2);
            end
            for (integer i = 0; i < NL; ++i) begin
                automatic mem_bus_attr_t a;
                automatic logic [LSU_ADDR_WIDTH-1:0] wa;
                wa = LSU_ADDR_WIDTH'((base >> 2) + 32'(i));
                a = '0;
            `ifdef VX_CFG_LMEM_ENABLE
                a.is_addr_local = (wa[LSU_ADDR_WIDTH-1:LMEM_WORD_LSB] == LMEM_WORD_BASE[LSU_ADDR_WIDTH-1:LMEM_WORD_LSB]);
            `endif
                lq_req[l].mask[i]   = (4'(i) < nlanes);
                lq_req[l].addr[i]   = wa;
                lq_req[l].byteen[i] = {LSU_WORD_SIZE{1'b1}};
                lq_req[l].attr[i]   = a;
                lq_req[l].data[i]   = '0;
            end
            begin
                automatic ld_tag_t t;
                t.lid = LID_W'(l);
                t.row = ld_row[l];
                t.kw  = lc_layout[l] ? KW_W'(32'(ld_k[l]) >> 2) : KW_W'(32'(ld_k[l]) >> 2);
                lq_req[l].tag = LSU_CLIENT_TAG_WIDTH'(t);
            end
        end
        assign lq_valid[l] = ld_act[l] && ~ld_reqs_done[l];
    end

    // round-robin over loaders
    localparam LID_RR_W = `UP(LID_W);
    reg [LID_RR_W-1:0] ld_rr_r;
    logic [LID_RR_W-1:0] ld_sel;
    logic                ld_any;
    always_comb begin
        ld_sel = ld_rr_r;
        ld_any = 1'b0;
        for (integer i = 0; i < NLD; ++i) begin
            automatic logic [LID_RR_W-1:0] s = LID_RR_W'((32'(ld_rr_r) + i) % NLD);
            if (!ld_any && lq_valid[s]) begin
                ld_sel = s;
                ld_any = 1'b1;
            end
        end
    end
    assign mem_if.req_valid = ld_any;
    assign mem_if.req_data  = lq_req[ld_sel];
    wire ld_req_fire = ld_any && mem_if.req_ready;
    for (genvar l = 0; l < NLD; ++l) begin : g_lq_grant
        assign lq_grant[l] = ld_req_fire && (ld_sel == LID_RR_W'(l));
    end

    always_ff @(posedge clk) begin
        if (reset) ld_rr_r <= '0;
        else if (ld_req_fire) ld_rr_r <= LID_RR_W'((32'(ld_sel) + 1) % NLD);
    end

    // responses
    assign mem_if.rsp_ready = 1'b1;
    ld_tag_t rsp_tag;
    assign rsp_tag = ld_tag_t'(mem_if.rsp_data.tag[$bits(ld_tag_t)-1:0]);
    `UNUSED_VAR (mem_if.rsp_data.sop)
    `UNUSED_VAR (mem_if.rsp_data.eop)
    if (LSU_CLIENT_TAG_WIDTH > $bits(ld_tag_t)) begin : g_tag_unused
        `UNUSED_VAR (mem_if.rsp_data.tag[LSU_CLIENT_TAG_WIDTH-1:$bits(ld_tag_t)])
    end
    wire rsp_fire = mem_if.rsp_valid;
    wire [3:0] rsp_nwords = 4'($countones(mem_if.rsp_data.mask));

    // loader sequencing
    wire [NLD-1:0] ld_start, ld_finish;
    for (genvar l = 0; l < NLD; ++l) begin : g_ld_seq
        reg             act_r, reqs_done_r;
        reg [7:0]       row_r;
        reg [KB_W-1:0]  k_r;
        reg [31:0]      words_exp_r, words_got_r;
        wire got = rsp_fire && (rsp_tag.lid == LID_W'(l));
        assign ld_start[l]     = ~act_r && lc_ready[l];
        assign ld_act[l]       = act_r;
        assign ld_reqs_done[l] = reqs_done_r;
        assign ld_row[l]       = row_r;
        assign ld_k[l]         = k_r;
        // finish once every requested word has been written
        assign ld_finish[l]    = act_r && reqs_done_r && (words_got_r == words_exp_r);
        always_ff @(posedge clk) begin
            if (reset) begin
                act_r       <= 1'b0;
                reqs_done_r <= 1'b0;
                row_r       <= '0;
                k_r         <= '0;
                words_exp_r <= '0;
                words_got_r <= '0;
            end else begin
                if (ld_start[l]) begin
                    act_r       <= 1'b1;
                    reqs_done_r <= 1'b0;
                    row_r       <= '0;
                    k_r         <= '0;
                    words_exp_r <= (32'(lc_rows[l]) * 32'(lc_kt[l])) >> 2;
                    words_got_r <= '0;
                end else begin
                    if (lq_grant[l]) begin
                        if (lc_layout[l]) begin
                            // advance n0 by 4; at the end of the rows go to the next chunk
                            if ((32'(row_r) + 32'd4) >= 32'(lc_rows[l])) begin
                                row_r <= '0;
                                if ((32'(k_r) + 32'd8) >= 32'(lc_kt[l])) reqs_done_r <= 1'b1;
                                k_r <= k_r + KB_W'(8);
                            end else begin
                                row_r <= row_r + 8'd4;
                            end
                        end else begin
                            if ((32'(k_r) + 32'd32) >= 32'(lc_kt[l])) begin
                                k_r <= '0;
                                if ((32'(row_r) + 32'd1) >= 32'(lc_rows[l])) reqs_done_r <= 1'b1;
                                row_r <= row_r + 8'd1;
                            end else begin
                                k_r <= k_r + KB_W'(32);
                            end
                        end
                    end
                    if (got) begin
                        words_got_r <= words_got_r + 32'(rsp_nwords);
                    end
                    if (ld_finish[l]) begin
                        act_r <= 1'b0;
                    end
                end
            end
        end
    end

    // response -> bank writes. Lane pair p = (2p, 2p+1) shares one 64-bit chunk.
    wire [2:0] lx_slot = lx_head.slot;
    always_comb begin
        xb_we = '0; xb_wa = '0; xb_wd = '0; xb_wbe = '0;
        wb_we = '0; wb_wa = '0; wb_wd = '0; wb_wbe = '0;
        for (integer p = 0; p < 4; ++p) begin
            automatic logic [1:0]  m2 = mem_if.rsp_data.mask[2*p +: 2];
            automatic logic [7:0]  row;
            automatic logic [CH_W-1:0] ch;
            automatic logic [63:0] d;
            automatic logic [7:0]  be;
            if (rsp_tag.lid == LID_W'(0)) begin
                row = rsp_tag.row;
                ch  = CH_W'((32'(rsp_tag.kw) + 32'(2 * p)) >> 1);
            end else if (lc_layout[rsp_tag.lid]) begin
                row = rsp_tag.row + 8'(p);
                ch  = CH_W'(32'(rsp_tag.kw) >> 1);
            end else begin
                row = rsp_tag.row;
                ch  = CH_W'((32'(rsp_tag.kw) + 32'(2 * p)) >> 1);
            end
            d  = {mem_if.rsp_data.data[2*p+1], mem_if.rsp_data.data[2*p]};
            be = {{4{m2[1]}}, {4{m2[0]}}};
            if (rsp_fire && (m2 != 2'b00)) begin
                if (rsp_tag.lid == LID_W'(0)) begin
                    automatic logic [2:0] bk = 3'(32'(row) + 32'(ch));
                    xb_we[bk]  = 1'b1;
                    xb_wa[bk]  = XA_W'((32'(lx_slot) * XROWS + 32'(row)) * CX + (32'(ch) >> 3));
                    xb_wd[bk]  = d;
                    xb_wbe[bk] = be;
                end else begin
                    for (integer b = 0; b < NB; ++b) begin
                        if (rsp_tag.lid == LID_W'(b + 1)) begin
                            automatic logic [2:0] bk = 3'(32'(row) + 32'(ch));
                            wb_we[b][bk]  = 1'b1;
                            wb_wa[b][bk]  = WA_W'((32'(lw_head[b].slot) * WROWS + 32'(row)) * CX + (32'(ch) >> 3));
                            wb_wd[b][bk]  = d;
                            wb_wbe[b][bk] = be;
                        end
                    end
                end
            end
        end
    `ifdef VX_CFG_TSU_ENABLE
        // stream landing: one 64 B row segment = chunks seg*8 .. seg*8+7 of row n -> all 8 banks, one address
        // (bank = (n + ch) mod 8, entry = (slot*64 + n)*CX + ch/8). Never overlaps an LSU write (wl_ready).
        // SIMT8 beat: 8 B piece j = chunk c (= seg) of row r0 + j (r0 = row, a multiple of 8) -> bank (r0 + j + c)
        // mod 8, entry (slot*64 + r0 + j)*CX + c/8: 8 distinct banks, one address each.
        for (integer l = 0; l < WLL; ++l) begin
            if (wl_fire[l]) begin
                for (integer j = 0; j < 8; ++j) begin
                    automatic logic [2:0] bk = wl_beat[l].simt ? 3'(32'(wl_beat[l].row) + 32'(j) + 32'(wl_beat[l].seg))
                                                               : 3'(32'(wl_beat[l].row) + 32'(j));
                    wb_we[wl_blk[l]][bk]  = 1'b1;
                    wb_wa[wl_blk[l]][bk]  = wl_beat[l].simt
                        ? WA_W'((32'(wl_slot[l]) * WROWS + 32'(wl_beat[l].row) + 32'(j)) * CX + (32'(wl_beat[l].seg) >> 3))
                        : WA_W'((32'(wl_slot[l]) * WROWS + 32'(wl_beat[l].row)) * CX + 32'(wl_beat[l].seg));
                    wb_wd[wl_blk[l]][bk]  = wl_beat[l].data[j*64 +: 64];
                    wb_wbe[wl_blk[l]][bk] = 8'hFF;
                end
            end
        end
    `endif
    end
    for (genvar b = 0; b < NB; ++b) begin : g_wb_lsu_busy
        assign wb_lsu_busy[b] = rsp_fire && (|mem_if.rsp_data.mask) && (rsp_tag.lid == LID_W'(b + 1));
    end

    // loader completion events, row-count registers
    assign ldx_pop       = ld_finish[0];
    assign ldx_done_ev   = ld_finish[0];
    assign ldx_done_wid  = lx_head.wid;
    assign ldx_done_slot = lx_head.slot;
    for (genvar b = 0; b < NB; ++b) begin : g_ldw_done
        assign ldw_pop[b]       = ld_finish[b+1];
        assign ldw_done_ev[b]   = ld_finish[b+1];
        assign ldw_done_wid[b]  = lw_head[b].wid;
        assign ldw_done_slot[b] = lw_head[b].slot;
    end
    always_ff @(posedge clk) begin
        if (reset) begin
            for (integer i = 0; i < NXS; ++i) x_rows_r[i] <= '0;
            for (integer b = 0; b < NB; ++b) for (integer i = 0; i < NWS; ++i) w_rows_r[b][i] <= '0;
        end else begin
            if (ld_start[0]) x_rows_r[lx_head.slot] <= lx_head.rows;
            for (integer b = 0; b < NB; ++b) begin
                if (ld_start[b+1]) w_rows_r[b][lw_head[b].slot] <= lw_head[b].rows;
            end
            // stream landings are always full 64-row slabs
            if (lws_done_ev) w_rows_r[lws_done_blk][lws_done_slot] <= 7'd64;
            // ... and already from their first beat, for a tgemm consuming a partly landed slot (the WAR cap
            // guarantees no earlier tgemm still reads the slot once a beat is accepted)
            for (integer b = 0; b < NB; ++b) begin
                if (lws_beat_ev[b]) w_rows_r[b][lws_beat_slot[b]] <= 7'd64;
            end
        end
    end

    // ===================================================================================================
    // X read arbitration (one block per cycle reads one chunk of all 8 rows)
    // ===================================================================================================
    wire [NB-1:0]      xr_req;
    wire [NB-1:0]      xr_gnt;
    wire [2:0]         xr_slot [NB];
    wire [CH_W-1:0]    xr_ch   [NB];
    reg  [BK_W-1:0]    xr_rr_r;
    logic [BK_W-1:0]   xr_sel;
    logic              xr_any;
    always_comb begin
        xr_sel = xr_rr_r;
        xr_any = 1'b0;
        for (integer i = 0; i < NB; ++i) begin
            automatic logic [BK_W-1:0] s = BK_W'((32'(xr_rr_r) + i) % NB);
            if (!xr_any && xr_req[s]) begin
                xr_sel = s;
                xr_any = 1'b1;
            end
        end
    end
    always_ff @(posedge clk) begin
        if (reset) xr_rr_r <= '0;
        else if (xr_any) xr_rr_r <= BK_W'((32'(xr_sel) + 1) % NB);
    end
    for (genvar b = 0; b < NB; ++b) begin : g_xr_gnt
        assign xr_gnt[b] = xr_any && (xr_sel == BK_W'(b));
    end
    assign xb_re = xr_any;
    always_comb begin
        for (integer bk = 0; bk < 8; ++bk) begin
            automatic logic [2:0] m = 3'(bk - 32'(xr_ch[xr_sel]));
            xb_ra[bk] = XA_W'((32'(xr_slot[xr_sel]) * XROWS + 32'(m)) * CX + (32'(xr_ch[xr_sel]) >> 3));
        end
    end
    reg [BK_W-1:0] xr_sel_d;
    reg            xr_any_d;
    reg [CH_W-1:0] xr_ch_d;
    reg [2:0]      xr_slot_d;
    always_ff @(posedge clk) begin
        if (reset) begin
            xr_any_d <= 1'b0;
        end else begin
            xr_any_d <= xr_any;
        end
        xr_sel_d  <= xr_sel;
        xr_ch_d   <= xr_ch[xr_sel];
        xr_slot_d <= xr_slot[xr_sel];
    end
    // de-skewed X rows of the last read
    logic [7:0][63:0] xr_rows;
    always_comb begin
        for (integer m = 0; m < 8; ++m) begin
            automatic logic [2:0] bk = 3'(32'(m) + 32'(xr_ch_d));
            xr_rows[m] = (4'(m) < x_rows_r[xr_slot_d]) ? xb_rd[bk] : 64'd0;
        end
    end

    // ===================================================================================================
    // Per-block MAC engine, drain, store engine
    // ===================================================================================================
    // LMEM DMA requests from the store engines (round-robin)
    logic [NB-1:0]             dq_valid;
    logic [NB-1:0]             dq_rw;
    logic [NB-1:0][LMEM_ROW_W-1:0] dq_row;
    logic [NB-1:0][255:0]      dq_data;
    logic [NB-1:0][31:0]       dq_be;
    wire  [NB-1:0]             dq_grant;
    wire  [NB-1:0]             dq_rsp;

    // perf (simulation report)
    reg [63:0] perf_gemms, perf_gemm_k, perf_busy, perf_slot_wait, perf_drain, perf_ldx, perf_ldw, perf_st;
    reg [63:0] perf_lws, perf_lws_beats;   // stream landings (transfers) and landed 64 B beats

    wire [NB-1:0] perf_blk_busy, perf_blk_wait, perf_blk_drain;
    wire [NB-1:0] perf_blk_lwait, perf_blk_early;   // fill stalled on a partly landed slot; tgemm started on one
    reg  [63:0]   perf_land_wait, perf_early;
    wire [NB-1:0] perf_blk_wx, perf_blk_ww, perf_blk_wa;   // slot_wait split by the unmet interlock (X / W / acc tile)
    reg  [63:0]   perf_wait_x, perf_wait_w, perf_wait_a;

    for (genvar b = 0; b < NB; ++b) begin : g_blk
        // ------------------------------------------------ MAC fill
        gemm_cmd_t gh;
        assign gh = gm_q[b][gm_rp[b]];
        wire gh_v  = (gm_cnt[b] != 0);
        // W readiness: the slot's loads are all done, or (landed watermark) the one load still outstanding
        // is the slot's head stream landing, which the block tracker is following: then the fill consumes
        // k-group g once the watermark has passed it.
        wire gh_w_done = gen_ok(w_ld_done[b][gh.ws], gh.capw);
        wire gh_w_strm = ((gh.capw - w_ld_done[b][gh.ws]) == GEN_W'(1)) && (lws_cnt[b][gh.ws] != 2'd0)
                       && trk_v[b] && (trk_slot[b] == gh.ws);
        wire gh_ok = gh_v && gen_ok(x_ld_done[gh.xs], gh.capx) && (gh_w_done || gh_w_strm)
                          && gen_ok(a_st_done[b][gh.tile], gh.capa);

        // group buffers
        reg  [1:0]             gb_v;
        reg  [7:0][63:0]       gb_x  [2];
        reg  [63:0]            gb_w  [2][WROWS];
        reg  [1:0]             gb_first, gb_last;
        gemm_cmd_t             gb_cmd [2];

        reg                    fl_act;      // filling the current command
        reg  [KB_W-1:0]        fl_g;        // group index (bytes / 8)
        reg  [3:0]             fl_t;        // 0..7 W read step, 8 = X/W reads done
        reg                    fl_buf;
        reg                    fl_xdone;
        wire [KB_W-1:0]        fl_groups = KB_W'(32'(gh.k) >> 3);
        wire [CH_W-1:0]        fl_ch     = CH_W'((32'(gh.koff) >> 3) + 32'(fl_g));
        // the group's W chunk has landed (whole slot loaded, or k-group fl_ch / 8 below the landed watermark)
        wire                   fl_w_av   = gh_w_done || (gh_w_strm && ((32'(fl_ch) >> 3) < 32'(trk_wm[b])));
        wire                   fl_can    = fl_act && ~gb_v[fl_buf] && fl_w_av;

        assign xr_req[b]  = fl_can && ~fl_xdone;
        assign xr_slot[b] = gh.xs;
        assign xr_ch[b]   = fl_ch;

        // W reads: step t reads column n = 8t + ((bk - c) mod 8) from bank bk
        wire w_rd = fl_can && (fl_t < 4'd8);
        assign wb_re[b] = w_rd;
        always_comb begin
            for (integer bk = 0; bk < 8; ++bk) begin
                automatic logic [2:0] off = 3'(bk - 32'(fl_ch));
                automatic logic [5:0] n   = 6'(32'(fl_t) * 8 + 32'(off));
                wb_ra[b][bk] = WA_W'((32'(gh.ws) * WROWS + 32'(n)) * CX + (32'(fl_ch) >> 3));
            end
        end
        reg        w_rd_d;
        reg [3:0]  w_t_d;
        reg [CH_W-1:0] w_ch_d;
        reg        w_buf_d;
        reg [1:0]  w_ws_d;
        reg        fl_last_step_d;
        reg        fl_first_g_d, fl_last_g_d;

        // X read happens with step 0; step 0 advances only once the X read is granted (or already done)
        wire step_adv  = w_rd && ((fl_t != 4'd0) || fl_xdone || xr_gnt[b]);

        // consume-side state (declared here: the buffer valid bits are cleared by the consumer)
        reg        cs_buf;
        reg [2:0]  cs_j;
        wire       cs_free;
        reg        w_buf_d_x;   // buffer that the X read granted this cycle fills

        always_ff @(posedge clk) begin
            if (reset) begin
                fl_act   <= 1'b0;
                fl_g     <= '0;
                fl_t     <= '0;
                fl_buf   <= 1'b0;
                fl_xdone <= 1'b0;
                w_rd_d   <= 1'b0;
                fl_last_step_d <= 1'b0;
            end else begin
                w_rd_d <= step_adv;
                fl_last_step_d <= step_adv && (fl_t == 4'd7);
                if (~fl_act) begin
                    if (gh_ok) begin
                        fl_act   <= 1'b1;
                        fl_g     <= '0;
                        fl_t     <= '0;
                        fl_xdone <= 1'b0;
                    end
                end else begin
                    if (xr_gnt[b]) fl_xdone <= 1'b1;
                    if (step_adv) begin
                        if (fl_t == 4'd7) begin
                            fl_t     <= '0;
                            fl_xdone <= 1'b0;
                            fl_buf   <= ~fl_buf;
                            if ((32'(fl_g) + 32'd1) >= 32'(fl_groups)) begin
                                fl_act <= 1'b0;
                            end
                            fl_g <= fl_g + KB_W'(1);
                        end else begin
                            fl_t <= fl_t + 4'd1;
                        end
                    end
                end
            end
            w_t_d   <= fl_t;
            w_ch_d  <= fl_ch;
            w_buf_d <= fl_buf;
            w_ws_d  <= gh.ws;
            fl_first_g_d <= (fl_g == '0);
            fl_last_g_d  <= ((32'(fl_g) + 32'd1) >= 32'(fl_groups));
        end
        // the command leaves the queue when its last group's reads have been issued
        assign gm_pop[b] = fl_act && step_adv && (fl_t == 4'd7) && ((32'(fl_g) + 32'd1) >= 32'(fl_groups));
        gemm_cmd_t gh_d;
        always_ff @(posedge clk) begin
            gh_d <= gh;
        end

        // capture read data into the group buffer
        always_ff @(posedge clk) begin
            if (reset) begin
                gb_v <= '0;
            end else begin
                if (fl_last_step_d) begin
                    gb_v[w_buf_d]     <= 1'b1;
                    gb_first[w_buf_d] <= fl_first_g_d;
                    gb_last[w_buf_d]  <= fl_last_g_d;
                    gb_cmd[w_buf_d]   <= gh_d;
                end
                if (cs_free) gb_v[cs_buf] <= 1'b0;
            end
            if (w_rd_d) begin
                for (integer bk = 0; bk < 8; ++bk) begin
                    automatic logic [2:0] off = 3'(bk - 32'(w_ch_d));
                    automatic logic [5:0] n   = 6'(32'(w_t_d) * 8 + 32'(off));
                    gb_w[w_buf_d][n] <= (7'(n) < w_rows_r[b][w_ws_d]) ? wb_rd[b][bk] : 64'd0;
                end
            end
            if (xr_any_d && (xr_sel_d == BK_W'(b))) begin
                gb_x[w_buf_d_x] <= xr_rows;
            end
        end
        // X read data lands in the buffer being filled at grant time
        always_ff @(posedge clk) begin
            w_buf_d_x <= fl_buf;
        end

        // ------------------------------------------------ MAC consume
        wire       cs_v     = gb_v[cs_buf];
        wire       cs_first = gb_first[cs_buf] && (cs_j == 3'd0);
        wire       cs_last  = gb_last[cs_buf] && (cs_j == 3'd7);

        // span metadata FIFO (pushed on a span's first beat, popped on its last drained beat)
        localparam SQ = 4;
        gemm_cmd_t sp_q [SQ];
        reg [2:0]  sp_cnt;
        reg [1:0]  sp_wp, sp_rp;
        wire       sp_full = (sp_cnt == 3'(SQ));

        wire [7:0][7:0]      in_a;
        wire [WROWS-1:0][7:0] in_b;
        for (genvar m = 0; m < 8; ++m) begin : g_in_a
            assign in_a[m] = gb_x[cs_buf][m][8*cs_j +: 8];
        end
        for (genvar n = 0; n < WROWS; ++n) begin : g_in_b
            assign in_b[n] = gb_w[cs_buf][n][8*cs_j +: 8];
        end

        wire mac_in_ready;
        wire mac_in_valid = cs_v && (~cs_first || ~sp_full);
        wire mac_in_fire  = mac_in_valid && mac_in_ready;
        assign cs_free    = mac_in_fire && (cs_j == 3'd7);

        always_ff @(posedge clk) begin
            if (reset) begin
                cs_buf <= 1'b0;
                cs_j   <= '0;
            end else if (mac_in_fire) begin
                cs_j <= cs_j + 3'd1;
                if (cs_j == 3'd7) cs_buf <= ~cs_buf;
            end
        end

        wire                    mac_out_valid;
        wire                    mac_out_ready;
        wire [1:0][7:0][31:0]   mac_out_data;
        wire [4:0]              mac_out_col;
        wire                    mac_out_last;

        VX_tcu_mac_block #(
            .ROWS     (8),
            .COLS     (WROWS),
            .OUT_COLS (2)
        ) mac (
            .clk       (clk),
            .reset     (reset),
            .in_valid  (mac_in_valid),
            .in_ready  (mac_in_ready),
            .in_first  (cs_first),
            .in_last   (cs_last),
            .in_signed (1'b1),
            .in_a      (in_a),
            .in_b      (in_b),
            .out_valid (mac_out_valid),
            .out_ready (mac_out_ready),
            .out_data  (mac_out_data),
            .out_col   (mac_out_col),
            .out_last  (mac_out_last)
        );

        // ------------------------------------------------ accumulator tiles + drain RMW
        localparam ACC_A_W = `CLOG2(ACC_D);
        logic               acc_we;
        logic [ACC_A_W-1:0] acc_wa;
        logic [ACC_W-1:0]   acc_wd;
        logic               acc_re;
        logic [ACC_A_W-1:0] acc_ra;
        reg   [ACC_W-1:0]   acc_rd;
        (* ram_style = "ultra" *) reg [ACC_W-1:0] acc_mem [ACC_D];
        always_ff @(posedge clk) begin
            if (acc_we) acc_mem[acc_wa] <= acc_wd;
            if (acc_re) acc_rd <= acc_mem[acc_ra];
        end

        gemm_cmd_t sp_head;
        assign sp_head = sp_q[sp_rp];
        `UNUSED_VAR (sp_head)
        wire [ACC_A_W-1:0] dr_addr = ACC_A_W'(32'(sp_head.tile) * 32 + 32'(mac_out_col));
        reg  dr_rd_pend;      // accumulate: read issued last cycle for the presented beat
        wire dr_need_rd = sp_head.accum;
        wire dr_rd_issue = mac_out_valid && dr_need_rd && ~dr_rd_pend;
        assign mac_out_ready = mac_out_valid && (~dr_need_rd || dr_rd_pend);
        wire dr_fire = mac_out_valid && mac_out_ready;

        // store engine read requests (lower priority than the drain)
        wire               st_rd_req;
        wire [ACC_A_W-1:0] st_rd_addr;
        wire               st_rd_gnt = st_rd_req && ~dr_rd_issue;

        assign acc_re = dr_rd_issue || st_rd_gnt;
        assign acc_ra = dr_rd_issue ? dr_addr : st_rd_addr;
        assign acc_we = dr_fire;
        assign acc_wa = dr_addr;
        always_comb begin
            for (integer j = 0; j < 2; ++j) begin
                for (integer r = 0; r < 8; ++r) begin
                    automatic logic [31:0] old = dr_need_rd ? acc_rd[(j*8 + r)*32 +: 32] : 32'd0;
                    acc_wd[(j*8 + r)*32 +: 32] = old + mac_out_data[j][r];
                end
            end
        end
        always_ff @(posedge clk) begin
            if (reset) begin
                dr_rd_pend <= 1'b0;
            end else begin
                if (dr_fire) dr_rd_pend <= 1'b0;
                else if (dr_rd_issue) dr_rd_pend <= 1'b1;
            end
        end

        wire sp_push = mac_in_fire && cs_first;
        wire sp_pop  = dr_fire && mac_out_last;
        always_ff @(posedge clk) begin
            if (reset) begin
                sp_cnt <= '0; sp_wp <= '0; sp_rp <= '0;
            end else begin
                if (sp_push) begin sp_q[sp_wp] <= gb_cmd[cs_buf]; sp_wp <= sp_wp + 2'd1; end
                if (sp_pop) sp_rp <= sp_rp + 2'd1;
                sp_cnt <= sp_cnt + 3'(sp_push) - 3'(sp_pop);
            end
        end
        `RUNTIME_ASSERT(~mac_out_valid || (sp_cnt != 0), ("%t: *** %s blk%0d: drained beat without span", $time, INSTANCE_ID, b))

        assign gm_done_ev[b]   = sp_pop;
        assign gm_done_wid[b]  = sp_head.wid;
        assign gm_done_xs[b]   = sp_head.xs;
        assign gm_done_ws[b]   = sp_head.ws;
        assign gm_done_tile[b] = sp_head.tile;

        // ------------------------------------------------ store engine (tacc.st)
        st_cmd_t sh;
        assign sh = st_q[b][st_rp[b]];
        wire sh_ok = (st_cnt[b] != 0) && gen_ok(a_gm_done[b][sh.tile], sh.cap);

        // phases: 0 idle, 1 read 8 beats of quarter q, 2 write 16 row chunks, 3 read-back, 4 wait rsp
        reg [2:0]  so_ph;
        reg [1:0]  so_q;
        reg [3:0]  so_i;          // read: beat index 0..7 (+1 latency) ; write: chunk index 0..15
        reg        so_half;       // unaligned chunk: second write
        reg [ACC_W-1:0] so_buf [8];
        reg        so_rd_d;
        reg [2:0]  so_rd_i_d;
        reg [LMEM_ROW_W-1:0] so_last_row;

        assign st_rd_req  = (so_ph == 3'd1) && (so_i < 4'd8);
        assign st_rd_addr = ACC_A_W'(32'(sh.tile) * 32 + 32'(so_q) * 8 + 32'(so_i));

        // chunk (j, m) of quarter q: 8 int32 = beats 0..7 of the quarter, lane (j, m)
        wire       so_j = so_i[3];
        wire [2:0] so_m = so_i[2:0];
        wire [31:0] so_addr = sh.dst + 32'(so_m) * sh.stride + (32'(so_j) * 32 + 32'(so_q) * 8) * 4;
        wire [2:0]  so_off  = so_addr[4:2];
        `UNUSED_VAR (so_addr)
        logic [7:0][31:0] so_chunk;
        always_comb begin
            for (integer i = 0; i < 8; ++i) so_chunk[i] = so_buf[i][(32'(so_j) * 8 + 32'(so_m)) * 32 +: 32];
        end
        logic [7:0][31:0] so_wdata;
        logic [31:0]      so_wbe;
        logic [LMEM_ROW_W-1:0] so_wrow;
        always_comb begin
            so_wdata = '0;
            so_wbe   = '0;
            so_wrow  = LMEM_ROW_W'(so_addr[`VX_CFG_LMEM_LOG_SIZE-1:5]);
            for (integer w = 0; w < 8; ++w) begin
                if (~so_half) begin
                    // words off..7 of row r <- chunk[0 .. 7-off]
                    if (w >= 32'(so_off)) begin
                        so_wdata[w] = so_chunk[w - 32'(so_off)];
                        so_wbe[w*4 +: 4] = 4'hF;
                    end
                end else begin
                    // words 0..off-1 of row r+1 <- chunk[8-off .. 7]
                    if (w < 32'(so_off)) begin
                        so_wdata[w] = so_chunk[8 - 32'(so_off) + w];
                        so_wbe[w*4 +: 4] = 4'hF;
                    end
                end
            end
            if (so_half) so_wrow = so_wrow + LMEM_ROW_W'(1);
        end

        assign dq_valid[b] = (so_ph == 3'd2) || (so_ph == 3'd3);
        assign dq_rw[b]    = (so_ph == 3'd2);
        assign dq_row[b]   = (so_ph == 3'd3) ? so_last_row : so_wrow;
        assign dq_data[b]  = so_wdata;
        assign dq_be[b]    = (so_ph == 3'd3) ? 32'd0 : so_wbe;

        always_ff @(posedge clk) begin
            if (reset) begin
                so_ph   <= '0;
                so_q    <= '0;
                so_i    <= '0;
                so_half <= 1'b0;
                so_rd_d <= 1'b0;
            end else begin
                so_rd_d <= st_rd_gnt;
                case (so_ph)
                3'd0: if (sh_ok) begin
                    so_ph <= 3'd1; so_q <= '0; so_i <= '0;
                end
                3'd1: begin
                    if (st_rd_gnt) so_i <= so_i + 4'd1;
                    if ((so_i == 4'd8) && ~so_rd_d) begin
                        so_ph <= 3'd2; so_i <= '0; so_half <= 1'b0;
                    end
                end
                3'd2: if (dq_grant[b]) begin
                    so_last_row <= so_wrow;
                    if (~so_half && (so_off != 3'd0)) begin
                        so_half <= 1'b1;
                    end else begin
                        so_half <= 1'b0;
                        if (so_i == 4'd15) begin
                            if (so_q == 2'd3) begin
                                so_ph <= 3'd3;
                            end else begin
                                so_q <= so_q + 2'd1; so_i <= '0; so_ph <= 3'd1;
                            end
                        end else begin
                            so_i <= so_i + 4'd1;
                        end
                    end
                end
                3'd3: if (dq_grant[b]) so_ph <= 3'd4;
                3'd4: if (dq_rsp[b]) so_ph <= 3'd0;
                default: so_ph <= 3'd0;
                endcase
            end
            so_rd_i_d <= 3'(so_i);
            if (so_rd_d) so_buf[so_rd_i_d] <= acc_rd;
        end
        `UNUSED_VAR (so_rd_i_d)
        assign st_pop[b]       = (so_ph == 3'd4) && dq_rsp[b];
        assign st_done_ev[b]   = st_pop[b];
        assign st_done_wid[b]  = sh.wid;
        assign st_done_tile[b] = sh.tile;

        assign perf_blk_busy[b]  = mac_in_fire;
        assign perf_blk_wait[b]  = gh_v && ~gh_ok && ~fl_act;
        assign perf_blk_drain[b] = dr_fire;
        assign perf_blk_lwait[b] = fl_act && ~gb_v[fl_buf] && ~fl_w_av;
        assign perf_blk_early[b] = ~fl_act && gh_ok && ~gh_w_done;
        assign perf_blk_wx[b]    = perf_blk_wait[b] && ~gen_ok(x_ld_done[gh.xs], gh.capx);
        assign perf_blk_ww[b]    = perf_blk_wait[b] && ~(gh_w_done || gh_w_strm);
        assign perf_blk_wa[b]    = perf_blk_wait[b] && ~gen_ok(a_st_done[b][gh.tile], gh.capa);
    end

    // ===================================================================================================
    // LMEM DMA port (store engines, round-robin; tag value = block)
    // ===================================================================================================
    reg  [BK_W-1:0] dq_rr_r;
    logic [BK_W-1:0] dq_sel;
    logic            dq_any;
    always_comb begin
        dq_sel = dq_rr_r;
        dq_any = 1'b0;
        for (integer i = 0; i < NB; ++i) begin
            automatic logic [BK_W-1:0] s = BK_W'((32'(dq_rr_r) + i) % NB);
            if (!dq_any && dq_valid[s]) begin
                dq_sel = s;
                dq_any = 1'b1;
            end
        end
    end
    assign lmem_if.req_valid        = dq_any;
    assign lmem_if.req_data.rw      = dq_rw[dq_sel];
    assign lmem_if.req_data.addr    = dq_row[dq_sel];
    assign lmem_if.req_data.data    = dq_data[dq_sel];
    assign lmem_if.req_data.byteen  = dq_be[dq_sel];
    assign lmem_if.req_data.attr    = '0;
    assign lmem_if.req_data.tag.uuid  = '0;
    assign lmem_if.req_data.tag.value = DMA_TAG_VW'(dq_sel);
    wire dq_fire = dq_any && lmem_if.req_ready;
    always_ff @(posedge clk) begin
        if (reset) dq_rr_r <= '0;
        else if (dq_fire) dq_rr_r <= BK_W'((32'(dq_sel) + 1) % NB);
    end
    for (genvar b = 0; b < NB; ++b) begin : g_dq
        assign dq_grant[b] = dq_fire && (dq_sel == BK_W'(b));
        assign dq_rsp[b]   = lmem_if.rsp_valid && (lmem_if.rsp_data.tag.value[BK_W-1:0] == BK_W'(b));
    end
    assign lmem_if.rsp_ready = 1'b1;
    `UNUSED_VAR (lmem_if.rsp_data)

    // ===================================================================================================
    // Perf (reported at the end of simulation; CSR export is future work)
    // ===================================================================================================
    always_ff @(posedge clk) begin
        if (reset) begin
            perf_gemms <= '0; perf_gemm_k <= '0; perf_busy <= '0; perf_slot_wait <= '0; perf_drain <= '0;
            perf_ldx <= '0; perf_ldw <= '0; perf_st <= '0; perf_lws <= '0; perf_lws_beats <= '0;
            perf_land_wait <= '0; perf_early <= '0; perf_wait_x <= '0; perf_wait_w <= '0; perf_wait_a <= '0;
        end else begin
            perf_gemms     <= perf_gemms + 64'($countones(gm_done_ev));
            perf_busy      <= perf_busy + 64'($countones(perf_blk_busy));
            perf_slot_wait <= perf_slot_wait + 64'($countones(perf_blk_wait));
            perf_drain     <= perf_drain + 64'($countones(perf_blk_drain));
            perf_ldx       <= perf_ldx + 64'(ldx_done_ev);
            perf_ldw       <= perf_ldw + 64'($countones(ldw_done_ev));
            perf_lws       <= perf_lws + 64'(lws_done_ev);
            perf_lws_beats <= perf_lws_beats + 64'($countones(wl_fire));
            perf_st        <= perf_st + 64'($countones(st_done_ev));
            perf_land_wait <= perf_land_wait + 64'($countones(perf_blk_lwait));
            perf_early     <= perf_early + 64'($countones(perf_blk_early));
            perf_wait_x    <= perf_wait_x + 64'($countones(perf_blk_wx));
            perf_wait_w    <= perf_wait_w + 64'($countones(perf_blk_ww));
            perf_wait_a    <= perf_wait_a + 64'($countones(perf_blk_wa));
            perf_gemm_k    <= perf_gemm_k + 64'(f_fire && (f_op == OP_GEMM) ? f_rs1 : 32'd0);
        end
    end
`ifndef SYNTHESIS
    // Hosts that never close the device skip final blocks: also print the cumulative counters each time every
    // warp's pending count has stayed zero for 4096 cycles after new work (one line per kernel call).
    reg perf_dirty;
    reg [63:0] perf_win_cyc;
    reg [12:0] perf_idle_cnt;
    always_ff @(posedge clk) begin
        automatic logic all_idle = 1'b1;
        for (integer w = 0; w < NW; ++w) if (pending[w] != 16'd0) all_idle = 1'b0;
        if (reset) begin
            perf_dirty <= 1'b0; perf_win_cyc <= '0; perf_idle_cnt <= '0;
        end else begin
            perf_win_cyc  <= perf_dirty ? perf_win_cyc + 64'd1 : 64'd0;
            perf_idle_cnt <= (all_idle && !(f_fire && f_async)) ? perf_idle_cnt + 13'd1 : 13'd0;
            if (f_fire && f_async) perf_dirty <= 1'b1;
            else if (perf_dirty && perf_idle_cnt[12]) begin
                perf_dirty <= 1'b0;
                $display("%s: TCU-SEQ perf: busy_window=%0d tgemms=%0d gemm_k=%0d mac_beats=%0d slot_wait=%0d drain_beats=%0d ldx=%0d ldw=%0d tacc_st=%0d ldws=%0d ldws_beats=%0d land_wait=%0d early=%0d wait_x=%0d wait_w=%0d wait_a=%0d",
                    INSTANCE_ID, perf_win_cyc, perf_gemms, perf_gemm_k, perf_busy, perf_slot_wait, perf_drain, perf_ldx, perf_ldw, perf_st, perf_lws, perf_lws_beats, perf_land_wait, perf_early, perf_wait_x, perf_wait_w, perf_wait_a);
            end
        end
    end
    final begin
        if (perf_gemms != 0 || perf_ldx != 0) begin
            $display("%s: TCU-SEQ perf: tgemms=%0d gemm_k=%0d mac_beats=%0d slot_wait=%0d drain_beats=%0d ldx=%0d ldw=%0d tacc_st=%0d ldws=%0d ldws_beats=%0d",
                INSTANCE_ID, perf_gemms, perf_gemm_k, perf_busy, perf_slot_wait, perf_drain, perf_ldx, perf_ldw, perf_st, perf_lws, perf_lws_beats);
        end
    end
`endif

endmodule

`endif // VX_CFG_TCU_SEQ_ENABLE
