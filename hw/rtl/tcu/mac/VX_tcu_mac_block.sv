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

`include "VX_tcu_mac_defs.vh"

// 512-MAC int8 tensor block (balanced-design spec phase 3, track B; DSP packing option (a)).
//
// Computes one ROWS x COLS int32 output tile  D[r][c] = sum_k A[r][k] * W[c][k]
// over an accumulation span of K k-steps delimited by in_first .. in_last, one
// k-step per accepted beat:  ROWS activations (A column at k) and COLS weights
// (W row at k). Default ROWS 8 x COLS 64 = 512 MAC/cycle on ROWS/2 x COLS = 256
// DSP48E2 (2 MACs per DSP). Exact int32 (wrapping) for int8 x int8 (in_signed=1)
// and uint8 x uint8 (in_signed=0); bit-exact with the TFR / DSP_I8 int8 paths.
//
// Packing (as VX_tcu_fedp_dsp_i8, phase 2): DSP(p, c) holds rows 2p and 2p+1 of
// column c.  A = a_{2p+1} * 2^18 + a_{2p} (27 bits, the DSP48E2 multiplier input),
// B = w_c (18 bits). The DSP accumulates a packed GROUP of at most 4 k-steps,
// P = S1 * 2^18 + S0, which keeps the fields exact; the fields are extracted
//     S0 = sext(P[17:0])  (zext when unsigned)
//     S1 = (P >>> 18) + P[17]  (borrow correction; none when unsigned)
// and added into int32 fabric accumulators. Groups are k-steps 0-3, 4-7, ... of a
// span; the last group of a span may be shorter.
//
// Extraction is time-shared: one extraction unit per 4 DSPs (one row pair x 4
// adjacent columns). A group end captures the 4 P values into registers; the unit
// reads them through one 4:1 mux over 4 cycles and rotates a 4-deep accumulator
// chain in lockstep. Consecutive group ends must be
// >= 4 cycles apart (true for full groups; in_ready inserts bubbles after short
// last groups).
//
// Working accumulators: 2 x 4 int32 per unit in FFs = ROWS x COLS int32 (16 kbit
// at 8x64). FFs (not RAMB18/LUTRAM) because every unit read-modify-writes 2 words
// per cycle in a rotating pattern: a RAM would need 128 write ports per cycle.
// Final values of a span go to a shadow tile (LUTRAM, 4 x 64 b per unit) that is
// drained through the output port while the next span accumulates, so a span of
// K >= ~(BEATS + 16) k-steps never stalls on the drain.
//
// Ports / timing:
//   in_valid/in_ready : one k-step per beat. in_first starts a span (the first
//                       group overwrites the accumulators), in_last ends it,
//                       in_signed selects int8 (1) vs uint8 (0) for the span
//                       (sample it on in_first; it must stay constant in a span).
//                       in_ready does not depend on in_valid; it depends on
//                       in_first/in_last of the presented beat (it drops for a
//                       beat that would end a group < 4 cycles after the previous
//                       group end, or end a span while the shadow tile is still
//                       draining).
//   in_a[r]           : activation row r at this k (int8 / uint8), ROWS x 8 bits.
//   in_b[c]           : weight column c at this k, COLS x 8 bits.
//   out_valid/ready   : drained tile, BEATS = COLS / OUT_COLS beats per span.
//                       Beat t carries columns  j * (COLS/OUT_COLS) + t  for
//                       j = 0 .. OUT_COLS-1, all ROWS rows:
//                       out_data[j][r] = D[r][j*(COLS/OUT_COLS) + t];
//                       out_col = t, out_last on t = BEATS-1.
//   Latency: last k-step accepted -> first output beat ~ 14 clocks.
// RMW / URAM folding: a sequencer that splits K into k-tiles either issues one
// span per K (the block accumulates internally) or folds each drained span into
// its accumulator tile (D_tile += drained beat) at OUT_COLS x ROWS int32 per beat.
module VX_tcu_mac_block #(
    parameter ROWS     = 8,     // batch rows (even)
    parameter COLS     = 64,    // weight columns (multiple of 4 * OUT_COLS)
    parameter OUT_COLS = 2      // columns per output beat
) (
    input  wire                         clk,
    input  wire                         reset,

    input  wire                         in_valid,
    output wire                         in_ready,
    input  wire                         in_first,
    input  wire                         in_last,
    input  wire                         in_signed,
    input  wire [ROWS-1:0][7:0]         in_a,
    input  wire [COLS-1:0][7:0]         in_b,

    output wire                         out_valid,
    input  wire                         out_ready,
    output wire [OUT_COLS-1:0][ROWS-1:0][31:0] out_data,
    output wire [`CLOG2(COLS/OUT_COLS)-1:0] out_col,
    output wire                         out_last
);
    localparam NP      = ROWS / 2;            // row pairs
    localparam NU      = COLS / 4;            // extraction units per row pair
    localparam NREP    = (COLS + 7) / 8;      // A broadcast replicas (fanout 8 each)
    localparam BEATS   = COLS / OUT_COLS;
    localparam UPL     = NU / OUT_COLS;       // units per output lane
    localparam BEATW   = `CLOG2(BEATS);
    localparam SEP     = 18;

    `STATIC_ASSERT ((ROWS % 2) == 0, ("ROWS must be even"))
    `STATIC_ASSERT ((COLS % (4 * OUT_COLS)) == 0, ("COLS must be a multiple of 4*OUT_COLS"))

    // ------------------------------------------------------------------
    // Input acceptance: group / span bookkeeping and flow control
    // ------------------------------------------------------------------
    logic [1:0] kcnt_r;          // index of the next k-step inside its group
    logic [2:0] since_ge_r;      // cycles since the last accepted group end (sat. 4)
    logic       shadow_resv_r;   // a span end is in flight or its tile is draining
    logic       fg_r;            // the open group is the first group of its span

    wire [1:0] idx       = in_first ? 2'd0 : kcnt_r;
    wire       fg        = in_first || fg_r;
    wire       beat_ge   = in_last || (idx == 2'd3);
    wire       ge_ok     = (since_ge_r >= 3'd4);
    wire       last_ok   = ~shadow_resv_r;
    assign in_ready      = (~beat_ge || ge_ok) && (~in_last || last_ok);
    wire       accept    = in_valid && in_ready;

    logic shadow_release;
    always_ff @(posedge clk) begin
        if (reset) begin
            kcnt_r        <= '0;
            since_ge_r    <= 3'd4;
            shadow_resv_r <= 1'b0;
            fg_r          <= 1'b0;
        end else begin
            if (accept) begin
                kcnt_r <= beat_ge ? 2'd0 : (idx + 2'd1);
                fg_r   <= beat_ge ? 1'b0 : fg;
            end
            if (accept && beat_ge) begin
                since_ge_r <= 3'd1;
            end else if (since_ge_r != 3'd4) begin
                since_ge_r <= since_ge_r + 3'd1;
            end
            if (accept && in_last) begin
                shadow_resv_r <= 1'b1;
            end else if (shadow_release) begin
                shadow_resv_r <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------
    // S0: input register
    // ------------------------------------------------------------------
    // control word: {valid, gs, ge, first, last, signed}
    typedef struct packed {
        logic v;
        logic gs;
        logic ge;
        logic first;
        logic last;
        logic sgn;
    } ctl_t;

    ctl_t                 ctl0;
    logic [ROWS-1:0][7:0] a0;
    logic [COLS-1:0][7:0] b0;
    logic                 sgn_span_r;  // signedness latched at in_first

    always_ff @(posedge clk) begin
        if (reset) begin
            ctl0.v <= 1'b0;
        end else begin
            ctl0.v <= accept;
        end
        if (accept && in_first) begin
            sgn_span_r <= in_signed;
        end
        ctl0.gs    <= (idx == 2'd0);
        ctl0.ge    <= beat_ge;
        ctl0.first <= fg;            // "this k-step's group is the span's first group"
        ctl0.last  <= in_last;
        ctl0.sgn   <= in_first ? in_signed : sgn_span_r;
        a0 <= in_a;
        b0 <= in_b;
    end

    // ------------------------------------------------------------------
    // S1: pack A pairs, extend B
    // ------------------------------------------------------------------
    ctl_t                    ctl1;
    logic signed [26:0]      apk1 [NP];
    logic signed [17:0]      b1   [COLS];

    for (genvar p = 0; p < NP; ++p) begin : g_pack
        wire signed [8:0] s_a0 = ctl0.sgn ? $signed({a0[2*p][7],   a0[2*p]})   : $signed({1'b0, a0[2*p]});
        wire signed [8:0] s_a1 = ctl0.sgn ? $signed({a0[2*p+1][7], a0[2*p+1]}) : $signed({1'b0, a0[2*p+1]});
        // A = a1 * 2^18 + a0: low 18 bits = sext(a0), high 9 bits = a1 - (a0 < 0)
        wire [SEP-1:0]    a_lo = {{(SEP-9){s_a0[8]}}, s_a0};
        wire [27-SEP-1:0] a_hi = (27-SEP)'(s_a1) - (27-SEP)'({{(27-SEP-1){1'b0}}, s_a0[8]});
        always_ff @(posedge clk) begin
            apk1[p] <= $signed({a_hi, a_lo});
        end
    end
    for (genvar c = 0; c < COLS; ++c) begin : g_bext
        always_ff @(posedge clk) begin
            b1[c] <= ctl0.sgn ? $signed({{10{b0[c][7]}}, b0[c]}) : $signed({10'b0, b0[c]});
        end
    end

    // ------------------------------------------------------------------
    // S2: A broadcast replication (each replica feeds 8 columns x NP DSPs)
    // ------------------------------------------------------------------
    ctl_t ctl2;
    (* equivalent_register_removal = "no" *) logic signed [26:0] apk2 [NREP][NP];
    logic signed [17:0] b2 [COLS];

    for (genvar r = 0; r < NREP; ++r) begin : g_arep
        for (genvar p = 0; p < NP; ++p) begin : g_p
            always_ff @(posedge clk) begin
                apk2[r][p] <= apk1[p];
            end
        end
    end
    for (genvar c = 0; c < COLS; ++c) begin : g_brep
        always_ff @(posedge clk) begin
            b2[c] <= b1[c];
        end
    end

    // ------------------------------------------------------------------
    // Control pipeline S0 -> S6 (aligned with the DSP M stage at S5)
    // ------------------------------------------------------------------
    ctl_t ctl3, ctl4;
    /* verilator lint_off UNUSEDSIGNAL */
    ctl_t ctl6;                          // ctl6.gs is unused (P restart already applied at S5)
    /* verilator lint_on UNUSEDSIGNAL */
    (* max_fanout = 32 *) ctl_t ctl5;

    always_ff @(posedge clk) begin
        if (reset) begin
            ctl1.v <= 1'b0; ctl2.v <= 1'b0; ctl3.v <= 1'b0;
            ctl4.v <= 1'b0; ctl5.v <= 1'b0; ctl6.v <= 1'b0;
        end else begin
            ctl1.v <= ctl0.v; ctl2.v <= ctl1.v; ctl3.v <= ctl2.v;
            ctl4.v <= ctl3.v; ctl5.v <= ctl4.v; ctl6.v <= ctl5.v;
        end
        {ctl1.gs, ctl1.ge, ctl1.first, ctl1.last, ctl1.sgn} <= {ctl0.gs, ctl0.ge, ctl0.first, ctl0.last, ctl0.sgn};
        {ctl2.gs, ctl2.ge, ctl2.first, ctl2.last, ctl2.sgn} <= {ctl1.gs, ctl1.ge, ctl1.first, ctl1.last, ctl1.sgn};
        {ctl3.gs, ctl3.ge, ctl3.first, ctl3.last, ctl3.sgn} <= {ctl2.gs, ctl2.ge, ctl2.first, ctl2.last, ctl2.sgn};
        {ctl4.gs, ctl4.ge, ctl4.first, ctl4.last, ctl4.sgn} <= {ctl3.gs, ctl3.ge, ctl3.first, ctl3.last, ctl3.sgn};
        {ctl5.gs, ctl5.ge, ctl5.first, ctl5.last, ctl5.sgn} <= {ctl4.gs, ctl4.ge, ctl4.first, ctl4.last, ctl4.sgn};
        {ctl6.gs, ctl6.ge, ctl6.first, ctl6.last, ctl6.sgn} <= {ctl5.gs, ctl5.ge, ctl5.first, ctl5.last, ctl5.sgn};
    end

    // ------------------------------------------------------------------
    // S3..S6: DSP MACC array (S3 A1/B1, S4 A2/B2, S5 M, S6 P)
    // ------------------------------------------------------------------
    wire signed [47:0] pv [NP][COLS];
    for (genvar p = 0; p < NP; ++p) begin : g_dsp_p
        for (genvar c = 0; c < COLS; ++c) begin : g_dsp_c
            VX_tcu_mac_dsp dsp (
                .clk (clk),
                .a   (apk2[c / 8][p]),
                .b   (b2[c]),
                .vld (ctl5.v),
                .gs  (ctl5.gs),
                .p   (pv[p][c])
            );
        end
    end

    // group end visible on P (ctl6): load the capture chains at the next edge
    (* max_fanout = 32 *) wire cap_ld = ctl6.v && ctl6.ge;

    // ------------------------------------------------------------------
    // Extraction units: capture chain -> field extraction (X) -> accumulators
    // One global copy of the (identical) unit control, replicated by max_fanout.
    // ------------------------------------------------------------------
    logic [2:0] proc_cnt_r;                       // heads left to process (0..4)
    logic       chain_first_r, chain_last_r, chain_sgn_r;
    (* max_fanout = 32 *) wire proc = (proc_cnt_r != 3'd0);
    (* max_fanout = 32 *) logic [1:0] slot_r;     // index of the head being processed (capture mux select)

    // X stage (extracted fields registered)
    (* max_fanout = 32 *) logic x_v;
    logic       x_first, x_last;
    logic [1:0] x_slot;

    always_ff @(posedge clk) begin
        if (reset) begin
            proc_cnt_r <= '0;
            x_v        <= 1'b0;
        end else begin
            if (cap_ld) begin
                proc_cnt_r <= 3'd4;
            end else if (proc) begin
                proc_cnt_r <= proc_cnt_r - 3'd1;
            end
            x_v <= proc;
        end
        if (cap_ld) begin
            chain_first_r <= ctl6.first;
            chain_last_r  <= ctl6.last;
            chain_sgn_r   <= ctl6.sgn;
            slot_r        <= 2'd0;
        end else if (proc) begin
            slot_r <= slot_r + 2'd1;
        end
        x_first <= chain_first_r;
        x_last  <= chain_last_r && proc;
        x_slot  <= slot_r;
    end

    // capture-chain load must not collide with an unfinished chain
    `RUNTIME_ASSERT (~(cap_ld && (proc_cnt_r > 3'd1)), ("VX_tcu_mac_block: group ends < 4 cycles apart"))

    // per-unit datapath
    logic [63:0] shadow_rd [NP][NU];     // async LUTRAM read at the drain entry index
    logic [1:0]  drain_ent;              // entry (column within unit) being drained

    for (genvar p = 0; p < NP; ++p) begin : g_unit_p
        for (genvar u = 0; u < NU; ++u) begin : g_unit
            // capture registers: 4 x 48-bit P values (columns 4u .. 4u+3), loaded together (FF CE only)
            // and read through ONE 4:1 mux (1 LUT6 per bit) -- a load-or-shift chain would cost a 2:1
            // mux per bit per stage (3 x 48 LUTs per unit; measured 9.2k LUT at 8x64).
            logic signed [47:0] cap_r [4];
            always_ff @(posedge clk) begin
                if (cap_ld) begin
                    for (int s = 0; s < 4; ++s) cap_r[s] <= pv[p][4*u + s];
                end
            end
            wire signed [47:0] cap_head = cap_r[slot_r];

            // X: field extraction of the head
            logic [SEP-1:0]  f_lo;
            logic [29:0]     f_hi;
            logic [31:0]     xs0, xs1;
            always_comb begin
                f_lo = cap_head[SEP-1:0];
                f_hi = cap_head[47:SEP];
            end
            always_ff @(posedge clk) begin
                xs0 <= chain_sgn_r ? 32'($signed(f_lo)) : 32'(f_lo);
                xs1 <= 32'($signed(f_hi)) + (chain_sgn_r ? 32'(f_lo[SEP-1]) : 32'd0);
            end

            // accumulator chains (rotate in lockstep with the processed head)
            logic [31:0] acc0_r [4];
            logic [31:0] acc1_r [4];
            wire  [31:0] new0 = x_first ? xs0 : (acc0_r[0] + xs0);
            wire  [31:0] new1 = x_first ? xs1 : (acc1_r[0] + xs1);
            always_ff @(posedge clk) begin
                if (x_v) begin
                    for (int s = 0; s < 3; ++s) begin
                        acc0_r[s] <= acc0_r[s+1];
                        acc1_r[s] <= acc1_r[s+1];
                    end
                    acc0_r[3] <= new0;
                    acc1_r[3] <= new1;
                end
            end

            // shadow tile entry per column of the unit: {row 2p+1, row 2p}
            `USE_FAST_BRAM logic [63:0] shadow_r [4];
            always_ff @(posedge clk) begin
                if (x_last) begin
                    shadow_r[x_slot] <= {new1, new0};
                end
            end
            assign shadow_rd[p][u] = shadow_r[drain_ent];
        end
    end

    // ------------------------------------------------------------------
    // Drain: shadow tile -> output beats
    // ------------------------------------------------------------------
    logic             shadow_full_r;   // tile complete, draining
    logic [BEATW-1:0] beat_r;
    logic             out_v_r;
    logic [OUT_COLS-1:0][ROWS-1:0][31:0] out_d_r;
    logic [BEATW-1:0] out_col_r;
    logic             out_last_r;

    // the last X write of a span: x_last on slot 3
    wire tile_done = x_last && (x_slot == 2'd3);

    wire load_beat = shadow_full_r && (~out_v_r || out_ready);
    assign drain_ent = beat_r[1:0];
    wire [BEATW-1:0] unit_in_lane = beat_r >> 2;

    always_ff @(posedge clk) begin
        if (reset) begin
            shadow_full_r <= 1'b0;
            beat_r        <= '0;
            out_v_r       <= 1'b0;
        end else begin
            if (tile_done) begin
                shadow_full_r <= 1'b1;
                beat_r        <= '0;
            end else if (load_beat) begin
                beat_r <= beat_r + BEATW'(1);
                if (beat_r == BEATW'(BEATS - 1)) begin
                    shadow_full_r <= 1'b0;
                end
            end
            if (load_beat) begin
                out_v_r <= 1'b1;
            end else if (out_ready) begin
                out_v_r <= 1'b0;
            end
        end
        if (load_beat) begin
            for (int j = 0; j < OUT_COLS; ++j) begin
                for (int p = 0; p < NP; ++p) begin
                    out_d_r[j][2*p]   <= shadow_rd[p][j*UPL + int'(unit_in_lane)][31:0];
                    out_d_r[j][2*p+1] <= shadow_rd[p][j*UPL + int'(unit_in_lane)][63:32];
                end
            end
            out_col_r  <= beat_r;
            out_last_r <= (beat_r == BEATW'(BEATS - 1));
        end
    end

    assign shadow_release = load_beat && (beat_r == BEATW'(BEATS - 1));

    `RUNTIME_ASSERT (~(tile_done && shadow_full_r), ("VX_tcu_mac_block: shadow overwritten while draining"))

    assign out_valid = out_v_r;
    assign out_data  = out_d_r;
    assign out_col   = out_col_r;
    assign out_last  = out_last_r;


endmodule
