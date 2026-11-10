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

// TSU burst request issuer & in-flight tracker (the stream-path counterpart of
// VX_dxa_gmem_req).
//
// The address generator runs at BURST_SIZE granularity: each token is one
// BURST_SIZE-aligned global-memory granule with the valid byte span
// [byte_offset, byte_offset + valid_length) of the row inside it. This module
// allocates a slot per granule, turns the span into the set of BEAT_SIZE beats
// that are wanted (`beaten`, a contiguous run) and issues ONE stream request
// for them. Responses arrive one beat at a time and are handed to smem_wr as
// BEAT_SIZE "cache lines": the per-beat byte window, LMEM destination and the
// tiled-scatter N base are recomputed from the slot and the beat index, so
// smem_wr keeps its 64 B datapath and sees the same stream of CLs it would
// have seen from the line engine, just in 256 B transactions on the memory
// side. A slot is released after smem_wr has drained every requested beat.
//
// OOB granules (tile padding) issue no memory request: their beats are
// presented from the slot table (cfill is applied by smem_wr).

`include "VX_define.vh"

module VX_tsu_burst_req import VX_gpu_pkg::*, VX_dxa_pkg::*; #(
    parameter MAX_OUTSTANDING = 64,
    parameter BURST_SIZE      = 256,
    parameter BEAT_SIZE       = 64,
    parameter GMEM_ADDR_WIDTH = `VX_CFG_MEM_ADDR_WIDTH - `CLOG2(BURST_SIZE),
    parameter STR_TAG_WIDTH   = UUID_WIDTH + `CLOG2(MAX_OUTSTANDING),
    parameter SMEM_ADDR_W     = DXA_SMEM_ADDR_W,
    localparam BURST_OFF_BITS = `CLOG2(BURST_SIZE),
    localparam BEAT_OFF_BITS  = `CLOG2(BEAT_SIZE),
    localparam NUM_BEATS      = BURST_SIZE / BEAT_SIZE,
    localparam BEAT_W         = `UP(`CLOG2(NUM_BEATS)),
    localparam TAG_W          = `CLOG2(MAX_OUTSTANDING),
    localparam SEQ_W          = `CLOG2(MAX_OUTSTANDING + 1),
    localparam BEAT_DATAW     = BEAT_SIZE * 8
) (
    input  wire                        clk,
    input  wire                        reset,
    input  wire                        transfer_active,
    input  wire                        transfer_start,

    input  wire [UUID_WIDTH-1:0]       active_uuid,
    input  wire [3:0]                  elem_bytes,   // stable per transfer (tiled-scatter N base per beat)

    // From addr_gen (BURST_SIZE granules).
    input  wire                        ag_valid,
    output wire                        ag_ready,
    input  wire [GMEM_ADDR_WIDTH-1:0]  ag_cl_addr,
    input  wire [SMEM_ADDR_W-1:0]      ag_smem_byte_addr,
    input  wire [BURST_OFF_BITS-1:0]   ag_byte_offset,
    input  wire [BURST_OFF_BITS:0]     ag_valid_length,
    input  wire                        ag_oob,
    input  wire                        ag_last,
    input  wire [15:0]                 ag_k_row,
    input  wire [15:0]                 ag_n_base,

    // Stream bus (reads only).
    VX_stream_bus_if.master            str_bus_if,

    // Direct drain channel to smem_wr (BEAT_SIZE lines).
    output wire                        sw_valid,
    input  wire                        sw_ready,
    output wire [TAG_W-1:0]            sw_tag,
    output wire [BEAT_DATAW-1:0]       sw_data,
    output wire [SMEM_ADDR_W-1:0]      sw_smem_byte_addr,
    output wire [BEAT_OFF_BITS-1:0]    sw_byte_offset,
    output wire [BEAT_OFF_BITS:0]      sw_valid_length,
    output wire                        sw_oob,
    output wire                        sw_last,
    output wire [15:0]                 sw_k_row,
    output wire [15:0]                 sw_n_base,
    output wire [SEQ_W-1:0]            sw_outstanding,

    // Resource release (from smem_wr), one per drained beat.
    input  wire                        release_en,
    input  wire [TAG_W-1:0]            release_tag,

    // Progress events.
    output wire                        gmem_req_fire,
    output wire                        stall_no_slot

`ifdef PERF_ENABLE
    ,
    output wire [31:0]                 perf_gmem_reqs,      // bursts issued
    output wire [31:0]                 perf_gmem_span_cycles,
    output wire [31:0]                 perf_bytes           // beats issued x BEAT_SIZE
`endif
);
    `STATIC_ASSERT(STR_TAG_WIDTH >= UUID_WIDTH + TAG_W, ("stream tag too narrow for slot encoding"))
    `STATIC_ASSERT(`IS_POW2(NUM_BEATS) && NUM_BEATS >= 1, ("BURST_SIZE must be a power-of-two multiple of BEAT_SIZE"))

    localparam SPAN_W = BURST_OFF_BITS + 1;   // byte positions 0..BURST_SIZE inclusive
    localparam CNT_W  = `CLOG2(NUM_BEATS + 1);

    // ════════════════════════════════════════════════════════════════════
    // Slot table.
    // ════════════════════════════════════════════════════════════════════
    typedef struct packed {
        logic [SMEM_ADDR_W-1:0]    smem_byte_addr;  // LMEM destination of the span's first byte
        logic [BURST_OFF_BITS-1:0] byte_offset;     // span start inside the granule
        logic [BURST_OFF_BITS:0]   valid_length;    // span length (bytes)
        logic                      oob;
        logic                      last;            // this granule carries the transfer's last byte
        logic [15:0]               k_row;
        logic [15:0]               n_base;          // N index of the span's first element
        logic [BEAT_W-1:0]         last_beat;       // highest requested beat
        logic [CNT_W-1:0]          nbeats;          // number of requested beats
    } slot_t;

    slot_t slot_table_r [MAX_OUTSTANDING];
    reg [MAX_OUTSTANDING-1:0][CNT_W-1:0] released_r;   // beats drained so far
    reg [MAX_OUTSTANDING-1:0][BEAT_W-1:0] oob_beat_r;  // next OOB beat to present

    reg [MAX_OUTSTANDING-1:0] busy_r;
    reg [MAX_OUTSTANDING-1:0] oob_pending_r;
    reg [SEQ_W-1:0]           outstanding_count_r;

    wire [TAG_W-1:0] free_tag;
    wire             have_free_tag;
    VX_priority_encoder #(
        .N (MAX_OUTSTANDING)
    ) free_pe (
        .data_in   (~busy_r),
        .index_out (free_tag),
        .valid_out (have_free_tag),
        `UNUSED_PIN (onehot_out)
    );

    wire [TAG_W-1:0] oob_tag;
    wire             have_oob;
    VX_priority_encoder #(
        .N (MAX_OUTSTANDING)
    ) oob_pe (
        .data_in   (oob_pending_r),
        .index_out (oob_tag),
        .valid_out (have_oob),
        `UNUSED_PIN (onehot_out)
    );

    // ════════════════════════════════════════════════════════════════════
    // Beat span of the incoming granule: beat b is wanted iff its byte window
    // [b*BEAT, (b+1)*BEAT) overlaps [byte_offset, byte_offset+valid_length).
    // ════════════════════════════════════════════════════════════════════
    wire [SPAN_W-1:0] ag_span_lo = SPAN_W'(ag_byte_offset);
    wire [SPAN_W-1:0] ag_span_hi = SPAN_W'(ag_byte_offset) + ag_valid_length;

    wire [NUM_BEATS-1:0] alloc_beaten;
    for (genvar b = 0; b < NUM_BEATS; ++b) begin : g_beaten
        wire [SPAN_W-1:0] blo = SPAN_W'(b * BEAT_SIZE);
        wire [SPAN_W-1:0] bhi = SPAN_W'((b + 1) * BEAT_SIZE);
        assign alloc_beaten[b] = (ag_span_lo < bhi) && (ag_span_hi > blo);
    end

    reg [BEAT_W-1:0] alloc_last_beat;
    reg [CNT_W-1:0]  alloc_nbeats;
    always @(*) begin
        alloc_last_beat = '0;
        alloc_nbeats    = '0;
        for (int b = 0; b < NUM_BEATS; ++b) begin
            if (alloc_beaten[b]) begin
                alloc_last_beat = BEAT_W'(b);
                alloc_nbeats    = alloc_nbeats + CNT_W'(1);
            end
        end
    end

    // ════════════════════════════════════════════════════════════════════
    // Issue (one stream request per granule).
    // ════════════════════════════════════════════════════════════════════
    wire req_ready_w;
    wire can_alloc   = have_free_tag;
    wire normal_fire = ag_valid && ~ag_oob && can_alloc && req_ready_w;
    wire oob_fire    = ag_valid &&  ag_oob && can_alloc;
    wire accept      = normal_fire || oob_fire;
    wire [TAG_W-1:0] alloc_tag = free_tag;

    assign ag_ready      = accept;
    assign gmem_req_fire = normal_fire;
    assign stall_no_slot = ag_valid && ~ag_oob && ~have_free_tag;

    // Registered producer boundary towards the stream switch / AXI bridge.
    localparam REQ_DATAW = GMEM_ADDR_WIDTH + NUM_BEATS + STR_TAG_WIDTH;
    wire [REQ_DATAW-1:0] req_data_w = {ag_cl_addr, alloc_beaten, STR_TAG_WIDTH'({active_uuid, alloc_tag})};
    wire [REQ_DATAW-1:0] req_data_q;
    VX_elastic_buffer #(
        .DATAW (REQ_DATAW),
        .SIZE  (2),
        .OUT_REG (1)
    ) req_buf (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (ag_valid && ~ag_oob && can_alloc),
        .ready_in  (req_ready_w),
        .data_in   (req_data_w),
        .valid_out (str_bus_if.req_valid),
        .ready_out (str_bus_if.req_ready),
        .data_out  (req_data_q)
    );
    assign {str_bus_if.req_data.addr, str_bus_if.req_data.beaten, str_bus_if.req_data.tag} = req_data_q;

    // ════════════════════════════════════════════════════════════════════
    // Presentation to smem_wr: a bus beat when one is pending, otherwise the
    // next beat of an OOB granule.
    // ════════════════════════════════════════════════════════════════════
    wire             bus_rsp_present = str_bus_if.rsp_valid;
    wire [TAG_W-1:0] bus_rsp_tag     = TAG_W'(str_bus_if.rsp_data.tag);
    wire [BEAT_W-1:0] bus_rsp_beat   = str_bus_if.rsp_data.beat;
    wire             present_oob     = ~bus_rsp_present && have_oob;

    wire [TAG_W-1:0]  present_tag  = bus_rsp_present ? bus_rsp_tag  : oob_tag;
    wire [BEAT_W-1:0] present_beat = bus_rsp_present ? bus_rsp_beat : oob_beat_r[oob_tag];
    slot_t ps;
    assign ps = slot_table_r[present_tag];

    // Per-beat byte window inside the granule, clipped to the valid span.
    wire [SPAN_W-1:0] p_lo0  = SPAN_W'(ps.byte_offset);
    wire [SPAN_W-1:0] p_hi0  = SPAN_W'(ps.byte_offset) + ps.valid_length;
    wire [SPAN_W-1:0] b_lo   = SPAN_W'(present_beat) << BEAT_OFF_BITS;
    wire [SPAN_W-1:0] b_hi   = b_lo + SPAN_W'(BEAT_SIZE);
    wire [SPAN_W-1:0] w_lo   = (p_lo0 > b_lo) ? p_lo0 : b_lo;
    wire [SPAN_W-1:0] w_hi   = (p_hi0 < b_hi) ? p_hi0 : b_hi;
    wire [SPAN_W-1:0] w_len  = w_hi - w_lo;
    wire [SPAN_W-1:0] w_skip = w_lo - p_lo0;   // valid bytes that precede this beat

    // log2(elem_bytes) for the tiled-scatter N base.
    wire [1:0] lg_elem = (elem_bytes == 4'd8) ? 2'd3 : (elem_bytes == 4'd4) ? 2'd2 : (elem_bytes == 4'd2) ? 2'd1 : 2'd0;

    assign sw_valid          = bus_rsp_present || present_oob;
    assign sw_tag            = present_tag;
    assign sw_data           = str_bus_if.rsp_data.data;   // don't-care when sw_oob
    assign sw_smem_byte_addr = ps.smem_byte_addr + SMEM_ADDR_W'(w_skip);
    assign sw_byte_offset    = w_lo[BEAT_OFF_BITS-1:0];
    assign sw_valid_length   = w_len[BEAT_OFF_BITS:0];
    assign sw_oob            = ps.oob;
    assign sw_last           = ps.last && (present_beat == ps.last_beat);
    assign sw_k_row          = ps.k_row;
    assign sw_n_base         = ps.n_base + 16'(w_skip >> lg_elem);
    assign sw_outstanding    = outstanding_count_r;

    assign str_bus_if.rsp_ready = sw_ready && bus_rsp_present;

    // ════════════════════════════════════════════════════════════════════
    // Sequential update.
    // ════════════════════════════════════════════════════════════════════
    wire oob_present_fire = present_oob && sw_ready;
    wire oob_slot_done    = oob_present_fire && (oob_beat_r[oob_tag] == slot_table_r[oob_tag].last_beat);

    // A slot frees when its last requested beat has been drained.
    wire release_last = release_en &&
        ((released_r[release_tag] + CNT_W'(1)) == slot_table_r[release_tag].nbeats);

    wire [MAX_OUTSTANDING-1:0] busy_set        = accept       ? (MAX_OUTSTANDING'(1) << alloc_tag)   : '0;
    wire [MAX_OUTSTANDING-1:0] busy_clr        = release_last ? (MAX_OUTSTANDING'(1) << release_tag) : '0;
    wire [MAX_OUTSTANDING-1:0] oob_pending_set = oob_fire     ? (MAX_OUTSTANDING'(1) << alloc_tag)   : '0;
    wire [MAX_OUTSTANDING-1:0] oob_pending_clr = oob_slot_done ? (MAX_OUTSTANDING'(1) << oob_tag)    : '0;

    always @(posedge clk) begin
        if (reset) begin
            busy_r              <= '0;
            oob_pending_r       <= '0;
            outstanding_count_r <= '0;
        end else begin
            if (accept) begin
                slot_table_r[alloc_tag].smem_byte_addr <= ag_smem_byte_addr;
                slot_table_r[alloc_tag].byte_offset    <= ag_byte_offset;
                slot_table_r[alloc_tag].valid_length   <= ag_valid_length;
                slot_table_r[alloc_tag].oob            <= ag_oob;
                slot_table_r[alloc_tag].last           <= ag_last;
                slot_table_r[alloc_tag].k_row          <= ag_k_row;
                slot_table_r[alloc_tag].n_base         <= ag_n_base;
                slot_table_r[alloc_tag].last_beat      <= alloc_last_beat;
                slot_table_r[alloc_tag].nbeats         <= alloc_nbeats;
                released_r[alloc_tag]                  <= '0;
                oob_beat_r[alloc_tag]                  <= '0;
            end
            if (release_en) begin
                released_r[release_tag] <= released_r[release_tag] + CNT_W'(1);
            end
            if (oob_present_fire && ~oob_slot_done) begin
                oob_beat_r[oob_tag] <= oob_beat_r[oob_tag] + BEAT_W'(1);
            end

            busy_r        <= (busy_r        | busy_set       ) & ~busy_clr;
            oob_pending_r <= (oob_pending_r | oob_pending_set) & ~oob_pending_clr;

            case ({accept, release_last})
                2'b10: outstanding_count_r <= outstanding_count_r + SEQ_W'(1);
                2'b01: outstanding_count_r <= outstanding_count_r - SEQ_W'(1);
                default: ;
            endcase
        end
    end

`ifdef PERF_ENABLE
    reg [31:0] rdp_total_req_r;
    reg [31:0] rdp_total_bytes_r;
    reg [31:0] rdp_cycle_ctr_r;
    reg [31:0] rdp_first_req_cycle_r;
    reg [31:0] rdp_last_rsp_cycle_r;
    reg        rdp_has_req_r;

    always @(posedge clk) begin
        if (reset || transfer_start) begin
            rdp_total_req_r       <= '0;
            rdp_total_bytes_r     <= '0;
            rdp_cycle_ctr_r       <= '0;
            rdp_first_req_cycle_r <= '0;
            rdp_last_rsp_cycle_r  <= '0;
            rdp_has_req_r         <= 1'b0;
        end else begin
            rdp_cycle_ctr_r <= rdp_cycle_ctr_r + 32'd1;
            if (normal_fire) begin
                rdp_total_req_r   <= rdp_total_req_r + 32'd1;
                rdp_total_bytes_r <= rdp_total_bytes_r + (32'(alloc_nbeats) << BEAT_OFF_BITS);
                if (!rdp_has_req_r) begin
                    rdp_first_req_cycle_r <= rdp_cycle_ctr_r;
                    rdp_has_req_r <= 1'b1;
                end
            end
            if (release_en) begin
                rdp_last_rsp_cycle_r <= rdp_cycle_ctr_r;
            end
        end
    end

    assign perf_gmem_reqs        = rdp_total_req_r;
    assign perf_bytes            = rdp_total_bytes_r;
    assign perf_gmem_span_cycles = (rdp_has_req_r && rdp_last_rsp_cycle_r >= rdp_first_req_cycle_r)
                                 ? (rdp_last_rsp_cycle_r - rdp_first_req_cycle_r + 32'd1) : 32'd0;
`endif

`ifdef DBG_TRACE_DXA
    always @(posedge clk) begin
        if (~reset && transfer_active) begin
            if (normal_fire) begin
                $write("TSU_PIPE,%0d,BURST_REQ,addr=0x%0h,tag=%0d,beaten=0x%0h,smem=0x%0h\n",
                    $time, ag_cl_addr, alloc_tag, alloc_beaten, ag_smem_byte_addr);
            end
            if (sw_valid && sw_ready) begin
                $write("TSU_PIPE,%0d,BEAT,tag=%0d,beat=%0d,oob=%0d,last=%0d,smem=0x%0h,len=%0d\n",
                    $time, present_tag, present_beat, sw_oob, sw_last, sw_smem_byte_addr, sw_valid_length);
            end
        end
    end
`endif

    `UNUSED_VAR (transfer_active)
    `UNUSED_VAR (transfer_start)
    `UNUSED_VAR (w_len[SPAN_W-1:BEAT_OFF_BITS+1])
    `UNUSED_VAR (w_lo[SPAN_W-1:BEAT_OFF_BITS])

endmodule
