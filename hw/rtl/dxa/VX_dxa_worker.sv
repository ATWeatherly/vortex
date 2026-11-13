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

// DXA worker: purely structural — wire declarations and module
// instantiations. All logic lives in the submodules:
//   setup → addr_gen → gmem_req → smem_wr + watchdog

`include "VX_define.vh"

// STREAM=1 turns the worker into a tile-stream (TSU) worker: the address
// generator runs at TSU_BURST_BYTES granules, VX_tsu_burst_req issues one
// stream request per granule over NUM_STR_PORTS stream masters (selected by
// the pseudo-channel address bits) and hands the returning 64 B beats to the
// unchanged smem_wr, which lands them in LMEM and raises the same completion.

module VX_dxa_worker import VX_gpu_pkg::*, VX_dxa_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter WORKER_ID = 0,
    parameter GMEM_TAG_WIDTH = SOCKET_MEM_TAG_WIDTH,
    parameter STREAM        = 0,                    // 1: TSU burst worker (stream bus), 0: DXA line worker (socket memory port)
    parameter NUM_STR_PORTS = 1,                    // stream masters owned by this worker (STREAM=1)
    parameter STR_SEL_LSB   = TSU_SEL_LSB_GRANULE,  // granule-address bit selecting the master
    parameter STR_MASTER_BASE = 0                   // global index of this worker's master 0 (m_axi_str_<i>)
) (
    input wire clk,
    input wire reset,
`ifdef PERF_ENABLE
    output dxa_perf_t dxa_perf,
`endif
    VX_dxa_worker_req_if.slave req_if,
    VX_mem_bus_if.master gmem_bus_if,
    VX_stream_bus_if.master str_bus_if [NUM_STR_PORTS],
`ifdef VX_CFG_TSU_ENABLE
`ifdef VX_CFG_TCU_SEQ_ENABLE
    // Phase-4 W-slot landing (STREAM workers; line workers tie wl_ready low and never drive wl_*)
    output wire                 wl_valid,
    input  wire                 wl_ready,
    input  wire [TSU_WL_GO_W-1:0] wl_go,      // landing start gate per {blk, slot} (sequencer)
    output dxa_wland_beat_t     wl_beat,
    output wire                 wl_done_valid,
    output dxa_wland_done_t     wl_done,
`endif
`endif
    VX_mem_bus_if.master smem_bus_if
);
    `UNUSED_SPARAM (INSTANCE_ID)
    `UNUSED_SPARAM (WORKER_ID)
    `UNUSED_PARAM (STR_MASTER_BASE)   // used only by the VX_CFG_TSU_LINK_GROUP range assert

    // Address-generator granule: an L1 line for the line engine, a burst for
    // the stream engine.
    localparam GMEM_BYTES      = STREAM ? TSU_BURST_BYTES : `VX_CFG_L1_LINE_SIZE;
    localparam GMEM_OFF_BITS   = `CLOG2(GMEM_BYTES);
    // stream workers address the full TSU width (VX_CFG_TSU_ADDR_WIDTH, all HBM PCs); line workers the memory bus
    localparam GMEM_ADDR_WIDTH = (STREAM ? `VX_CFG_TSU_ADDR_WIDTH : `VX_CFG_MEM_ADDR_WIDTH) - GMEM_OFF_BITS;

    // What smem_wr sees per delivery: a whole line, or one burst beat.
    localparam CL_BYTES        = STREAM ? TSU_BEAT_BYTES : GMEM_BYTES;
    localparam CL_DATAW        = CL_BYTES * 8;
    localparam CL_OFF_BITS     = `CLOG2(CL_BYTES);

    localparam SMEM_BYTES      = DXA_LMEM_WORD_SIZE;
    localparam SMEM_ADDR_WIDTH = DXA_LMEM_ADDR_W;

    localparam MAX_OUTSTANDING = STREAM ? TSU_MAX_INFLIGHT : `VX_CFG_DXA_MAX_INFLIGHT;
    localparam TAG_W = `CLOG2(MAX_OUTSTANDING);
    localparam SEQ_W = `CLOG2(MAX_OUTSTANDING + 1);

    // ════════════════════════════════════════════════════════════════════
    // Inter-module wires
    // ════════════════════════════════════════════════════════════════════

    // setup → downstream
    wire                        transfer_active;
    wire                        pipeline_start;
    dxa_setup_params_t          setup_params;
    wire [NC_WIDTH-1:0]         active_core_id;
    wire [UUID_WIDTH-1:0]       active_uuid;
    wire [NW_WIDTH-1:0]         active_wid;
    wire [BAR_ADDR_W-1:0]       active_bar_addr;
    wire                        active_notify_smem_done;
    wire                        active_is_multicast;
    wire [`VX_CFG_NUM_WARPS-1:0]       active_cta_mask;
    wire [31:0]                 active_smem_stride;

    // addr_gen → gmem_req
    wire                        ag_valid;
    wire                        ag_ready;
    wire [GMEM_ADDR_WIDTH-1:0]  ag_cl_addr;
    wire [DXA_SMEM_ADDR_W-1:0]  ag_smem_byte_addr;
    wire [GMEM_OFF_BITS-1:0]    ag_byte_offset;
    wire [GMEM_OFF_BITS:0]      ag_valid_length;
    wire                        ag_oob;
    wire                        ag_last;
    wire [31:0]                 ag_cfill;
    wire                        ag_dest_kmajor;
    wire [15:0]                 ag_per_lane_stride_bytes;
    wire [3:0]                  ag_elem_bytes;
    wire [1:0]                  ag_dest_mode;
    wire [3:0]                  ag_lg_ratio;
    wire [3:0]                  ag_lg_tcN;
    wire [3:0]                  ag_lg_nsteps;
    wire [3:0]                  ag_lg_bkK;
    wire [15:0]                 ag_k_row;
    wire [15:0]                 ag_n_base;
    wire [DXA_SMEM_ADDR_W-1:0]  ag_smem_base;

    // gmem_req / burst_req → smem_wr (direct-drain channel, CL_BYTES per delivery).
    wire                        sw_valid;
    wire                        sw_ready;
    wire [TAG_W-1:0]            sw_tag;
    wire [CL_DATAW-1:0]         sw_data;
    wire [DXA_SMEM_ADDR_W-1:0]  sw_smem_byte_addr;
    wire [CL_OFF_BITS-1:0]      sw_byte_offset;
    wire [CL_OFF_BITS:0]        sw_valid_length;
    wire                        sw_oob;
    wire                        sw_last;
    wire [15:0]                 sw_k_row;
    wire [15:0]                 sw_n_base;
    wire [SEQ_W-1:0]            sw_outstanding;
    wire                        perf_wl_take;    // a landing beat left the burst tracker toward the sequencer
    wire                        perf_wl_wait;    // a landing beat is presented but cannot leave
`ifndef PERF_ENABLE
    `UNUSED_VAR ({perf_wl_take, perf_wl_wait})
`endif

    // smem_wr → gmem_req (release)
    wire                        sw_release_en;
    wire [TAG_W-1:0]            sw_release_tag;

    // smem_wr → setup (completion)
    wire                        transfer_done;
    wire [31:0]                 wr_done_count;
    wire                        smem_req_fire;

    // gmem_req → watchdog
    wire                        gmem_req_fire;
    wire                        stall_no_slot;

`ifdef PERF_ENABLE
    wire [31:0]                 perf_gmem_reqs;
    wire [31:0]                 perf_gmem_span_cycles;
    wire [31:0]                 perf_lmem_writes;
    wire [31:0]                 perf_str_bytes;   // STREAM: bytes requested by the transfer's bursts
`endif

    // ════════════════════════════════════════════════════════════════════
    // Stage 1: Setup
    // ════════════════════════════════════════════════════════════════════

    VX_dxa_setup setup (
        .clk                  (clk),
        .reset                (reset),
        .req_valid            (req_if.valid),
        .req_ready            (req_if.ready),
        .req_data             (req_if.req_data),
        .desc_data            (req_if.desc_data),
        .transfer_done        (transfer_done),
        .transfer_active      (transfer_active),
        .pipeline_start       (pipeline_start),
        .setup_params         (setup_params),
        .active_core_id       (active_core_id),
        .active_uuid          (active_uuid),
        .active_wid           (active_wid),
        .active_bar_addr      (active_bar_addr),
        .active_notify_smem_done (active_notify_smem_done),
        .active_is_multicast  (active_is_multicast),
        .active_cta_mask      (active_cta_mask),
        .active_smem_stride   (active_smem_stride)
    );

    // ════════════════════════════════════════════════════════════════════
    // Stage 2: Address Generator
    // ════════════════════════════════════════════════════════════════════

    // W-slot landing start gate: a TILEBUF_B transfer issues no read until the sequencer reports that its slot's
    // expectation is queued and free to write (wl_go), so its beats never wait inside the TSU and its stream
    // responses never back-pressure the memory system (silicon u50p5_1c100: that back-pressure blocked the core's
    // own X loads, which the expectation was waiting behind -> deadlock).
    wire ag_valid_raw, ag_ready_raw;
`ifdef VX_CFG_TSU_ENABLE
`ifdef VX_CFG_TCU_SEQ_ENABLE
    wire ag_go = ~setup_params.wl_en || wl_go[setup_params.wl_tag[3:0]];
`else
    wire ag_go = 1'b1;
`endif
`else
    wire ag_go = 1'b1;
`endif
    assign ag_valid     = ag_valid_raw && ag_go;
    assign ag_ready_raw = ag_ready && ag_go;

    VX_dxa_addr_gen #(
        .GMEM_LINE_SIZE  (GMEM_BYTES),
        .GMEM_ADDR_WIDTH (GMEM_ADDR_WIDTH)
    ) addr_gen (
        .clk                  (clk),
        .reset                (reset),
        .start                (pipeline_start),
        .setup_params         (setup_params),
        .out_valid            (ag_valid_raw),
        .out_ready            (ag_ready_raw),
        .out_cl_addr          (ag_cl_addr),
        .out_smem_byte_addr   (ag_smem_byte_addr),
        .out_byte_offset      (ag_byte_offset),
        .out_valid_length     (ag_valid_length),
        .out_oob              (ag_oob),
        .out_last             (ag_last),
        .out_cfill            (ag_cfill),
        .out_dest_kmajor      (ag_dest_kmajor),
        .out_per_lane_stride_bytes (ag_per_lane_stride_bytes),
        .out_elem_bytes       (ag_elem_bytes),
        .out_dest_mode        (ag_dest_mode),
        .out_lg_ratio         (ag_lg_ratio),
        .out_lg_tcN           (ag_lg_tcN),
        .out_lg_nsteps        (ag_lg_nsteps),
        .out_lg_bkK           (ag_lg_bkK),
        .out_k_row            (ag_k_row),
        .out_n_base           (ag_n_base),
        .out_smem_base        (ag_smem_base)
    );

    // ════════════════════════════════════════════════════════════════════
    // Stage 3: GMEM Request Issuer
    // ════════════════════════════════════════════════════════════════════

    if (STREAM != 0) begin : g_stream_req
        // Tile-stream engine: one burst request per granule on the stream bus,
        // steered to this worker's master by the pseudo-channel address bits;
        // beats come back 64 B at a time and go to smem_wr as lines.
        VX_stream_bus_if #(
            .BURST_SIZE (TSU_BURST_BYTES),
            .BEAT_SIZE  (TSU_BEAT_BYTES),
            .TAG_WIDTH  (TSU_STR_TAG_W)
        ) str_w ();

        VX_tsu_burst_req #(
            .MAX_OUTSTANDING (MAX_OUTSTANDING),
            .BURST_SIZE      (TSU_BURST_BYTES),
            .BEAT_SIZE       (TSU_BEAT_BYTES),
            .GMEM_ADDR_WIDTH (GMEM_ADDR_WIDTH),
            .STR_TAG_WIDTH   (TSU_STR_TAG_W),
            .SMEM_ADDR_W     (DXA_SMEM_ADDR_W)
        ) burst_req (
            .clk                (clk),
            .reset              (reset),
        `ifdef PERF_ENABLE
            .perf_gmem_reqs       (perf_gmem_reqs),
            .perf_gmem_span_cycles(perf_gmem_span_cycles),
            .perf_bytes           (perf_str_bytes),
        `endif
            .transfer_active    (transfer_active),
            .transfer_start     (pipeline_start),
            .active_uuid        (active_uuid),
            .elem_bytes         (ag_elem_bytes),
            .ag_valid           (ag_valid),
            .ag_ready           (ag_ready),
            .ag_cl_addr         (ag_cl_addr),
            .ag_smem_byte_addr  (ag_smem_byte_addr),
            .ag_byte_offset     (ag_byte_offset),
            .ag_valid_length    (ag_valid_length),
            .ag_oob             (ag_oob),
            .ag_last            (ag_last),
            .ag_k_row           (ag_k_row),
            .ag_n_base          (ag_n_base),
            .str_bus_if         (str_w),
            .sw_valid           (sw_valid),
            .sw_ready           (sw_ready),
            .sw_tag             (sw_tag),
            .sw_data            (sw_data),
            .sw_smem_byte_addr  (sw_smem_byte_addr),
            .sw_byte_offset     (sw_byte_offset),
            .sw_valid_length    (sw_valid_length),
            .sw_oob             (sw_oob),
            .sw_last            (sw_last),
            .sw_k_row           (sw_k_row),
            .sw_n_base          (sw_n_base),
            .sw_outstanding     (sw_outstanding),
            .release_en         (sw_release_en),
            .release_tag        (sw_release_tag),
            .gmem_req_fire      (gmem_req_fire),
            .stall_no_slot      (stall_no_slot)
        );

        localparam STR_SEL_W = `UP(`CLOG2(NUM_STR_PORTS));
        wire [STR_SEL_W-1:0] str_sel;
        if (NUM_STR_PORTS > 1) begin : g_sel
            // VX_CFG_TSU_SEL_LSB = 28 + log2(32 / STREAM_NUM_MASTERS) must leave the select bits inside the
            // address (e.g. 4 masters on a 32-bit address would select on bits 31..32: override SEL_LSB).
            `STATIC_ASSERT(STR_SEL_LSB + STR_SEL_W <= TSU_STR_ADDR_W, ("TSU master-select bits above the stream address"))
        `ifdef VX_CFG_TSU_MASTER_INTERLEAVE
            // Phase-4 feed: consecutive bursts of one transfer round-robin over the worker's masters, so a tile that
            // sits in ONE pseudo-channel still uses all of them (each master keeps reading the same PC: no per-burst
            // PC switching inside a master). Needs every master to reach every PC (platforms.mk TSU_LINK=full).
            `ifdef VX_CFG_TSU_LINK_GROUP
            // Hard compile error in every tool (STATIC_ASSERT is an initial-block $error: lint and synthesis skip it):
            // VX_CFG_TSU_MASTER_INTERLEAVE needs TSU_LINK=full -- group-linked masters reach only their own PCs.
            `VX_CFG_TSU_MASTER_INTERLEAVE_requires_TSU_LINK_full
            `endif
            assign str_sel = str_w.req_data.addr[0 +: STR_SEL_W];
        `else
            assign str_sel = str_w.req_data.addr[STR_SEL_LSB +: STR_SEL_W];
        `endif
        end else begin : g_nosel
            assign str_sel = '0;
        end

    `ifdef VX_CFG_TSU_LINK_GROUP
        // Group-linked platform (platforms.mk TSU_LINK=group): m_axi_str_<i> is linked to HBM[i*g : i*g+g-1]
        // only, g = 2^(SEL_LSB - 28) (32/n for TSU_LINK=group, 16/n for TSU_LINK=range), and an address outside that range reads garbage on silicon
        // (u50wgts_2c200, 2026-11-11). Every burst must lie in the PC group of the master it is routed to:
        // byte-address group index addr >> (28 + log2 g) == STR_MASTER_BASE + str_sel. Flat-memory rtlsim
        // cannot see the corruption, so this assert is the pre-silicon check.
        wire [63:0] str_byte_addr = 64'(str_w.req_data.addr) << `CLOG2(TSU_BURST_BYTES);
        wire [31:0] str_grp = 32'(str_byte_addr >> `VX_CFG_TSU_SEL_LSB);
        `RUNTIME_ASSERT(~(str_w.req_valid && str_w.req_ready) || (str_grp == 32'(STR_MASTER_BASE + 32'(str_sel))),
            ("%t: *** %s TSU burst 0x%0h is in PC group %0d but routes to m_axi_str_%0d (group-linked platform: only its own %0d PCs are reachable)",
             $time, INSTANCE_ID, str_byte_addr, str_grp, STR_MASTER_BASE + 32'(str_sel), 1 << (`VX_CFG_TSU_SEL_LSB - 28)))
    `endif

        VX_stream_bus_switch #(
            .NUM_OUTPUTS (NUM_STR_PORTS),
            .BURST_SIZE  (TSU_BURST_BYTES),
            .BEAT_SIZE   (TSU_BEAT_BYTES),
            .TAG_WIDTH   (TSU_STR_TAG_W),
            .REQ_OUT_BUF ((NUM_STR_PORTS > 1) ? 3 : 0),
            .RSP_OUT_BUF ((NUM_STR_PORTS > 1) ? 3 : 0)
        ) str_switch (
            .clk        (clk),
            .reset      (reset),
            .bus_sel    (str_sel),
            .bus_in_if  (str_w),
            .bus_out_if (str_bus_if)
        );

        // No socket memory port traffic from a stream worker.
        assign gmem_bus_if.req_valid = 1'b0;
        assign gmem_bus_if.req_data  = '0;
        assign gmem_bus_if.rsp_ready = 1'b0;
        `UNUSED_VAR (gmem_bus_if.req_ready)
        `UNUSED_VAR (gmem_bus_if.rsp_valid)
        `UNUSED_VAR (gmem_bus_if.rsp_data)
    end else begin : g_line_req
        // The request issuer registers its own gmem output (VX_mem_bus_slice), so
        // gmem_bus_if is already a clean flopped producer boundary here.
        VX_dxa_gmem_req #(
            .MAX_OUTSTANDING (MAX_OUTSTANDING),
            .GMEM_ADDR_WIDTH (GMEM_ADDR_WIDTH),
            .GMEM_TAG_WIDTH  (GMEM_TAG_WIDTH),
            .CL_OFF_BITS     (GMEM_OFF_BITS),
            .SMEM_ADDR_W     (DXA_SMEM_ADDR_W)
        ) gmem_req (
            .clk                (clk),
            .reset              (reset),
        `ifdef PERF_ENABLE
            .perf_gmem_reqs       (perf_gmem_reqs),
            .perf_gmem_span_cycles(perf_gmem_span_cycles),
        `endif
            .transfer_active    (transfer_active),
            .transfer_start     (pipeline_start),
            .active_uuid        (active_uuid),
            .ag_valid           (ag_valid),
            .ag_ready           (ag_ready),
            .ag_cl_addr         (ag_cl_addr),
            .ag_smem_byte_addr  (ag_smem_byte_addr),
            .ag_byte_offset     (ag_byte_offset),
            .ag_valid_length    (ag_valid_length),
            .ag_oob             (ag_oob),
            .ag_last            (ag_last),
            .ag_k_row           (ag_k_row),
            .ag_n_base          (ag_n_base),
            .gmem_bus_if        (gmem_bus_if),
            .sw_valid           (sw_valid),
            .sw_ready           (sw_ready),
            .sw_tag             (sw_tag),
            .sw_data            (sw_data),
            .sw_smem_byte_addr  (sw_smem_byte_addr),
            .sw_byte_offset     (sw_byte_offset),
            .sw_valid_length    (sw_valid_length),
            .sw_oob             (sw_oob),
            .sw_last            (sw_last),
            .sw_k_row           (sw_k_row),
            .sw_n_base          (sw_n_base),
            .sw_outstanding     (sw_outstanding),
            .release_en         (sw_release_en),
            .release_tag        (sw_release_tag),
            .gmem_req_fire      (gmem_req_fire),
            .stall_no_slot      (stall_no_slot)
        );
    `ifdef PERF_ENABLE
        assign perf_str_bytes = '0;
    `endif
        // No stream traffic from a line worker.
        for (genvar i = 0; i < NUM_STR_PORTS; ++i) begin : g_str_tieoff
            assign str_bus_if[i].req_valid = 1'b0;
            assign str_bus_if[i].req_data  = '0;
            assign str_bus_if[i].rsp_ready = 1'b0;
            `UNUSED_VAR (str_bus_if[i].req_ready)
            `UNUSED_VAR (str_bus_if[i].rsp_valid)
            `UNUSED_VAR (str_bus_if[i].rsp_data)
        end
    end

    // ════════════════════════════════════════════════════════════════════
    // Stage 3b: W-slot landing steer (phase 4). A TILEBUF_B transfer on a
    // stream worker bypasses smem_wr: every delivered beat (a full, aligned
    // 64 B piece of one weight row) goes straight to the TCU sequencer's W
    // slot, releases its burst-slot beat on acceptance, and the transfer
    // completes once the last beat has landed and no burst slot is
    // outstanding. LMEM transfers take the unchanged smem_wr path.
    // ════════════════════════════════════════════════════════════════════

    wire                        sm_valid;
    wire                        sm_ready;
    wire                        sm_release_en;
    wire [TAG_W-1:0]            sm_release_tag;
    wire                        sm_transfer_done;

`ifdef VX_CFG_TSU_ENABLE
`ifdef VX_CFG_TCU_SEQ_ENABLE
    if (STREAM != 0) begin : g_wland
        wire wl_mode = setup_params.wl_en;
        wire [3:0] wl_lgkt = setup_params.wl_tag[7:4];
        wire [DXA_WLAND_OFF_W-1:0] wl_off = DXA_WLAND_OFF_W'(sw_smem_byte_addr);
        wire [DXA_WLAND_OFF_W-1:0] wl_kmask = (DXA_WLAND_OFF_W'(1) << wl_lgkt) - DXA_WLAND_OFF_W'(1);
        wire [DXA_WLAND_OFF_W-1:0] wl_k = wl_off & wl_kmask;
        reg  wl_last_seen_r, wl_done_r;
        dxa_wland_beat_t wl_beat_w;
        assign wl_beat_w.blk   = setup_params.wl_tag[3:2];
        assign wl_beat_w.slot  = setup_params.wl_tag[1:0];
        // row-major image: byte o -> row o >> lg_kt, segment (o & (kt-1)) >> 6; SIMT8 image (tile-local, 512 B per
        // 8 B k-chunk of 64 rows): byte o -> chunk o >> 9, rows ((o >> 3) & 63) .. +7
        wire wl_simt = setup_params.wl_tag[8];
        assign wl_beat_w.simt  = wl_simt;
        assign wl_beat_w.row   = wl_simt ? 6'(wl_off >> 3) : 6'(wl_off >> wl_lgkt);
        assign wl_beat_w.seg   = wl_simt ? 8'(wl_off >> 9) : 8'(wl_k >> 6);
        assign wl_beat_w.data  = 512'(sw_data);
        assign sm_valid      = sw_valid && ~wl_mode;
        // wl_take: a beat leaves the burst tracker (its burst-slot beat is released). wl_empty: no accepted beat
        // is still on its way to the sequencer (the transfer may complete).
        wire wl_take;
        wire wl_empty;
    `ifdef VX_CFG_TSU_WLAND_NOREG
        assign wl_valid  = sw_valid && wl_mode;
        assign wl_beat   = wl_beat_w;
        assign sw_ready  = wl_mode ? wl_ready : sm_ready;
        assign wl_take   = wl_valid && wl_ready;
        assign wl_empty  = 1'b1;
    `else
        // Landing register (default): a 2-entry skid buffer between the burst tracker and the sequencer's W-bank
        // write ports, so the slot-table read / beat-window / landing-address cone (the 2-core synth critical path)
        // and the sequencer's wl_ready cone do not chain into one cycle. Full throughput, +1 cycle per beat.
        wire wq_ready_in;
        assign wl_take  = sw_valid && wl_mode && wq_ready_in;
        assign sw_ready = wl_mode ? wq_ready_in : sm_ready;
        VX_elastic_buffer #(
            .DATAW   ($bits(dxa_wland_beat_t)),
            .SIZE    (2),
            .OUT_REG (1)
        ) wl_buf (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (sw_valid && wl_mode),
            .ready_in  (wq_ready_in),
            .data_in   (wl_beat_w),
            .data_out  (wl_beat),
            .ready_out (wl_ready),
            .valid_out (wl_valid)
        );
        assign wl_empty = ~wl_valid;
    `endif
        assign sw_release_en  = sm_release_en || wl_take;
        assign sw_release_tag = wl_take ? sw_tag : sm_release_tag;
        assign wl_done_valid  = wl_done_r;
        assign wl_done.blk    = setup_params.wl_tag[3:2];
        assign wl_done.slot   = setup_params.wl_tag[1:0];
        assign transfer_done  = sm_transfer_done || wl_done_r;
        always @(posedge clk) begin
            if (reset || pipeline_start) begin
                wl_last_seen_r <= 1'b0;
                wl_done_r      <= 1'b0;
            end else begin
                wl_done_r <= 1'b0;
                if (wl_take && sw_last) wl_last_seen_r <= 1'b1;
                if (wl_mode && transfer_active && wl_last_seen_r && (sw_outstanding == '0) && wl_empty && ~wl_done_r) begin
                    wl_done_r      <= 1'b1;
                    wl_last_seen_r <= 1'b0;
                end
            end
        end
        `RUNTIME_ASSERT(~wl_take || (~sw_oob && (sw_byte_offset == '0) && (32'(sw_valid_length) == 32'(CL_BYTES))
                                     && (wl_off[5:0] == 6'd0) && (wl_lgkt >= 4'd6) && (wl_lgkt <= 4'd11)),
            ("%t: *** %s W-slot landing needs full 64 B beats of a 2^lg_kt (64..2048) byte row: off=0x%0h len=%0d oob=%b lgkt=%0d",
             $time, INSTANCE_ID, sw_byte_offset, sw_valid_length, sw_oob, wl_lgkt))
        `RUNTIME_ASSERT(~wl_take || ((32'(wl_off) >> wl_lgkt) < 32'd64),
            ("%t: *** %s W-slot landing offset 0x%0h is past row 63 of a 2^%0d B-row slot (issue with the LMEM base csr LOCAL_MEM_BASE, not __local_mem() or 0)",
             $time, INSTANCE_ID, wl_off, wl_lgkt))
        `RUNTIME_ASSERT(~(sm_release_en && wl_take), ("%t: *** %s W-landing / smem_wr release collision", $time, INSTANCE_ID))
        `UNUSED_VAR (wl_k)
        assign perf_wl_take = wl_take;
        assign perf_wl_wait = sw_valid && wl_mode && ~sw_ready;
    end else begin : g_no_wland
        assign perf_wl_take = 1'b0;
        assign perf_wl_wait = 1'b0;
        assign wl_valid       = 1'b0;
        assign wl_beat        = '0;
        assign wl_done_valid  = 1'b0;
        assign wl_done        = '0;
        assign sm_valid       = sw_valid;
        assign sw_ready       = sm_ready;
        assign sw_release_en  = sm_release_en;
        assign sw_release_tag = sm_release_tag;
        assign transfer_done  = sm_transfer_done;
        `UNUSED_VAR (wl_ready)
        `RUNTIME_ASSERT(~(transfer_active && setup_params.wl_en), ("%t: *** %s TILEBUF_B descriptor on the DXA line engine (set the TSU ENGINE bit)", $time, INSTANCE_ID))
    end
`else
    assign sm_valid       = sw_valid;
    assign sw_ready       = sm_ready;
    assign sw_release_en  = sm_release_en;
    assign sw_release_tag = sm_release_tag;
    assign transfer_done  = sm_transfer_done;
    assign perf_wl_take   = 1'b0;
    assign perf_wl_wait   = 1'b0;
`endif

    // Line workers drive the VX_CFG_MEM_ADDR_WIDTH memory bus: a descriptor base above it (full-width
    // VX_CFG_TSU_ADDR_WIDTH descriptor, meant for the TSU) would be silently truncated.
    if (!STREAM && (DXA_GADDR_W > `VX_CFG_MEM_ADDR_WIDTH)) begin : g_line_base_chk
        `RUNTIME_ASSERT(~transfer_active || ((setup_params.initial_gmem_base >> `VX_CFG_MEM_ADDR_WIDTH) == '0),
            ("%t: *** %s DXA line-engine base 0x%0h is above the %0d-bit memory bus (route it to the TSU)",
             $time, INSTANCE_ID, setup_params.initial_gmem_base, `VX_CFG_MEM_ADDR_WIDTH))
    end
`else
    assign sm_valid       = sw_valid;
    assign sw_ready       = sm_ready;
    assign sw_release_en  = sm_release_en;
    assign sw_release_tag = sm_release_tag;
    assign transfer_done  = sm_transfer_done;
    assign perf_wl_take   = 1'b0;
    assign perf_wl_wait   = 1'b0;
`endif

    // ════════════════════════════════════════════════════════════════════
    // Stage 4: SMEM Writer (gmem_req drives smem_wr directly via the
    // sw_* channel; smem_wr's pend slot + deferred-last register handle
    // out-of-order drain).
    // ════════════════════════════════════════════════════════════════════

`ifdef VX_CFG_TSU_WLAND_ONLY
`ifndef VX_CFG_TSU_ENABLE
    `VX_CFG_TSU_WLAND_ONLY_requires_VX_CFG_TSU_ENABLE
`endif
`ifndef VX_CFG_TCU_SEQ_ENABLE
    `VX_CFG_TSU_WLAND_ONLY_requires_VX_CFG_TCU_SEQ_ENABLE
`endif
    // VX_CFG_TSU_WLAND_ONLY (sequencer builds): stream workers only land W slots (TILEBUF_B, the g_wland path
    // above), so they carry no LMEM writer. A TSU transfer to LMEM on such a build is a fatal assert (it would
    // hang: nothing drains it). Line workers keep smem_wr.
    localparam HAS_SMEM_WR = (STREAM == 0);
`else
    localparam HAS_SMEM_WR = 1;
`endif

if (HAS_SMEM_WR != 0) begin : g_smem_wr
    VX_dxa_smem_wr #(
        .MAX_OUTSTANDING (MAX_OUTSTANDING),
        .CL_SIZE         (CL_BYTES),
        .SMEM_WORD_SIZE  (SMEM_BYTES),
        .SMEM_ADDR_WIDTH (SMEM_ADDR_WIDTH),
        .GMEM_DATAW      (CL_DATAW)
    ) smem_wr (
        .clk                   (clk),
        .reset                 (reset),
    `ifdef PERF_ENABLE
        .perf_lmem_writes      (perf_lmem_writes),
    `endif
        .transfer_active       (transfer_active),
        .transfer_start        (pipeline_start),
        .cfill                 (ag_cfill),
        .active_core_id        (active_core_id),
        .active_uuid           (active_uuid),
        .active_bar_addr       (active_bar_addr),
        .active_notify_smem_done (active_notify_smem_done),
        .sw_valid              (sm_valid),
        .sw_ready              (sm_ready),
        .sw_tag                (sw_tag),
        .sw_data               (sw_data),
        .sw_smem_byte_addr     (sw_smem_byte_addr),
        .sw_byte_offset        (sw_byte_offset),
        .sw_valid_length       (sw_valid_length),
        .sw_oob                (sw_oob),
        .sw_last               (sw_last),
        .sw_k_row              (sw_k_row),
        .sw_n_base             (sw_n_base),
        .sw_outstanding        (sw_outstanding),
        .release_en            (sm_release_en),
        .release_tag           (sm_release_tag),
        .smem_bus_if           (smem_bus_if),
        .transfer_done         (sm_transfer_done),
        .wr_done_count         (wr_done_count),
        .smem_req_fire         (smem_req_fire),
        .is_multicast          (active_is_multicast),
        .cta_mask              (active_cta_mask),
        .smem_stride           (active_smem_stride),
        .dest_kmajor           (ag_dest_kmajor),
        .per_lane_stride_bytes (ag_per_lane_stride_bytes),
        .elem_bytes            (ag_elem_bytes),
        .dest_mode             (ag_dest_mode),
        .lg_ratio              (ag_lg_ratio),
        .lg_tcN                (ag_lg_tcN),
        .lg_nsteps             (ag_lg_nsteps),
        .lg_bkK                (ag_lg_bkK),
        .smem_base             (ag_smem_base)
    );
end else begin : g_no_smem_wr
    assign sm_ready         = 1'b0;
    assign sm_release_en    = 1'b0;
    assign sm_release_tag   = '0;
    assign sm_transfer_done = 1'b0;
    assign wr_done_count    = '0;
    assign smem_req_fire    = 1'b0;
`ifdef PERF_ENABLE
    assign perf_lmem_writes = '0;
`endif
    assign smem_bus_if.req_valid = 1'b0;
    assign smem_bus_if.req_data  = '0;
    assign smem_bus_if.rsp_ready = 1'b0;
    `UNUSED_VAR (smem_bus_if.req_ready)
    `UNUSED_VAR (smem_bus_if.rsp_valid)
    `UNUSED_VAR (smem_bus_if.rsp_data)
    `UNUSED_VAR ({ag_cfill, active_core_id, active_uuid, active_bar_addr, active_notify_smem_done, sm_valid, sw_tag,
                  sw_byte_offset, sw_valid_length, sw_oob, sw_k_row, sw_n_base, active_is_multicast, active_cta_mask,
                  active_smem_stride, ag_dest_kmajor, ag_per_lane_stride_bytes, ag_elem_bytes, ag_dest_mode,
                  ag_lg_ratio, ag_lg_tcN, ag_lg_nsteps, ag_lg_bkK, ag_smem_base})
    `RUNTIME_ASSERT(~(transfer_active && ~setup_params.wl_en),
        ("%t: *** %s TSU transfer to LMEM on a VX_CFG_TSU_WLAND_ONLY build (stream workers only land W slots)", $time, INSTANCE_ID))
end

    // ════════════════════════════════════════════════════════════════════
    // Watchdog
    // ════════════════════════════════════════════════════════════════════

    VX_dxa_watchdog #(
        .INSTANCE_ID (INSTANCE_ID)
    ) watchdog (
        .clk              (clk),
        .reset            (reset),
        .transfer_active  (transfer_active),
        .gmem_req_fire    (gmem_req_fire),
        .gmem_rsp_valid   (sw_valid && sw_ready),  // any CL delivered to smem_wr
        .smem_req_fire    (smem_req_fire),
        .active_core_id   (active_core_id),
        .active_wid       (active_wid),
        .active_bar_addr  (active_bar_addr)
    );

    // ════════════════════════════════════════════════════════════════════
    // Perf counters
    // ════════════════════════════════════════════════════════════════════

`ifdef PERF_ENABLE
    reg [PERF_CTR_BITS-1:0] perf_transfers_r;
    reg [PERF_CTR_BITS-1:0] perf_gmem_reads_r;
    reg [PERF_CTR_BITS-1:0] perf_gmem_dedup_r;
    reg [PERF_CTR_BITS-1:0] perf_lmem_writes_r;
    reg [PERF_CTR_BITS-1:0] perf_gmem_lt_r;
    reg [PERF_CTR_BITS-1:0] perf_noslot_r;
    reg [PERF_CTR_BITS-1:0] perf_tsu_bursts_r;
    reg [PERF_CTR_BITS-1:0] perf_tsu_bytes_r;
    reg [PERF_CTR_BITS-1:0] perf_land_beats_r, perf_land_stall_r;
    reg [SEQ_W-1:0]         perf_max_outst_r;
    reg [PERF_CTR_BITS-1:0] perf_outst_sum_r, perf_busy_cyc_r;
    reg                     perf_wl_take_d, perf_wl_wait_d;
    reg [SEQ_W-1:0]         perf_outst_d;
    always @(posedge clk) begin
        perf_wl_take_d <= perf_wl_take;
        perf_wl_wait_d <= perf_wl_wait;
        perf_outst_d   <= sw_outstanding;
        if (reset) begin
            perf_land_beats_r <= '0; perf_land_stall_r <= '0; perf_max_outst_r <= '0;
            perf_outst_sum_r  <= '0; perf_busy_cyc_r <= '0;
        end else begin
            perf_land_beats_r <= perf_land_beats_r + PERF_CTR_BITS'(perf_wl_take_d);
            perf_land_stall_r <= perf_land_stall_r + PERF_CTR_BITS'(perf_wl_wait_d);
            if (perf_outst_d > perf_max_outst_r) perf_max_outst_r <= perf_outst_d;
            perf_outst_sum_r <= perf_outst_sum_r + PERF_CTR_BITS'(perf_outst_d);
            perf_busy_cyc_r  <= perf_busy_cyc_r + PERF_CTR_BITS'(perf_outst_d != '0);
        end
    end
    always @(posedge clk) begin
        if (reset) begin
            perf_transfers_r   <= '0;
            perf_gmem_reads_r  <= '0;
            perf_gmem_dedup_r  <= '0;
            perf_lmem_writes_r <= '0;
            perf_gmem_lt_r     <= '0;
            perf_noslot_r      <= '0;
            perf_tsu_bursts_r  <= '0;
            perf_tsu_bytes_r   <= '0;
        end else begin
            if (stall_no_slot) begin
                perf_noslot_r <= perf_noslot_r + PERF_CTR_BITS'(1);
            end
            if (transfer_active && transfer_done) begin
                perf_transfers_r   <= perf_transfers_r + PERF_CTR_BITS'(1);
                perf_gmem_reads_r  <= perf_gmem_reads_r + PERF_CTR_BITS'(perf_gmem_reqs);
                perf_lmem_writes_r <= perf_lmem_writes_r + PERF_CTR_BITS'(perf_lmem_writes);
                perf_gmem_lt_r     <= perf_gmem_lt_r + PERF_CTR_BITS'(perf_gmem_span_cycles);
                if (STREAM != 0) begin
                    perf_tsu_bursts_r <= perf_tsu_bursts_r + PERF_CTR_BITS'(perf_gmem_reqs);
                    perf_tsu_bytes_r  <= perf_tsu_bytes_r + PERF_CTR_BITS'(perf_str_bytes);
                end
            end
        end
    end
    assign dxa_perf.transfers    = perf_transfers_r;
    assign dxa_perf.gmem_reads   = perf_gmem_reads_r;
    assign dxa_perf.gmem_dedup   = perf_gmem_dedup_r;
    assign dxa_perf.lmem_writes  = perf_lmem_writes_r;
    assign dxa_perf.gmem_latency = perf_gmem_lt_r;
    assign dxa_perf.noslot_stalls = perf_noslot_r;
    assign dxa_perf.tsu_bursts   = perf_tsu_bursts_r;
    assign dxa_perf.tsu_bytes    = perf_tsu_bytes_r;
    assign dxa_perf.tsu_land_beats = perf_land_beats_r;
    assign dxa_perf.tsu_land_stall = perf_land_stall_r;
    assign dxa_perf.tsu_max_outst  = PERF_CTR_BITS'(perf_max_outst_r);
    assign dxa_perf.tsu_outst_sum  = perf_outst_sum_r;
    assign dxa_perf.tsu_busy_cyc   = perf_busy_cyc_r;
`else
    `UNUSED_VAR (stall_no_slot)
`endif
`ifndef DBG_TRACE_DXA
    `UNUSED_VAR (wr_done_count)
`endif

`ifdef DBG_TRACE_DXA
    always @(posedge clk) begin
        if (~reset) begin
            if (pipeline_start) begin
                `TRACE(1, ("%t: %s setup-done: core=%0d, wid=%0d, bar=%0d\n",
                    $time, INSTANCE_ID, active_core_id, active_wid, active_bar_addr))
                $write("DXA_TL,%0d,SETUP_DONE,core=%0d,wid=%0d,bar=%0d\n",
                    $time, active_core_id, active_wid, active_bar_addr);
            end
            if (transfer_active) begin
                if (gmem_req_fire) begin
                    `TRACE(2, ("%t: %s gmem-req-fire\n", $time, INSTANCE_ID))
                end
                if (sw_valid && sw_ready) begin
                    `TRACE(2, ("%t: %s sw-deliver: tag=%0d, oob=%0d, last=%0d\n",
                        $time, INSTANCE_ID, sw_tag, sw_oob, sw_last))
                end
                if (smem_req_fire) begin
                    `TRACE(2, ("%t: %s smem-wr-fire: count=%0d\n",
                        $time, INSTANCE_ID, wr_done_count))
                end
            end
            if (transfer_active && transfer_done) begin
                `TRACE(1, ("%t: %s done: core=%0d, wid=%0d, bar=%0d, wr_count=%0d\n",
                    $time, INSTANCE_ID, active_core_id, active_wid,
                    active_bar_addr, wr_done_count))
                $write("DXA_TL,%0d,XFER_DONE,core=%0d,wid=%0d,bar=%0d,wr_count=%0d\n",
                    $time, active_core_id, active_wid, active_bar_addr,
                    wr_done_count);
            end
        end
    end
`endif

endmodule
