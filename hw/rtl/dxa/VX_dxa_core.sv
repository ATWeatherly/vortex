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

module VX_dxa_core import VX_gpu_pkg::*, VX_dxa_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter NUM_REQS = 1,
    parameter GMEM_OUT_PORTS = 1,
    parameter NUM_STR_PORTS = `UP(SOCKET_STR_PORTS),  // tile-stream masters (1 dummy when the TSU is off)
    parameter STR_MASTER_BASE = 0                     // global m_axi_str index of this socket's master 0
) (
    input wire clk,
    input wire reset,
`ifdef PERF_ENABLE
    output dxa_perf_t dxa_perf,
`endif
    VX_dcr_bus_if.slave dcr_bus_if,
    VX_dxa_req_bus_if.slave req_bus_if[NUM_REQS],
    VX_mem_bus_if.master gmem_bus_if[GMEM_OUT_PORTS],
    VX_stream_bus_if.master str_bus_if[NUM_STR_PORTS],
    VX_mem_bus_if.master smem_bus_if[1],
`ifdef VX_CFG_TSU_ENABLE
`ifdef VX_CFG_TCU_SEQ_ENABLE
    // Phase-4 W-slot landing to the socket's core TCU sequencer (one core per socket)
    output wire                 wl_valid,
    input  wire                 wl_ready,
    output dxa_wland_beat_t     wl_beat,
    output wire                 wl_done_valid,
    output dxa_wland_done_t     wl_done,
`endif
`endif
    output wire busy
);
    `UNUSED_SPARAM (INSTANCE_ID)
    `UNUSED_PARAM (STR_MASTER_BASE)   // TSU workers only

    // Worker pool: line workers [0, NUM_DXA_CORES) on the socket memory port,
    // stream workers [NUM_DXA_CORES, NUM_WORKERS) on the tile-stream masters.
    localparam NUM_LINE_WORKERS = `VX_CFG_NUM_DXA_CORES;
`ifdef VX_CFG_TSU_ENABLE
    localparam NUM_STR_WORKERS  = TSU_NUM_WORKERS;
`else
    localparam NUM_STR_WORKERS  = 0;
`endif
    localparam NUM_WORKERS      = NUM_LINE_WORKERS + NUM_STR_WORKERS;
    `STATIC_ASSERT(NUM_WORKERS == DXA_LMEM_NUM_WORKERS, ("DXA worker count mismatch with VX_gpu_pkg"))

    // ================================================================
    // Descriptor table (single read port, driven by dispatch queue)
    // ================================================================
    wire [DXA_DESC_SLOT_W-1:0] desc_read_addr;
    dxa_desc_t desc_read_data;

    VX_dxa_desc_table desc_table (
        .clk        (clk),
        .reset      (reset),
        .dcr_bus_if (dcr_bus_if),
        .read_addr  (desc_read_addr),
        .read_desc  (desc_read_data)
    );

    // ================================================================
    // Request path: sockets → arb(N:1) → queue → dispatch(1:N) → workers
    // ================================================================
    localparam REQ_DATAW = $bits(dxa_req_data_t);

    VX_dxa_req_bus_if arb_out_bus_if[1]();

    VX_dxa_req_arb #(
        .NUM_INPUTS  (NUM_REQS),
        .NUM_OUTPUTS (1),
        // Register the arbitration output whenever it actually arbitrates
        // (>1 input). 1:1 stays a passthrough (no area/latency cost).
        .OUT_BUF     ((NUM_REQS > 1) ? 3 : 0)
    ) req_arb (
        .clk        (clk),
        .reset      (reset),
        .bus_in_if  (req_bus_if),
        .bus_out_if (arb_out_bus_if)
    );

    VX_dxa_req_bus_if queue_out_bus_if[1]();

    VX_elastic_buffer #(
        .DATAW  (REQ_DATAW),
        .SIZE   (`VX_CFG_DXA_QUEUE_SIZE),
        // LUTRAM=0: the wide-shallow queue is BRAM-eligible, moving it off the
        // LUT pool onto an (abundant) BRAM tile.
        .LUTRAM (0)
    ) req_queue (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (arb_out_bus_if[0].req_valid),
        .ready_in  (arb_out_bus_if[0].req_ready),
        .data_in   (arb_out_bus_if[0].req_data),
        .valid_out (queue_out_bus_if[0].req_valid),
        .ready_out (queue_out_bus_if[0].req_ready),
        .data_out  (queue_out_bus_if[0].req_data)
    );

    // Read desc_table using the queued request's desc_slot. desc_store is now
    // a registered-read BRAM (OUT_REG=1), so desc_read_data lands one cycle
    // after desc_read_addr. A 1-deep fetch register absorbs that latency: the
    // queue head's slot drives the address combinationally; the next cycle the
    // captured request and its descriptor are presented together to dispatch.
    // DXA launches are infrequent (one per multi-hundred-cycle transfer), so
    // the extra launch cycle is throughput-irrelevant.
    assign desc_read_addr = DXA_DESC_SLOT_W'(queue_out_bus_if[0].req_data.meta[DXA_DESC_SLOT_W-1:0]);

    // Bundle request + descriptor into dispatch input
    VX_dxa_worker_req_if dispatch_in_if[1]();

    // desc_store is a registered-read BRAM: desc_read_data lags desc_read_addr
    // by one cycle. A 2-state fetch FSM waits that cycle, and crucially holds
    // the queue head (so desc_read_addr — and therefore desc_read_data — stays
    // stable) until dispatch consumes. The queue is popped only on consume, so
    // a request and its descriptor are always presented in alignment, even when
    // dispatch back-pressures during streamed back-to-back transfers. Launch
    // latency is +1 cycle, negligible against a multi-hundred-cycle drain.
    localparam FETCH_IDLE = 1'b0, FETCH_PRESENT = 1'b1;
    reg fetch_state_r;

    // Pop only when presenting AND dispatch accepts; the head (hence the read
    // address) stays put through IDLE→PRESENT and across any dispatch stall.
    assign queue_out_bus_if[0].req_ready =
        (fetch_state_r == FETCH_PRESENT) && dispatch_in_if[0].ready;

    always @(posedge clk) begin
        if (reset) begin
            fetch_state_r <= FETCH_IDLE;
        end else begin
            case (fetch_state_r)
                FETCH_IDLE:    if (queue_out_bus_if[0].req_valid) fetch_state_r <= FETCH_PRESENT;
                FETCH_PRESENT: if (dispatch_in_if[0].ready)       fetch_state_r <= FETCH_IDLE;
            endcase
        end
    end

    assign dispatch_in_if[0].valid     = (fetch_state_r == FETCH_PRESENT);
    assign dispatch_in_if[0].req_data  = queue_out_bus_if[0].req_data;
    assign dispatch_in_if[0].desc_data = desc_read_data;

    // Engine select: the descriptor's ENGINE meta bit routes a launch to the
    // line workers (DXA) or the tile-stream workers (TSU).
    VX_dxa_worker_req_if worker_req_if[`VX_CFG_NUM_DXA_CORES]();

`ifdef VX_CFG_TSU_ENABLE
    wire dispatch_is_tsu = (desc_read_data.meta[DXA_DESC_META_ENGINE_LSB] == DXA_ENGINE_TSU);

    VX_dxa_worker_req_if dispatch_dxa_if[1]();
    VX_dxa_worker_req_if dispatch_tsu_if[1]();
    VX_dxa_worker_req_if tsu_worker_req_if[NUM_STR_WORKERS]();

    assign dispatch_dxa_if[0].valid     = dispatch_in_if[0].valid && ~dispatch_is_tsu;
    assign dispatch_dxa_if[0].req_data  = dispatch_in_if[0].req_data;
    assign dispatch_dxa_if[0].desc_data = dispatch_in_if[0].desc_data;
    assign dispatch_tsu_if[0].valid     = dispatch_in_if[0].valid && dispatch_is_tsu;
    assign dispatch_tsu_if[0].req_data  = dispatch_in_if[0].req_data;
    assign dispatch_tsu_if[0].desc_data = dispatch_in_if[0].desc_data;
    assign dispatch_in_if[0].ready = dispatch_is_tsu ? dispatch_tsu_if[0].ready : dispatch_dxa_if[0].ready;

    VX_dxa_dispatch #(
        .NUM_INPUTS  (1),
        .NUM_OUTPUTS (`VX_CFG_NUM_DXA_CORES)
    ) issue_dispatch (
        .clk       (clk),
        .reset     (reset),
        .req_in    (dispatch_dxa_if),
        .req_out   (worker_req_if)
    );

`ifdef VX_CFG_TCU_SEQ_ENABLE
    // W-slot landings are steered to worker (blk mod NUM_STR_WORKERS) so every landing into one W slot goes through
    // ONE worker, which runs its transfers in order: the sequencer's per-slot expectation FIFO relies on landings
    // into a slot completing in issue order. Other TSU transfers go to the lowest-index idle worker.
    localparam TSW_W = `UP(`CLOG2(NUM_STR_WORKERS));
    wire        tsd_valid = dispatch_tsu_if[0].valid;
    wire        tsd_is_wl = (dispatch_tsu_if[0].desc_data.meta[DXA_DESC_META_DEST_KIND_LSB +: DXA_DESC_META_DEST_KIND_BITS]
                             == DXA_DEST_KIND_TILEBUF_B);
    wire [1:0]  tsd_blk   = dispatch_tsu_if[0].req_data.meta[6 +: 2];   // tag {lg_kt, blk, slot} at meta[11:4]
    wire [TSW_W-1:0] tsd_wl_tgt = TSW_W'(32'(tsd_blk) % NUM_STR_WORKERS);
    wire [NUM_STR_WORKERS-1:0] tsd_rdy;
    for (genvar i = 0; i < NUM_STR_WORKERS; ++i) begin : g_tsu_rdy
        assign tsd_rdy[i] = tsu_worker_req_if[i].ready;
    end
    logic [TSW_W-1:0] tsd_free;
    logic             tsd_any_free;
    always_comb begin
        tsd_free = '0;
        tsd_any_free = 1'b0;
        for (integer k = NUM_STR_WORKERS - 1; k >= 0; --k) begin
            if (tsd_rdy[k]) begin tsd_free = TSW_W'(k); tsd_any_free = 1'b1; end
        end
    end
    wire [TSW_W-1:0] tsd_tgt = tsd_is_wl ? tsd_wl_tgt : tsd_free;
    for (genvar i = 0; i < NUM_STR_WORKERS; ++i) begin : g_tsu_steer
        assign tsu_worker_req_if[i].valid     = tsd_valid && (tsd_tgt == TSW_W'(i)) && (tsd_is_wl || tsd_any_free);
        assign tsu_worker_req_if[i].req_data  = dispatch_tsu_if[0].req_data;
        assign tsu_worker_req_if[i].desc_data = dispatch_tsu_if[0].desc_data;
    end
    assign dispatch_tsu_if[0].ready = tsd_is_wl ? tsd_rdy[tsd_wl_tgt] : tsd_any_free;
`else
    VX_dxa_dispatch #(
        .NUM_INPUTS  (1),
        .NUM_OUTPUTS (NUM_STR_WORKERS)
    ) tsu_dispatch (
        .clk       (clk),
        .reset     (reset),
        .req_in    (dispatch_tsu_if),
        .req_out   (tsu_worker_req_if)
    );
`endif
`else
    VX_dxa_dispatch #(
        .NUM_INPUTS  (1),
        .NUM_OUTPUTS (`VX_CFG_NUM_DXA_CORES)
    ) issue_dispatch (
        .clk       (clk),
        .reset     (reset),
        .req_in    (dispatch_in_if),
        .req_out   (worker_req_if)
    );
`endif

    // ================================================================
    // Workers
    // ================================================================
    localparam GMEM_ARB_SEL_BITS = `ARB_SEL_BITS(`VX_CFG_NUM_DXA_CORES, GMEM_OUT_PORTS);
    localparam WORKER_GMEM_TAG_WIDTH = SOCKET_MEM_TAG_WIDTH - GMEM_ARB_SEL_BITS;

    VX_mem_bus_if #(
        .DATA_SIZE (`VX_CFG_L1_LINE_SIZE),
        .TAG_WIDTH (WORKER_GMEM_TAG_WIDTH)
    ) worker_gmem_bus_if[`VX_CFG_NUM_DXA_CORES]();

    VX_mem_bus_if #(
        .DATA_SIZE   (DXA_LMEM_WORD_SIZE),
        .TAG_WIDTH   (DXA_LMEM_TAG_W),
        .ATTR_WIDTH  (DXA_LMEM_ATTR_W),
        .ADDR_WIDTH  (DXA_LMEM_ADDR_W)
    ) worker_smem_bus_if[NUM_WORKERS]();

    // Line workers carry an unused stream port; stream workers an unused
    // socket memory port (both tied off inside the worker).
    VX_stream_bus_if #(
        .BURST_SIZE (TSU_BURST_BYTES),
        .BEAT_SIZE  (TSU_BEAT_BYTES),
        .TAG_WIDTH  (TSU_STR_TAG_W)
    ) line_str_dummy_if[`VX_CFG_NUM_DXA_CORES]();

`ifdef PERF_ENABLE
    dxa_perf_t worker_dxa_perf [NUM_WORKERS];
`endif

    for (genvar i = 0; i < `VX_CFG_NUM_DXA_CORES; ++i) begin : g_workers
        assign line_str_dummy_if[i].req_ready = 1'b0;
        assign line_str_dummy_if[i].rsp_valid = 1'b0;
        assign line_str_dummy_if[i].rsp_data  = '0;
        `UNUSED_VAR (line_str_dummy_if[i].req_valid)
        `UNUSED_VAR (line_str_dummy_if[i].req_data)
        `UNUSED_VAR (line_str_dummy_if[i].rsp_ready)

        VX_dxa_worker #(
            .INSTANCE_ID(`SFORMATF(("%s-worker%0d", INSTANCE_ID, i))),
            .WORKER_ID  (i),
            .GMEM_TAG_WIDTH (WORKER_GMEM_TAG_WIDTH),
            .STREAM         (0),
            .NUM_STR_PORTS  (1)
        ) worker (
            .clk            (clk),
            .reset          (reset),
        `ifdef PERF_ENABLE
            .dxa_perf       (worker_dxa_perf[i]),
        `endif
            .req_if         (worker_req_if[i]),
            .gmem_bus_if    (worker_gmem_bus_if[i]),
            .str_bus_if     (line_str_dummy_if[i +: 1]),
        `ifdef VX_CFG_TSU_ENABLE
        `ifdef VX_CFG_TCU_SEQ_ENABLE
            `UNUSED_PIN (wl_valid),
            .wl_ready       (1'b0),
            `UNUSED_PIN (wl_beat),
            `UNUSED_PIN (wl_done_valid),
            `UNUSED_PIN (wl_done),
        `endif
        `endif
            .smem_bus_if    (worker_smem_bus_if[i])
        );
    end

`ifdef VX_CFG_TSU_ENABLE
    VX_mem_bus_if #(
        .DATA_SIZE (`VX_CFG_L1_LINE_SIZE),
        .TAG_WIDTH (WORKER_GMEM_TAG_WIDTH)
    ) tsu_gmem_dummy_if[NUM_STR_WORKERS]();

`ifdef VX_CFG_TCU_SEQ_ENABLE
    // W-slot landing merge: one beat per cycle from the stream workers (round-robin, combinational so a
    // worker's completion pulse can never overtake its own last beat); completion pulses are held per worker
    // and forwarded one per cycle.
    wire [NUM_STR_WORKERS-1:0] tw_valid, tw_ready, tw_done_valid;
    dxa_wland_beat_t           tw_beat [NUM_STR_WORKERS];
    dxa_wland_done_t           tw_done [NUM_STR_WORKERS];
    localparam TW_W = `UP(`CLOG2(NUM_STR_WORKERS));
    reg  [TW_W-1:0] tw_rr_r;
    logic [TW_W-1:0] tw_sel;
    logic            tw_any;
    always_comb begin
        tw_sel = tw_rr_r;
        tw_any = 1'b0;
        for (integer k = 0; k < NUM_STR_WORKERS; ++k) begin
            automatic logic [TW_W-1:0] s = TW_W'((32'(tw_rr_r) + k) % NUM_STR_WORKERS);
            if (!tw_any && tw_valid[s]) begin tw_sel = s; tw_any = 1'b1; end
        end
    end
    assign wl_valid = tw_any;
    assign wl_beat  = tw_beat[tw_sel];
    for (genvar i = 0; i < NUM_STR_WORKERS; ++i) begin : g_tw_rdy
        assign tw_ready[i] = wl_ready && tw_any && (tw_sel == TW_W'(i));
    end
    reg [NUM_STR_WORKERS-1:0] tw_dpend_r;
    dxa_wland_done_t          tw_dval_r [NUM_STR_WORKERS];
    logic [TW_W-1:0] td_sel;
    logic            td_any;
    always_comb begin
        td_sel = '0;
        td_any = 1'b0;
        for (integer k = 0; k < NUM_STR_WORKERS; ++k) begin
            if (!td_any && tw_dpend_r[k]) begin td_sel = TW_W'(k); td_any = 1'b1; end
        end
    end
    assign wl_done_valid = td_any;
    assign wl_done       = tw_dval_r[td_sel];
    always @(posedge clk) begin
        if (reset) begin
            tw_rr_r    <= '0;
            tw_dpend_r <= '0;
        end else begin
            if (tw_any && wl_ready) tw_rr_r <= TW_W'((32'(tw_sel) + 1) % NUM_STR_WORKERS);
            for (integer k = 0; k < NUM_STR_WORKERS; ++k) begin
                if (td_any && (td_sel == TW_W'(k))) tw_dpend_r[k] <= 1'b0;
                if (tw_done_valid[k]) begin tw_dpend_r[k] <= 1'b1; tw_dval_r[k] <= tw_done[k]; end
            end
        end
    end
    `RUNTIME_ASSERT(~|(tw_done_valid & tw_dpend_r), ("%t: *** %s W-landing completion overrun", $time, INSTANCE_ID))
`endif

    for (genvar i = 0; i < NUM_STR_WORKERS; ++i) begin : g_tsu_workers
        assign tsu_gmem_dummy_if[i].req_ready = 1'b0;
        assign tsu_gmem_dummy_if[i].rsp_valid = 1'b0;
        assign tsu_gmem_dummy_if[i].rsp_data  = '0;
        `UNUSED_VAR (tsu_gmem_dummy_if[i].req_valid)
        `UNUSED_VAR (tsu_gmem_dummy_if[i].req_data)
        `UNUSED_VAR (tsu_gmem_dummy_if[i].rsp_ready)

        VX_dxa_worker #(
            .INSTANCE_ID(`SFORMATF(("%s-tsu%0d", INSTANCE_ID, i))),
            .WORKER_ID  (NUM_LINE_WORKERS + i),
            .GMEM_TAG_WIDTH (WORKER_GMEM_TAG_WIDTH),
            .STREAM         (1),
            .NUM_STR_PORTS  (TSU_MASTERS_PER_WORKER),
            .STR_MASTER_BASE(STR_MASTER_BASE + i * TSU_MASTERS_PER_WORKER)
        ) tsu_worker (
            .clk            (clk),
            .reset          (reset),
        `ifdef PERF_ENABLE
            .dxa_perf       (worker_dxa_perf[NUM_LINE_WORKERS + i]),
        `endif
            .req_if         (tsu_worker_req_if[i]),
            .gmem_bus_if    (tsu_gmem_dummy_if[i]),
            .str_bus_if     (str_bus_if[i * TSU_MASTERS_PER_WORKER +: TSU_MASTERS_PER_WORKER]),
        `ifdef VX_CFG_TCU_SEQ_ENABLE
            .wl_valid       (tw_valid[i]),
            .wl_ready       (tw_ready[i]),
            .wl_beat        (tw_beat[i]),
            .wl_done_valid  (tw_done_valid[i]),
            .wl_done        (tw_done[i]),
        `endif
            .smem_bus_if    (worker_smem_bus_if[NUM_LINE_WORKERS + i])
        );
    end
`else
    for (genvar i = 0; i < NUM_STR_PORTS; ++i) begin : g_str_tieoff
        assign str_bus_if[i].req_valid = 1'b0;
        assign str_bus_if[i].req_data  = '0;
        assign str_bus_if[i].rsp_ready = 1'b0;
        `UNUSED_VAR (str_bus_if[i].req_ready)
        `UNUSED_VAR (str_bus_if[i].rsp_valid)
        `UNUSED_VAR (str_bus_if[i].rsp_data)
    end
`endif

    // ================================================================
    // Output arbitration
    // ================================================================
    VX_mem_bus_arb #(
        .NUM_INPUTS  (`VX_CFG_NUM_DXA_CORES),
        .NUM_OUTPUTS (GMEM_OUT_PORTS),
        .DATA_SIZE   (`VX_CFG_L1_LINE_SIZE),
        .TAG_WIDTH   (WORKER_GMEM_TAG_WIDTH),
        .ARBITER     ("R"),
        .REQ_OUT_BUF ((`VX_CFG_NUM_DXA_CORES > 1) ? 3 : 0)
    ) gmem_arb (
        .clk        (clk),
        .reset      (reset),
        .bus_in_if  (worker_gmem_bus_if),
        .bus_out_if (gmem_bus_if)
    );

    VX_mem_bus_arb #(
        .NUM_INPUTS  (NUM_WORKERS),
        .NUM_OUTPUTS (1),
        .DATA_SIZE   (DXA_LMEM_WORD_SIZE),
        .TAG_WIDTH   (DXA_LMEM_TAG_W),
        .TAG_SEL_IDX (DXA_LMEM_TAG_W - UUID_WIDTH),
        .ATTR_WIDTH  (DXA_LMEM_ATTR_W),
        .ADDR_WIDTH  (DXA_LMEM_ADDR_W),
        .ARBITER     ("R"),
        // Register the SMEM write request at the DXA boundary: the tiled
        // dest-address is a deep cone whose sink is the far-away core LMEM BRAM,
        // so terminate it at a local flop to keep placement compact and isolate
        // the long DXA→core route into its own cycle.
        .REQ_OUT_BUF (3)
    ) lmem_arb (
        .clk        (clk),
        .reset      (reset),
        .bus_in_if  (worker_smem_bus_if),
        .bus_out_if (smem_bus_if)
    );

    // ================================================================
    // Busy / perf
    // ================================================================
    wire [NUM_REQS-1:0] req_bus_valid;
    for (genvar i = 0; i < NUM_REQS; ++i) begin : g_req_valid
        assign req_bus_valid[i] = req_bus_if[i].req_valid;
    end

    wire [NUM_WORKERS-1:0] worker_idle;
    for (genvar i = 0; i < `VX_CFG_NUM_DXA_CORES; ++i) begin : g_worker_idle
        assign worker_idle[i] = worker_req_if[i].ready;
    end
`ifdef VX_CFG_TSU_ENABLE
    for (genvar i = 0; i < NUM_STR_WORKERS; ++i) begin : g_tsu_worker_idle
        assign worker_idle[NUM_LINE_WORKERS + i] = tsu_worker_req_if[i].ready;
    end
`endif

    reg dxa_busy_r;
    always @(posedge clk) begin
        if (reset)
            dxa_busy_r <= 1'b0;
        else
            dxa_busy_r <= dcr_bus_if.req_valid | (|req_bus_valid) | ~(&worker_idle);
    end
    assign busy = dxa_busy_r | dcr_bus_if.req_valid | (|req_bus_valid);

`ifdef PERF_ENABLE
    always_comb begin
        dxa_perf = '0;
        for (int w = 0; w < NUM_WORKERS; ++w) begin
            dxa_perf.transfers   += worker_dxa_perf[w].transfers;
            dxa_perf.gmem_reads  += worker_dxa_perf[w].gmem_reads;
            dxa_perf.gmem_dedup  += worker_dxa_perf[w].gmem_dedup;
            dxa_perf.lmem_writes += worker_dxa_perf[w].lmem_writes;
            dxa_perf.gmem_latency += worker_dxa_perf[w].gmem_latency;
            dxa_perf.noslot_stalls += worker_dxa_perf[w].noslot_stalls;
            dxa_perf.tsu_bursts  += worker_dxa_perf[w].tsu_bursts;
            dxa_perf.tsu_bytes   += worker_dxa_perf[w].tsu_bytes;
        end
    end
`endif

`ifdef DBG_TRACE_DXA
    // Interface-array indexing must be elaboration-time constant; use a
    // genvar loop instead of an integer loop.
    for (genvar w = 0; w < `VX_CFG_NUM_DXA_CORES; ++w) begin : g_trace_worker
        always @(posedge clk) begin
            if (~reset && worker_req_if[w].valid) begin
                `TRACE(1, ("%t: %s dispatch-issue: worker=%0d, core=%0d, wid=%0d, meta=0x%0h\n",
                    $time, INSTANCE_ID, w,
                    worker_req_if[w].req_data.core_id,
                    worker_req_if[w].req_data.wid,
                    worker_req_if[w].req_data.meta))
            end
        end
    end
`endif

endmodule
