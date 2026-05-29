// Copyright 2024
// TLB: banked CAM-based address translation.
//
// The TLB is partitioned into TLB_NUM_BANKS independent banks, each an instance
// of VX_mmu_tlb_bank holding TLB_SIZE/TLB_NUM_BANKS entries with its own CAM,
// replacement state, and lookup/miss/replay FSM. Incoming lane requests are
// distributed across banks by low-order VPN bits via a crossbar, so up to
// TLB_NUM_BANKS translations can proceed in the same cycle and a miss stalled in
// one bank does not block hits in the others.
//
// TLB_NUM_BANKS=1 reproduces the original single-ported (fully serialized) TLB.
//
// Shared PTW caveat: the device-level PTW exposes a single requestor port per
// TLB and its fill path assumes at most one outstanding walk per requestor.
// Banking the lookup datapath would otherwise let multiple banks issue walks
// concurrently, so the miss/fill handshakes are serialized here onto that one
// port (one outstanding walk per TLB at a time). Hits remain fully concurrent;
// only the (rare) misses serialize. Per-bank PTW ports are a future extension.

`include "VX_define.vh"
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off UNUSEDSIGNAL */

module VX_mmu_tlb import VX_gpu_pkg::*; #(
    parameter NUM_REQS       = DCACHE_NUM_REQS,
    parameter DATA_SIZE      = DCACHE_WORD_SIZE,
    parameter TAG_WIDTH_IN   = DCACHE_TAG_WIDTH,
    parameter TAG_WIDTH_OUT  = TAG_WIDTH_IN + `UP(`CLOG2(NUM_REQS)),
    parameter ADDR_WIDTH     = DCACHE_ADDR_WIDTH,
    parameter FLAGS_WIDTH    = MEM_FLAGS_WIDTH
) (
    input wire clk,
    input wire reset,

    VX_mem_bus_if.slave  tlb_in_if [NUM_REQS],
    VX_mem_bus_if.master tlb_out_if [NUM_REQS],

    output wire            miss_valid,
    input  wire            miss_ready,
    output wire [`XLEN-1:0] miss_vaddr,

    input  wire            fill_valid,
    output wire            fill_ready,
    input  wire [`XLEN-1:0] fill_vaddr,
    input  wire [`XLEN-1:0] fill_paddr,
    input  wire [7:0]       fill_flags,

`ifdef PERF_ENABLE
    output mmu_perf_t    mmu_perf
`else
    output wire          mmu_perf_placeholder
`endif
);

    // =========================================================================
    // Local Parameters
    // =========================================================================

    localparam DATA_WIDTH    = DATA_SIZE * 8;
    localparam REQ_DATAW_IN  = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_SIZE + FLAGS_WIDTH + TAG_WIDTH_IN;
    localparam REQ_DATAW_OUT = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_SIZE + FLAGS_WIDTH + TAG_WIDTH_OUT;
    localparam RSP_DATAW_IN  = DATA_WIDTH + TAG_WIDTH_IN;
    localparam RSP_DATAW_OUT = DATA_WIDTH + TAG_WIDTH_OUT;
    localparam SOURCE_BITS   = `UP(`CLOG2(NUM_REQS));

    localparam PAGE_OFFSET_BITS = 12 - `CLOG2(DATA_SIZE);

    // Banking
    localparam TLB_SIZE        = 128;                 // total entries across all banks
    localparam NUM_BANKS       = `TLB_NUM_BANKS;
    localparam BANK_SIZE       = TLB_SIZE / NUM_BANKS; // entries per bank
    localparam BANK_SEL_BITS   = `CLOG2(NUM_BANKS);
    localparam BANK_SEL_WIDTH  = `UP(BANK_SEL_BITS);
    localparam BANK_IDX_BITS   = `UP(`CLOG2(NUM_BANKS));

    `STATIC_ASSERT(NUM_BANKS == (1 << `CLOG2(NUM_BANKS)), ("VX_mmu_tlb: TLB_NUM_BANKS must be a power of 2"))
    `STATIC_ASSERT((TLB_SIZE % NUM_BANKS) == 0, ("VX_mmu_tlb: TLB_SIZE must be divisible by TLB_NUM_BANKS"))
    `STATIC_ASSERT(NUM_BANKS <= TLB_SIZE, ("VX_mmu_tlb: TLB_NUM_BANKS must be <= TLB_SIZE"))

    // =========================================================================
    // Request packing + per-lane bank selection
    // =========================================================================

    wire [NUM_REQS-1:0]                   req_valid_in;
    wire [NUM_REQS-1:0][REQ_DATAW_IN-1:0] req_data_in;
    wire [NUM_REQS-1:0]                   req_ready_in;
    wire [NUM_REQS-1:0][BANK_SEL_WIDTH-1:0] req_bank_sel;

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_req_in
        assign req_valid_in[i] = tlb_in_if[i].req_valid;
        assign req_data_in[i]  = {
            tlb_in_if[i].req_data.rw,
            tlb_in_if[i].req_data.addr,
            tlb_in_if[i].req_data.data,
            tlb_in_if[i].req_data.byteen,
            tlb_in_if[i].req_data.flags[FLAGS_WIDTH-1:0],
            tlb_in_if[i].req_data.tag[TAG_WIDTH_IN-1:0]
        };
        assign tlb_in_if[i].req_ready = req_ready_in[i];

        // Bank select: low-order VPN bits (word-addr bit PAGE_OFFSET_BITS == byte-addr bit 12).
        if (NUM_BANKS > 1) begin : g_bank_sel
            assign req_bank_sel[i] = tlb_in_if[i].req_data.addr[PAGE_OFFSET_BITS +: BANK_SEL_BITS];
        end else begin : g_single_bank
            assign req_bank_sel[i] = '0;
        end
    end

    // =========================================================================
    // Request distribution crossbar (NUM_REQS -> NUM_BANKS)
    // =========================================================================

    wire [NUM_BANKS-1:0]                   bank_req_valid;
    wire [NUM_BANKS-1:0][REQ_DATAW_IN-1:0] bank_req_data;
    wire [NUM_BANKS-1:0][SOURCE_BITS-1:0]  bank_req_sel;   // originating LSU lane
    wire [NUM_BANKS-1:0]                   bank_req_ready;

    VX_stream_xbar #(
        .NUM_INPUTS  (NUM_REQS),
        .NUM_OUTPUTS (NUM_BANKS),
        .DATAW       (REQ_DATAW_IN),
        .ARBITER     ("R"),
        .OUT_BUF     (0)
    ) req_dist_xbar (
        .clk       (clk),
        .reset     (reset),
        `UNUSED_PIN (collisions),
        .valid_in  (req_valid_in),
        .data_in   (req_data_in),
        .sel_in    (req_bank_sel),
        .ready_in  (req_ready_in),
        .valid_out (bank_req_valid),
        .data_out  (bank_req_data),
        .sel_out   (bank_req_sel),
        .ready_out (bank_req_ready)
    );

    // =========================================================================
    // TLB banks
    // =========================================================================

    wire [NUM_BANKS-1:0]                    bank_out_valid;
    wire [NUM_BANKS-1:0][REQ_DATAW_OUT-1:0] bank_out_data;
    wire [NUM_BANKS-1:0]                    bank_out_ready;

    wire [NUM_BANKS-1:0] bank_miss_valid;
    wire [NUM_BANKS-1:0] bank_miss_ready;
    wire [`XLEN-1:0]     bank_miss_vaddr [NUM_BANKS];
    wire [NUM_BANKS-1:0] bank_fill_valid;
    wire [NUM_BANKS-1:0] bank_fill_ready;

`ifdef PERF_ENABLE
    wire [NUM_BANKS-1:0][PERF_CTR_BITS-1:0] bank_perf_reads;
    wire [NUM_BANKS-1:0][PERF_CTR_BITS-1:0] bank_perf_hits;
    wire [NUM_BANKS-1:0][PERF_CTR_BITS-1:0] bank_perf_misses;
    wire [NUM_BANKS-1:0][PERF_CTR_BITS-1:0] bank_perf_evictions;
    wire [NUM_BANKS-1:0][PERF_CTR_BITS-1:0] bank_perf_walks;
`endif

    for (genvar b = 0; b < NUM_BANKS; b++) begin : g_banks
        VX_mmu_tlb_bank #(
            .DATA_SIZE     (DATA_SIZE),
            .ADDR_WIDTH    (ADDR_WIDTH),
            .TAG_WIDTH_IN  (TAG_WIDTH_IN),
            .SOURCE_BITS   (SOURCE_BITS),
            .FLAGS_WIDTH   (FLAGS_WIDTH),
            .TLB_SIZE      (BANK_SIZE),
            .TAG_WIDTH_OUT (TAG_WIDTH_OUT)
        ) bank (
            .clk        (clk),
            .reset      (reset),
            .req_valid  (bank_req_valid[b]),
            .req_data   (bank_req_data[b]),
            .req_sel    (bank_req_sel[b]),
            .req_ready  (bank_req_ready[b]),
            .out_valid  (bank_out_valid[b]),
            .out_data   (bank_out_data[b]),
            .out_ready  (bank_out_ready[b]),
            .miss_valid (bank_miss_valid[b]),
            .miss_ready (bank_miss_ready[b]),
            .miss_vaddr (bank_miss_vaddr[b]),
            .fill_valid (bank_fill_valid[b]),
            .fill_ready (bank_fill_ready[b]),
            .fill_vaddr (fill_vaddr),
            .fill_paddr (fill_paddr),
            .fill_flags (fill_flags)
        `ifdef PERF_ENABLE
            ,.perf_tlb_reads     (bank_perf_reads[b])
            ,.perf_tlb_hits      (bank_perf_hits[b])
            ,.perf_tlb_misses    (bank_perf_misses[b])
            ,.perf_tlb_evictions (bank_perf_evictions[b])
            ,.perf_ptw_walks     (bank_perf_walks[b])
        `else
            ,`UNUSED_PIN (perf_placeholder)
        `endif
        );
    end

    // =========================================================================
    // Per-bank output buffer
    // =========================================================================
    // Decouples each bank's combinational hit pass-through (out_valid depends on
    // the input handshake) from the gather crossbar's arbiter (whose ready
    // depends on valid). Without this skid buffer the two form a circular
    // combinational path. The original single-TLB used a non-arbitrating switch
    // on output and so did not need this.

    wire [NUM_BANKS-1:0]                    bank_buf_valid;
    wire [NUM_BANKS-1:0][REQ_DATAW_OUT-1:0] bank_buf_data;
    wire [NUM_BANKS-1:0]                    bank_buf_ready;

    for (genvar b = 0; b < NUM_BANKS; b++) begin : g_bank_out_buf
        VX_elastic_buffer #(
            .DATAW   (REQ_DATAW_OUT),
            .SIZE    (2),
            .OUT_REG (0)
        ) bank_out_buf (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (bank_out_valid[b]),
            .data_in   (bank_out_data[b]),
            .ready_in  (bank_out_ready[b]),
            .valid_out (bank_buf_valid[b]),
            .data_out  (bank_buf_data[b]),
            .ready_out (bank_buf_ready[b])
        );
    end

    // =========================================================================
    // Output gather crossbar (NUM_BANKS -> NUM_REQS), keyed by source lane
    // =========================================================================

    wire [NUM_BANKS-1:0][SOURCE_BITS-1:0] bank_out_sel;
    for (genvar b = 0; b < NUM_BANKS; b++) begin : g_out_sel
        // Source lane is encoded in the low SOURCE_BITS of the outgoing tag.
        assign bank_out_sel[b] = bank_buf_data[b][SOURCE_BITS-1:0];
    end

    wire [NUM_REQS-1:0]                    out_valid_w;
    wire [NUM_REQS-1:0][REQ_DATAW_OUT-1:0] out_data_w;
    wire [NUM_REQS-1:0]                    out_ready_w;

    VX_stream_xbar #(
        .NUM_INPUTS  (NUM_BANKS),
        .NUM_OUTPUTS (NUM_REQS),
        .DATAW       (REQ_DATAW_OUT),
        .ARBITER     ("R"),
        .OUT_BUF     (2)
    ) req_gather_xbar (
        .clk       (clk),
        .reset     (reset),
        `UNUSED_PIN (collisions),
        .valid_in  (bank_buf_valid),
        .data_in   (bank_buf_data),
        .sel_in    (bank_out_sel),
        .ready_in  (bank_buf_ready),
        .valid_out (out_valid_w),
        .data_out  (out_data_w),
        `UNUSED_PIN (sel_out),
        .ready_out (out_ready_w)
    );

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_req_out
        assign tlb_out_if[i].req_valid       = out_valid_w[i];
        assign tlb_out_if[i].req_data.rw     = out_data_w[i][REQ_DATAW_OUT-1];
        assign tlb_out_if[i].req_data.addr   = out_data_w[i][REQ_DATAW_OUT-2 -: ADDR_WIDTH];
        assign tlb_out_if[i].req_data.data   = out_data_w[i][REQ_DATAW_OUT-2-ADDR_WIDTH -: DATA_WIDTH];
        assign tlb_out_if[i].req_data.byteen = out_data_w[i][REQ_DATAW_OUT-2-ADDR_WIDTH-DATA_WIDTH -: DATA_SIZE];
        assign tlb_out_if[i].req_data.flags  = out_data_w[i][REQ_DATAW_OUT-2-ADDR_WIDTH-DATA_WIDTH-DATA_SIZE -: FLAGS_WIDTH];
        assign tlb_out_if[i].req_data.tag    = out_data_w[i][TAG_WIDTH_OUT-1:0];
        assign out_ready_w[i]                = tlb_out_if[i].req_ready;
    end

    // =========================================================================
    // Miss/Fill serialization onto the single shared PTW requestor port
    // =========================================================================

    reg                     ptw_inflight;
    reg [BANK_IDX_BITS-1:0]  ptw_owner;
    reg [BANK_IDX_BITS-1:0]  miss_rr;

    reg [BANK_IDX_BITS-1:0]  miss_sel_bank;
    reg                      miss_sel_valid;
    always_comb begin
        miss_sel_bank  = '0;
        miss_sel_valid = 1'b0;
        for (int i = 0; i < NUM_BANKS; i++) begin
            int idx;
            idx = (int'(miss_rr) + i) % NUM_BANKS;
            if (bank_miss_valid[idx] && !miss_sel_valid) begin
                miss_sel_bank  = BANK_IDX_BITS'(idx);
                miss_sel_valid = 1'b1;
            end
        end
    end

    // Drive the shared PTW miss port from the selected bank (only when no walk is in flight).
    assign miss_valid = miss_sel_valid && !ptw_inflight;
    assign miss_vaddr = bank_miss_vaddr[miss_sel_bank];

    for (genvar b = 0; b < NUM_BANKS; b++) begin : g_bank_miss_ready
        assign bank_miss_ready[b] = miss_valid && miss_ready && (miss_sel_bank == BANK_IDX_BITS'(b));
    end

    // Route the fill back to the owning bank.
    for (genvar b = 0; b < NUM_BANKS; b++) begin : g_bank_fill
        assign bank_fill_valid[b] = fill_valid && ptw_inflight && (ptw_owner == BANK_IDX_BITS'(b));
    end
    assign fill_ready = ptw_inflight && bank_fill_ready[ptw_owner];

    always_ff @(posedge clk) begin
        if (reset) begin
            ptw_inflight <= 1'b0;
            ptw_owner    <= '0;
            miss_rr      <= '0;
        end else begin
            if (!ptw_inflight && miss_valid && miss_ready) begin
                ptw_inflight <= 1'b1;
                ptw_owner    <= miss_sel_bank;
                miss_rr      <= BANK_IDX_BITS'((int'(miss_sel_bank) + 1) % NUM_BANKS);
            end else if (ptw_inflight && fill_valid && fill_ready) begin
                ptw_inflight <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Response Serialize (NUM_REQS-to-1)
    // =========================================================================

    wire [NUM_REQS-1:0]                  rsp_valid_in;
    wire [NUM_REQS-1:0][RSP_DATAW_OUT-1:0] rsp_data_in;
    wire [NUM_REQS-1:0]                  rsp_ready_in;

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_rsp_in
        assign rsp_valid_in[i] = tlb_out_if[i].rsp_valid;
        assign rsp_data_in[i]  = {
            tlb_out_if[i].rsp_data.data,
            tlb_out_if[i].rsp_data.tag[TAG_WIDTH_OUT-1:0]
        };
        assign tlb_out_if[i].rsp_ready = rsp_ready_in[i];
    end

    wire                      ser_rsp_valid;
    wire [RSP_DATAW_OUT-1:0]  ser_rsp_data;
    wire                      ser_rsp_ready;

    VX_stream_arb #(
        .NUM_INPUTS  (NUM_REQS),
        .NUM_OUTPUTS (1),
        .DATAW       (RSP_DATAW_OUT),
        .ARBITER     ("R"),
        .OUT_BUF     (0)
    ) rsp_serialize_arb (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (rsp_valid_in),
        .data_in   (rsp_data_in),
        .ready_in  (rsp_ready_in),
        .valid_out (ser_rsp_valid),
        .data_out  (ser_rsp_data),
        `UNUSED_PIN (sel_out),
        .ready_out (ser_rsp_ready)
    );

    // =========================================================================
    // Response Deserialize (1-to-NUM_REQS)
    // =========================================================================

    wire [TAG_WIDTH_OUT-1:0] ser_rsp_tag = ser_rsp_data[TAG_WIDTH_OUT-1:0];
    wire [SOURCE_BITS-1:0]   rsp_source;
    wire [TAG_WIDTH_IN-1:0]  rsp_tag_restored;

    VX_bits_remove #(
        .N   (TAG_WIDTH_OUT),
        .S   (SOURCE_BITS),
        .POS (0)
    ) rsp_tag_decode (
        .data_in  (ser_rsp_tag),
        .sel_out  (rsp_source),
        .data_out (rsp_tag_restored)
    );

    wire [RSP_DATAW_IN-1:0] ser_rsp_data_restored = {
        ser_rsp_data[RSP_DATAW_OUT-1:TAG_WIDTH_OUT],
        rsp_tag_restored
    };

    wire                     deser_rsp_ready;
    wire [NUM_REQS-1:0]      deser_rsp_valid_out;
    wire [NUM_REQS-1:0][RSP_DATAW_IN-1:0] deser_rsp_data_out;
    wire [NUM_REQS-1:0]      deser_rsp_ready_out;

    VX_stream_switch #(
        .NUM_INPUTS  (1),
        .NUM_OUTPUTS (NUM_REQS),
        .DATAW       (RSP_DATAW_IN),
        .OUT_BUF     (0)
    ) rsp_deserialize_switch (
        .clk       (clk),
        .reset     (reset),
        .sel_in    (rsp_source),
        .valid_in  (ser_rsp_valid),
        .data_in   (ser_rsp_data_restored),
        .ready_in  (deser_rsp_ready),
        .valid_out (deser_rsp_valid_out),
        .data_out  (deser_rsp_data_out),
        .ready_out (deser_rsp_ready_out)
    );

    assign ser_rsp_ready = deser_rsp_ready;

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_rsp_out
        assign tlb_in_if[i].rsp_valid = deser_rsp_valid_out[i];
        assign tlb_in_if[i].rsp_data.data = deser_rsp_data_out[i][RSP_DATAW_IN-1 -: DATA_WIDTH];
        assign tlb_in_if[i].rsp_data.tag  = deser_rsp_data_out[i][TAG_WIDTH_IN-1:0];
        assign deser_rsp_ready_out[i] = tlb_in_if[i].rsp_ready;
    end

    // =========================================================================
    // Performance Counters (aggregate across banks)
    // =========================================================================

`ifdef PERF_ENABLE
    reg [PERF_CTR_BITS-1:0] sum_reads;
    reg [PERF_CTR_BITS-1:0] sum_hits;
    reg [PERF_CTR_BITS-1:0] sum_misses;
    reg [PERF_CTR_BITS-1:0] sum_evictions;
    reg [PERF_CTR_BITS-1:0] sum_walks;

    always_comb begin
        sum_reads     = '0;
        sum_hits      = '0;
        sum_misses    = '0;
        sum_evictions = '0;
        sum_walks     = '0;
        for (int b = 0; b < NUM_BANKS; b++) begin
            sum_reads     += bank_perf_reads[b];
            sum_hits      += bank_perf_hits[b];
            sum_misses    += bank_perf_misses[b];
            sum_evictions += bank_perf_evictions[b];
            sum_walks     += bank_perf_walks[b];
        end
    end

    assign mmu_perf.tlb_reads     = sum_reads;
    assign mmu_perf.tlb_hits      = sum_hits;
    assign mmu_perf.tlb_misses    = sum_misses;
    assign mmu_perf.tlb_evictions = sum_evictions;
    assign mmu_perf.ptw_walks     = sum_walks;
    assign mmu_perf.ptw_latency   = '0; // overridden in VX_core with ptw_latency_in from socket PTW
    assign mmu_perf.pwc_hits      = '0; // overridden in VX_core with pwc_hits_in from socket PTW
    assign mmu_perf.pwc_misses    = '0; // overridden in VX_core with pwc_misses_in from socket PTW
    assign mmu_perf.pwc2_hits     = '0; // overridden in VX_core with pwc2_hits_in from socket PTW
    assign mmu_perf.pwc2_misses   = '0; // overridden in VX_core with pwc2_misses_in from socket PTW
`else
    assign mmu_perf_placeholder = 1'b0;
`endif

endmodule
