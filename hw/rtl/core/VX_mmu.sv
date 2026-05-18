// Copyright 2024
// MMU: TLB + PTW for VA→PA translation

`include "VX_define.vh"
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off UNUSEDSIGNAL */

module VX_mmu import VX_gpu_pkg::*; #(
    parameter NUM_REQS       = DCACHE_NUM_REQS,
    parameter DATA_SIZE      = DCACHE_WORD_SIZE,
    parameter TAG_WIDTH      = DCACHE_TAG_WIDTH_BASE,
    parameter MEM_ADDR_WIDTH = `MEM_ADDR_WIDTH,
    parameter ADDR_WIDTH     = MEM_ADDR_WIDTH - `CLOG2(DATA_SIZE),
    parameter FLAGS_WIDTH    = MEM_FLAGS_WIDTH,
    parameter EBUF_SIZE      = 2
) (
    input wire clk,
    input wire reset,

    input wire [`XLEN-1:0] satp,

    VX_mem_bus_if.slave  lsu_mem_if [NUM_REQS],
    VX_mem_bus_if.master dcache_mem_if [NUM_REQS],

    // PTW miss/fill interface (shared PTW lives at socket level)
    output wire          ptw_req_valid,
    input  wire          ptw_req_ready,
    output wire [`XLEN-1:0] ptw_req_vaddr,

    input  wire          ptw_rsp_valid,
    output wire          ptw_rsp_ready,
    input  wire [`XLEN-1:0] ptw_rsp_vaddr,
    input  wire [`XLEN-1:0] ptw_rsp_paddr,
    input  wire [7:0]    ptw_rsp_flags,

`ifdef PERF_ENABLE
    output mmu_perf_t    mmu_perf
`else
    output wire          mmu_perf_placeholder
`endif
);

    // =========================================================================
    // Bypass Control
    // =========================================================================

    localparam [`XLEN-1:0] IO_REGION_END = `XLEN'h00010000;
    localparam [`XLEN-1:0] STARTUP_ADDR  = `XLEN'h80000000;
    localparam [`XLEN-1:0] STARTUP_END   = `XLEN'h80040000;
    localparam [`XLEN-1:0] PT_BASE_ADDR  = `XLEN'hF0000000;

    function automatic logic needs_translation(input logic [`XLEN-1:0] full_addr);
`ifdef XLEN_32
        if (!satp[31]) return 1'b0;            // SV32 BARE: mode bit = satp[31]
`else
        if (satp[63:60] == 4'b0) return 1'b0;  // SV39 BARE: mode field = satp[63:60]
`endif
        if (full_addr < IO_REGION_END) return 1'b0;
        if (full_addr >= STARTUP_ADDR && full_addr <= STARTUP_END) return 1'b0;
        if (full_addr >= PT_BASE_ADDR) return 1'b0;
        return 1'b1;
    endfunction

    // =========================================================================
    // Local Parameters
    // =========================================================================

    localparam DATA_WIDTH      = DATA_SIZE * 8;
    localparam TLB_SOURCE_BITS = `UP(`CLOG2(NUM_REQS));
    localparam TAG_WIDTH_TLB   = TAG_WIDTH + TLB_SOURCE_BITS;
    // MERGE_TAG_WIDTH: tag width used inside the merge arbiter.
    // Equals TAG_WIDTH_TLB since the only paths are bypass and TLB.
    localparam MERGE_TAG_WIDTH = TAG_WIDTH_TLB;
    localparam REQ_DATAW       = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_SIZE + FLAGS_WIDTH + TAG_WIDTH;
    localparam RSP_DATAW       = DATA_WIDTH + TAG_WIDTH;

    // =========================================================================
    // Internal Interfaces
    // =========================================================================

    VX_mem_bus_if #(
        .DATA_SIZE   (DATA_SIZE),
        .TAG_WIDTH   (TAG_WIDTH),
        .FLAGS_WIDTH (FLAGS_WIDTH)
    ) buffered_if[NUM_REQS]();

    VX_mem_bus_if #(
        .DATA_SIZE   (DATA_SIZE),
        .TAG_WIDTH   (TAG_WIDTH_TLB),
        .FLAGS_WIDTH (FLAGS_WIDTH)
    ) tlb_out_if[NUM_REQS]();

    VX_mem_bus_if #(
        .DATA_SIZE   (DATA_SIZE),
        .TAG_WIDTH   (TAG_WIDTH_TLB),
        .FLAGS_WIDTH (FLAGS_WIDTH)
    ) bypass_dcache_if[NUM_REQS]();

`ifdef PERF_ENABLE
    /* verilator lint_off UNUSEDSIGNAL */
    mmu_perf_t mmu_perf_tlb;
    /* verilator lint_on UNUSEDSIGNAL */
`endif

    // [0..NUM_REQS-1]=bypass, [NUM_REQS..2*NUM_REQS-1]=TLB
    VX_mem_bus_if #(
        .DATA_SIZE   (DATA_SIZE),
        .TAG_WIDTH   (MERGE_TAG_WIDTH),
        .FLAGS_WIDTH (FLAGS_WIDTH)
    ) merge_in_if[2 * NUM_REQS]();

    // =========================================================================
    // Elastic Buffers (TLB path only)
    // =========================================================================

    wire [NUM_REQS-1:0] ebuf_req_ready;
    wire [NUM_REQS-1:0] ebuf_rsp_valid;
    wire [DATA_WIDTH-1:0] ebuf_rsp_data [NUM_REQS];
    wire [TAG_WIDTH-1:0]  ebuf_rsp_tag  [NUM_REQS];
    wire [NUM_REQS-1:0]   ebuf_rsp_ready;
    wire [NUM_REQS-1:0] lane_needs_trans_ebuf;

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_elastic_buffers

        wire [`XLEN-1:0] full_addr_ebuf = `XLEN'({lsu_mem_if[i].req_data.addr, {`CLOG2(DATA_SIZE){1'b0}}});
        assign lane_needs_trans_ebuf[i] = needs_translation(full_addr_ebuf);

        wire [REQ_DATAW-1:0] req_data_in_packed;
        wire [REQ_DATAW-1:0] req_data_out_packed;

        assign req_data_in_packed = {
            lsu_mem_if[i].req_data.rw,
            lsu_mem_if[i].req_data.addr,
            lsu_mem_if[i].req_data.data,
            lsu_mem_if[i].req_data.byteen,
            lsu_mem_if[i].req_data.flags[FLAGS_WIDTH-1:0],
            lsu_mem_if[i].req_data.tag[TAG_WIDTH-1:0]
        };

        VX_elastic_buffer #(
            .DATAW  (REQ_DATAW),
            .SIZE   (EBUF_SIZE),
            .OUT_REG(0)
        ) req_buffer (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (lsu_mem_if[i].req_valid && lane_needs_trans_ebuf[i]),
            .data_in   (req_data_in_packed),
            .ready_in  (ebuf_req_ready[i]),
            .valid_out (buffered_if[i].req_valid),
            .data_out  (req_data_out_packed),
            .ready_out (buffered_if[i].req_ready)
        );

        assign buffered_if[i].req_data.rw     = req_data_out_packed[REQ_DATAW-1];
        assign buffered_if[i].req_data.addr   = req_data_out_packed[REQ_DATAW-2 -: ADDR_WIDTH];
        assign buffered_if[i].req_data.data   = req_data_out_packed[REQ_DATAW-2-ADDR_WIDTH -: DATA_WIDTH];
        assign buffered_if[i].req_data.byteen = req_data_out_packed[REQ_DATAW-2-ADDR_WIDTH-DATA_WIDTH -: DATA_SIZE];
        assign buffered_if[i].req_data.flags  = req_data_out_packed[REQ_DATAW-2-ADDR_WIDTH-DATA_WIDTH-DATA_SIZE -: FLAGS_WIDTH];
        assign buffered_if[i].req_data.tag    = req_data_out_packed[TAG_WIDTH-1:0];

        wire [RSP_DATAW-1:0] rsp_data_in_packed;
        wire [RSP_DATAW-1:0] rsp_data_out_packed;

        assign rsp_data_in_packed = {
            buffered_if[i].rsp_data.data,
            buffered_if[i].rsp_data.tag[TAG_WIDTH-1:0]
        };

        VX_elastic_buffer #(
            .DATAW  (RSP_DATAW),
            .SIZE   (EBUF_SIZE),
            .OUT_REG(0)
        ) rsp_buffer (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (buffered_if[i].rsp_valid),
            .data_in   (rsp_data_in_packed),
            .ready_in  (buffered_if[i].rsp_ready),
            .valid_out (ebuf_rsp_valid[i]),
            .data_out  (rsp_data_out_packed),
            .ready_out (ebuf_rsp_ready[i])
        );

        assign ebuf_rsp_data[i] = rsp_data_out_packed[RSP_DATAW-1 -: DATA_WIDTH];
        assign ebuf_rsp_tag[i]  = rsp_data_out_packed[TAG_WIDTH-1:0];

    end

    // =========================================================================
    // TLB Module
    // =========================================================================

    VX_mmu_tlb #(
        .NUM_REQS      (NUM_REQS),
        .DATA_SIZE     (DATA_SIZE),
        .TAG_WIDTH_IN  (TAG_WIDTH),
        .TAG_WIDTH_OUT (TAG_WIDTH_TLB),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .FLAGS_WIDTH   (FLAGS_WIDTH)
    ) tlb_unit (
        .clk           (clk),
        .reset         (reset),
        .tlb_in_if     (buffered_if),
        .tlb_out_if    (tlb_out_if),
        .miss_valid    (ptw_req_valid),
        .miss_ready    (ptw_req_ready),
        .miss_vaddr    (ptw_req_vaddr),
        .fill_valid    (ptw_rsp_valid),
        .fill_ready    (ptw_rsp_ready),
        .fill_vaddr    (ptw_rsp_vaddr),
        .fill_paddr    (ptw_rsp_paddr),
        .fill_flags    (ptw_rsp_flags),
    `ifdef PERF_ENABLE
        .mmu_perf      (mmu_perf_tlb)
    `else
        `UNUSED_PIN (mmu_perf_placeholder)
    `endif
    );

    // PTW is now shared at socket level; miss/fill ports are exposed externally.

    // =========================================================================
    // Bypass Path
    // =========================================================================

    wire [NUM_REQS-1:0] lane_needs_trans;
    wire [NUM_REQS-1:0] lane_bypass;

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_bypass_path
        wire [`XLEN-1:0] full_addr = `XLEN'({lsu_mem_if[i].req_data.addr, {`CLOG2(DATA_SIZE){1'b0}}});

        assign lane_needs_trans[i] = needs_translation(full_addr);
        assign lane_bypass[i] = ~lane_needs_trans[i];

        assign bypass_dcache_if[i].req_valid = lsu_mem_if[i].req_valid && lane_bypass[i];
        assign bypass_dcache_if[i].req_data.rw     = lsu_mem_if[i].req_data.rw;
        assign bypass_dcache_if[i].req_data.addr   = lsu_mem_if[i].req_data.addr;
        assign bypass_dcache_if[i].req_data.data   = lsu_mem_if[i].req_data.data;
        assign bypass_dcache_if[i].req_data.byteen = lsu_mem_if[i].req_data.byteen;
        assign bypass_dcache_if[i].req_data.flags  = lsu_mem_if[i].req_data.flags;
        assign bypass_dcache_if[i].req_data.tag = {lsu_mem_if[i].req_data.tag[TAG_WIDTH-1:0],
                                                   {TLB_SOURCE_BITS{1'b0}}};
    end

    // =========================================================================
    // Merge Arbiter Inputs
    // =========================================================================

    // Field-by-field assignment required when MERGE_TAG_WIDTH != TAG_WIDTH_TLB.
    // Bulk struct assignment would zero-extend/truncate at the MSB of the packed
    // representation (which is the rw field), corrupting all fields.
    // For the req tag: cast to MERGE_TAG_WIDTH (zero-extends at MSB when wider).
    // For the rsp tag: extract lower TAG_WIDTH_TLB bits (drops the zero padding).
    for (genvar i = 0; i < NUM_REQS; i++) begin : g_bypass_to_merge
        assign merge_in_if[i].req_valid            = bypass_dcache_if[i].req_valid;
        assign merge_in_if[i].req_data.rw          = bypass_dcache_if[i].req_data.rw;
        assign merge_in_if[i].req_data.addr        = bypass_dcache_if[i].req_data.addr;
        assign merge_in_if[i].req_data.data        = bypass_dcache_if[i].req_data.data;
        assign merge_in_if[i].req_data.byteen      = bypass_dcache_if[i].req_data.byteen;
        assign merge_in_if[i].req_data.flags       = bypass_dcache_if[i].req_data.flags;
        assign merge_in_if[i].req_data.tag         = MERGE_TAG_WIDTH'(bypass_dcache_if[i].req_data.tag);
        assign bypass_dcache_if[i].req_ready       = merge_in_if[i].req_ready;

        assign bypass_dcache_if[i].rsp_valid       = merge_in_if[i].rsp_valid;
        assign bypass_dcache_if[i].rsp_data.data   = merge_in_if[i].rsp_data.data;
        assign bypass_dcache_if[i].rsp_data.tag    = TAG_WIDTH_TLB'(merge_in_if[i].rsp_data.tag);
        assign merge_in_if[i].rsp_ready            = bypass_dcache_if[i].rsp_ready;
    end

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_tlb_to_merge
        assign merge_in_if[NUM_REQS + i].req_valid            = tlb_out_if[i].req_valid;
        assign merge_in_if[NUM_REQS + i].req_data.rw          = tlb_out_if[i].req_data.rw;
        assign merge_in_if[NUM_REQS + i].req_data.addr        = tlb_out_if[i].req_data.addr;
        assign merge_in_if[NUM_REQS + i].req_data.data        = tlb_out_if[i].req_data.data;
        assign merge_in_if[NUM_REQS + i].req_data.byteen      = tlb_out_if[i].req_data.byteen;
        assign merge_in_if[NUM_REQS + i].req_data.flags       = tlb_out_if[i].req_data.flags;
        assign merge_in_if[NUM_REQS + i].req_data.tag         = MERGE_TAG_WIDTH'(tlb_out_if[i].req_data.tag);
        assign tlb_out_if[i].req_ready                        = merge_in_if[NUM_REQS + i].req_ready;

        assign tlb_out_if[i].rsp_valid                        = merge_in_if[NUM_REQS + i].rsp_valid;
        assign tlb_out_if[i].rsp_data.data                    = merge_in_if[NUM_REQS + i].rsp_data.data;
        assign tlb_out_if[i].rsp_data.tag                     = TAG_WIDTH_TLB'(merge_in_if[NUM_REQS + i].rsp_data.tag);
        assign merge_in_if[NUM_REQS + i].rsp_ready            = tlb_out_if[i].rsp_ready;
    end

    // =========================================================================
    // Merge Arbiter
    // =========================================================================

    VX_mem_arb #(
        .NUM_INPUTS     (2 * NUM_REQS),
        .NUM_OUTPUTS    (NUM_REQS),
        .DATA_SIZE      (DATA_SIZE),
        .TAG_WIDTH      (MERGE_TAG_WIDTH),
        .TAG_SEL_IDX    (MERGE_TAG_WIDTH),
        .ARBITER        ("R"),
        .MEM_ADDR_WIDTH (MEM_ADDR_WIDTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .FLAGS_WIDTH    (FLAGS_WIDTH),
        .REQ_OUT_BUF    (2),
        .RSP_OUT_BUF    (2)
    ) merge_arb (
        .clk        (clk),
        .reset      (reset),
        .bus_in_if  (merge_in_if),
        .bus_out_if (dcache_mem_if)
    );

    // =========================================================================
    // LSU Interface
    // =========================================================================

    for (genvar i = 0; i < NUM_REQS; i++) begin : g_lsu_if

        assign lsu_mem_if[i].req_ready = lane_needs_trans_ebuf[i] ?
            ebuf_req_ready[i] : bypass_dcache_if[i].req_ready;

        localparam LSU_RSP_DATAW = DATA_WIDTH + TAG_WIDTH;

        wire [TAG_WIDTH-1:0] bypass_rsp_tag_restored =
            bypass_dcache_if[i].rsp_data.tag[TAG_WIDTH_TLB-1:TLB_SOURCE_BITS];
        wire [LSU_RSP_DATAW-1:0] bypass_rsp_packed = {
            bypass_dcache_if[i].rsp_data.data,
            bypass_rsp_tag_restored
        };

        wire [LSU_RSP_DATAW-1:0] ebuf_rsp_packed = {
            ebuf_rsp_data[i],
            ebuf_rsp_tag[i]
        };

        wire [1:0] rsp_arb_valid_in = {ebuf_rsp_valid[i], bypass_dcache_if[i].rsp_valid};
        wire [1:0][LSU_RSP_DATAW-1:0] rsp_arb_data_in = {ebuf_rsp_packed, bypass_rsp_packed};
        wire [1:0] rsp_arb_ready_in;
        wire rsp_arb_valid_out;
        wire [LSU_RSP_DATAW-1:0] rsp_arb_data_out;

        VX_stream_arb #(
            .NUM_INPUTS  (2),
            .NUM_OUTPUTS (1),
            .DATAW       (LSU_RSP_DATAW),
            .ARBITER     ("R"),
            .OUT_BUF     (0)
        ) rsp_arb (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (rsp_arb_valid_in),
            .data_in   (rsp_arb_data_in),
            .ready_in  (rsp_arb_ready_in),
            .valid_out (rsp_arb_valid_out),
            .data_out  (rsp_arb_data_out),
            .ready_out (lsu_mem_if[i].rsp_ready),
            `UNUSED_PIN (sel_out)
        );

        assign bypass_dcache_if[i].rsp_ready = rsp_arb_ready_in[0];
        assign ebuf_rsp_ready[i] = rsp_arb_ready_in[1];

        assign lsu_mem_if[i].rsp_valid = rsp_arb_valid_out;
        assign lsu_mem_if[i].rsp_data.data = rsp_arb_data_out[LSU_RSP_DATAW-1 -: DATA_WIDTH];
        assign lsu_mem_if[i].rsp_data.tag = rsp_arb_data_out[TAG_WIDTH-1:0];

    end

    // =========================================================================
    // Performance Counters
    // =========================================================================

`ifdef PERF_ENABLE
    assign mmu_perf.tlb_reads     = mmu_perf_tlb.tlb_reads;
    assign mmu_perf.tlb_hits      = mmu_perf_tlb.tlb_hits;
    assign mmu_perf.tlb_misses    = mmu_perf_tlb.tlb_misses;
    assign mmu_perf.tlb_evictions = mmu_perf_tlb.tlb_evictions;
    assign mmu_perf.ptw_walks     = mmu_perf_tlb.ptw_walks;
    assign mmu_perf.ptw_latency   = '0; // overridden in VX_core with ptw_latency_in from socket PTW
    assign mmu_perf.pwc_hits      = '0; // overridden in VX_core with pwc_hits_in from socket PTW
    assign mmu_perf.pwc_misses    = '0; // overridden in VX_core with pwc_misses_in from socket PTW
    assign mmu_perf.pwc2_hits     = '0; // overridden in VX_core with pwc2_hits_in from socket PTW
    assign mmu_perf.pwc2_misses   = '0; // overridden in VX_core with pwc2_misses_in from socket PTW
`else
    assign mmu_perf_placeholder = 1'b0;
`endif

endmodule
