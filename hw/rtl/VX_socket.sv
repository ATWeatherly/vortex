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

module VX_socket import VX_gpu_pkg::*; #(
    parameter SOCKET_ID = 0,
    parameter `STRING INSTANCE_ID = ""
) (
    `SCOPE_IO_DECL

    // Clock
    input wire              clk,
    input wire              reset,

`ifdef PERF_ENABLE
    input sysmem_perf_t     sysmem_perf,
`endif

    // DCRs
    VX_dcr_bus_if.slave     dcr_bus_if,

    // Memory
    VX_mem_bus_if.master    mem_bus_if [`L1_MEM_PORTS],

`ifdef GBAR_ENABLE
    // Barrier
    VX_gbar_bus_if.master   gbar_bus_if,
`endif

`ifdef VM_ENABLE
    // PTW miss requests from all TLBs — routed up to device-level shared PTW
    output wire [`SOCKET_SIZE*2-1:0] ptw_miss_valid,
    output wire [`XLEN-1:0]          ptw_miss_vaddr [`SOCKET_SIZE*2],
    input  wire [`SOCKET_SIZE*2-1:0] ptw_miss_ready,
    // PTW fill responses coming back down from device-level shared PTW
    input  wire [`SOCKET_SIZE*2-1:0] ptw_fill_valid,
    output wire [`SOCKET_SIZE*2-1:0] ptw_fill_ready,
    input  wire [`XLEN-1:0]          ptw_fill_vaddr [`SOCKET_SIZE*2],
    input  wire [`XLEN-1:0]          ptw_fill_paddr [`SOCKET_SIZE*2],
    input  wire [7:0]                ptw_fill_flags [`SOCKET_SIZE*2],
`endif

`ifdef PERF_ENABLE
`ifdef VM_ENABLE
    // PTW perf counters from device-level PTW, passed down to all cores
    input wire [PERF_CTR_BITS-1:0]  ptw_latency_in,
    input wire [PERF_CTR_BITS-1:0]  pwc_hits_in,
    input wire [PERF_CTR_BITS-1:0]  pwc_misses_in,
    input wire [PERF_CTR_BITS-1:0]  pwc2_hits_in,
    input wire [PERF_CTR_BITS-1:0]  pwc2_misses_in,
`endif
`endif

    // Status
    output wire             busy
);

`ifdef SCOPE
    localparam scope_core = 0;
    `SCOPE_IO_SWITCH (`SOCKET_SIZE);
`endif

`ifdef GBAR_ENABLE
    VX_gbar_bus_if per_core_gbar_bus_if[`SOCKET_SIZE]();

    VX_gbar_arb #(
        .NUM_REQS (`SOCKET_SIZE),
        .OUT_BUF  ((`SOCKET_SIZE > 1) ? 2 : 0)
    ) gbar_arb (
        .clk        (clk),
        .reset      (reset),
        .bus_in_if  (per_core_gbar_bus_if),
        .bus_out_if (gbar_bus_if)
    );
`endif

    ///////////////////////////////////////////////////////////////////////////

`ifdef PERF_ENABLE
    cache_perf_t icache_perf, dcache_perf;
    sysmem_perf_t sysmem_perf_tmp;
    always @(*) begin
        sysmem_perf_tmp = sysmem_perf;
        sysmem_perf_tmp.icache = icache_perf;
        sysmem_perf_tmp.dcache = dcache_perf;
    end
`endif

    ///////////////////////////////////////////////////////////////////////////

    VX_mem_bus_if #(
        .DATA_SIZE (ICACHE_WORD_SIZE),
        .TAG_WIDTH (ICACHE_TAG_WIDTH)
    ) per_core_icache_bus_if[`SOCKET_SIZE]();

    VX_mem_bus_if #(
        .DATA_SIZE (ICACHE_LINE_SIZE),
        .TAG_WIDTH (ICACHE_MEM_TAG_WIDTH)
    ) icache_mem_bus_if[1]();

    `RESET_RELAY (icache_reset, reset);

    VX_cache_cluster #(
        .INSTANCE_ID    (`SFORMATF(("%s-icache", INSTANCE_ID))),
        .NUM_UNITS      (`NUM_ICACHES),
        .NUM_INPUTS     (`SOCKET_SIZE),
        .TAG_SEL_IDX    (0),
        .CACHE_SIZE     (`ICACHE_SIZE),
        .LINE_SIZE      (ICACHE_LINE_SIZE),
        .NUM_BANKS      (1),
        .NUM_WAYS       (`ICACHE_NUM_WAYS),
        .WORD_SIZE      (ICACHE_WORD_SIZE),
        .NUM_REQS       (1),
        .MEM_PORTS      (1),
        .CRSQ_SIZE      (`ICACHE_CRSQ_SIZE),
        .MSHR_SIZE      (`ICACHE_MSHR_SIZE),
        .MRSQ_SIZE      (`ICACHE_MRSQ_SIZE),
        .MREQ_SIZE      (`ICACHE_MREQ_SIZE),
        .TAG_WIDTH      (ICACHE_TAG_WIDTH),
        .WRITE_ENABLE   (0),
        .REPL_POLICY    (`ICACHE_REPL_POLICY),
        .NC_ENABLE      (0),
        .CORE_OUT_BUF   (3),
        .MEM_OUT_BUF    (2)
    ) icache (
    `ifdef PERF_ENABLE
        .cache_perf     (icache_perf),
    `endif
        .clk            (clk),
        .reset          (icache_reset),
        .core_bus_if    (per_core_icache_bus_if),
        .mem_bus_if     (icache_mem_bus_if)
    );

    ///////////////////////////////////////////////////////////////////////////

    VX_mem_bus_if #(
        .DATA_SIZE (DCACHE_WORD_SIZE),
        .TAG_WIDTH (DCACHE_TAG_WIDTH)
    ) per_core_dcache_bus_if[`SOCKET_SIZE * DCACHE_NUM_REQS]();

    VX_mem_bus_if #(
        .DATA_SIZE (DCACHE_LINE_SIZE),
        .TAG_WIDTH (DCACHE_MEM_TAG_WIDTH)
    ) dcache_mem_bus_if[`L1_MEM_PORTS]();

    `RESET_RELAY (dcache_reset, reset);

    VX_cache_cluster #(
        .INSTANCE_ID    (`SFORMATF(("%s-dcache", INSTANCE_ID))),
        .NUM_UNITS      (`NUM_DCACHES),
        .NUM_INPUTS     (`SOCKET_SIZE),
        .TAG_SEL_IDX    (0),
        .CACHE_SIZE     (`DCACHE_SIZE),
        .LINE_SIZE      (DCACHE_LINE_SIZE),
        .NUM_BANKS      (`DCACHE_NUM_BANKS),
        .NUM_WAYS       (`DCACHE_NUM_WAYS),
        .WORD_SIZE      (DCACHE_WORD_SIZE),
        .NUM_REQS       (DCACHE_NUM_REQS),
        .MEM_PORTS      (`L1_MEM_PORTS),
        .CRSQ_SIZE      (`DCACHE_CRSQ_SIZE),
        .MSHR_SIZE      (`DCACHE_MSHR_SIZE),
        .MRSQ_SIZE      (`DCACHE_MRSQ_SIZE),
        .MREQ_SIZE      (`DCACHE_WRITEBACK ? `DCACHE_MSHR_SIZE : `DCACHE_MREQ_SIZE),
        .TAG_WIDTH      (DCACHE_TAG_WIDTH),
        .WRITE_ENABLE   (1),
        .WRITEBACK      (`DCACHE_WRITEBACK),
        .DIRTY_BYTES    (`DCACHE_DIRTYBYTES),
        .REPL_POLICY    (`DCACHE_REPL_POLICY),
        .NC_ENABLE      (1),
        .CORE_OUT_BUF   (3),
        .MEM_OUT_BUF    (2)
    ) dcache (
    `ifdef PERF_ENABLE
        .cache_perf     (dcache_perf),
    `endif
        .clk            (clk),
        .reset          (dcache_reset),
        .core_bus_if    (per_core_dcache_bus_if),
        .mem_bus_if     (dcache_mem_bus_if)
    );

    ///////////////////////////////////////////////////////////////////////////

    for (genvar i = 0; i < `L1_MEM_PORTS; ++i) begin : g_mem_bus_if
        if (i == 0) begin : g_i0
            VX_mem_bus_if #(
                .DATA_SIZE (`L1_LINE_SIZE),
                .TAG_WIDTH (L1_MEM_TAG_WIDTH)
            ) l1_mem_bus_if[2]();

            VX_mem_bus_if #(
                .DATA_SIZE (`L1_LINE_SIZE),
                .TAG_WIDTH (L1_MEM_ARB_TAG_WIDTH)
            ) l1_mem_arb_bus_if[1]();

            `ASSIGN_VX_MEM_BUS_IF_EX (l1_mem_bus_if[0], icache_mem_bus_if[0], L1_MEM_TAG_WIDTH, ICACHE_MEM_TAG_WIDTH, UUID_WIDTH);
            `ASSIGN_VX_MEM_BUS_IF_EX (l1_mem_bus_if[1], dcache_mem_bus_if[0], L1_MEM_TAG_WIDTH, DCACHE_MEM_TAG_WIDTH, UUID_WIDTH);

            VX_mem_arb #(
                .NUM_INPUTS (2),
                .NUM_OUTPUTS(1),
                .DATA_SIZE  (`L1_LINE_SIZE),
                .TAG_WIDTH  (L1_MEM_TAG_WIDTH),
                .TAG_SEL_IDX(0),
                .ARBITER    ("P"), // prioritize the icache
                .REQ_OUT_BUF(3),
                .RSP_OUT_BUF(3)
            ) mem_arb (
                .clk        (clk),
                .reset      (reset),
                .bus_in_if  (l1_mem_bus_if),
                .bus_out_if (l1_mem_arb_bus_if)
            );

            `ASSIGN_VX_MEM_BUS_IF (mem_bus_if[0], l1_mem_arb_bus_if[0]);
        end else begin : g_i
            VX_mem_bus_if #(
                .DATA_SIZE (`L1_LINE_SIZE),
                .TAG_WIDTH (L1_MEM_ARB_TAG_WIDTH)
            ) l1_mem_arb_bus_if();

            `ASSIGN_VX_MEM_BUS_IF_EX (l1_mem_arb_bus_if, dcache_mem_bus_if[i], L1_MEM_ARB_TAG_WIDTH, DCACHE_MEM_TAG_WIDTH, UUID_WIDTH);
            `ASSIGN_VX_MEM_BUS_IF (mem_bus_if[i], l1_mem_arb_bus_if);
        end
    end

    ///////////////////////////////////////////////////////////////////////////

    wire [`SOCKET_SIZE-1:0] per_core_busy;

    // Generate all cores
    for (genvar core_id = 0; core_id < `SOCKET_SIZE; ++core_id) begin : g_cores

        `RESET_RELAY (core_reset, reset);

        VX_dcr_bus_if core_dcr_bus_if();
        `BUFFER_DCR_BUS_IF (core_dcr_bus_if, dcr_bus_if, 1'b1, (`SOCKET_SIZE > 1))

        VX_core #(
            .CORE_ID  ((SOCKET_ID * `SOCKET_SIZE) + core_id),
            .INSTANCE_ID (`SFORMATF(("%s-core%0d", INSTANCE_ID, core_id)))
        ) core (
            `SCOPE_IO_BIND  (scope_core + core_id)

            .clk            (clk),
            .reset          (core_reset),

        `ifdef PERF_ENABLE
            .sysmem_perf    (sysmem_perf_tmp),
        `ifdef VM_ENABLE
            .ptw_latency_in  (ptw_latency_in),
            .pwc_hits_in     (pwc_hits_in),
            .pwc_misses_in   (pwc_misses_in),
            .pwc2_hits_in    (pwc2_hits_in),
            .pwc2_misses_in  (pwc2_misses_in),
        `endif
        `endif

            .dcr_bus_if     (core_dcr_bus_if),

            .dcache_bus_if  (per_core_dcache_bus_if[core_id * DCACHE_NUM_REQS +: DCACHE_NUM_REQS]),

            .icache_bus_if  (per_core_icache_bus_if[core_id]),

        `ifdef GBAR_ENABLE
            .gbar_bus_if    (per_core_gbar_bus_if[core_id]),
        `endif

        `ifdef VM_ENABLE
            .dtlb_ptw_req_valid (per_core_dtlb_ptw_req_valid[core_id]),
            .dtlb_ptw_req_ready (per_core_dtlb_ptw_req_ready[core_id]),
            .dtlb_ptw_req_vaddr (per_core_dtlb_ptw_req_vaddr[core_id]),
            .dtlb_ptw_rsp_valid (per_core_dtlb_ptw_rsp_valid[core_id]),
            .dtlb_ptw_rsp_ready (per_core_dtlb_ptw_rsp_ready[core_id]),
            .dtlb_ptw_rsp_vaddr (per_core_dtlb_ptw_rsp_vaddr[core_id]),
            .dtlb_ptw_rsp_paddr (per_core_dtlb_ptw_rsp_paddr[core_id]),
            .dtlb_ptw_rsp_flags (per_core_dtlb_ptw_rsp_flags[core_id]),
            .itlb_ptw_req_valid (per_core_itlb_ptw_req_valid[core_id]),
            .itlb_ptw_req_ready (per_core_itlb_ptw_req_ready[core_id]),
            .itlb_ptw_req_vaddr (per_core_itlb_ptw_req_vaddr[core_id]),
            .itlb_ptw_rsp_valid (per_core_itlb_ptw_rsp_valid[core_id]),
            .itlb_ptw_rsp_ready (per_core_itlb_ptw_rsp_ready[core_id]),
            .itlb_ptw_rsp_vaddr (per_core_itlb_ptw_rsp_vaddr[core_id]),
            .itlb_ptw_rsp_paddr (per_core_itlb_ptw_rsp_paddr[core_id]),
            .itlb_ptw_rsp_flags (per_core_itlb_ptw_rsp_flags[core_id]),
        `endif

            .busy           (per_core_busy[core_id])
        );
    end

    `BUFFER_EX(busy, (| per_core_busy), 1'b1, 1, (`SOCKET_SIZE > 1));

`ifdef VM_ENABLE

    ///////////////////////////////////////////////////////////////////////////
    // Per-core PTW miss/fill signals — pass-through to device-level shared PTW.
    // Layout: [0..SOCKET_SIZE-1] = dTLB (one per core)
    //         [SOCKET_SIZE..2*SOCKET_SIZE-1] = iTLB (one per core)
    ///////////////////////////////////////////////////////////////////////////

    // One bit/value per core, for dTLB
    wire [`SOCKET_SIZE-1:0]  per_core_dtlb_ptw_req_valid;
    wire [`SOCKET_SIZE-1:0]  per_core_dtlb_ptw_req_ready;
    wire [`XLEN-1:0]         per_core_dtlb_ptw_req_vaddr [`SOCKET_SIZE];
    wire [`SOCKET_SIZE-1:0]  per_core_dtlb_ptw_rsp_valid;
    wire [`SOCKET_SIZE-1:0]  per_core_dtlb_ptw_rsp_ready;
    wire [`XLEN-1:0]         per_core_dtlb_ptw_rsp_vaddr [`SOCKET_SIZE];
    wire [`XLEN-1:0]         per_core_dtlb_ptw_rsp_paddr [`SOCKET_SIZE];
    wire [7:0]               per_core_dtlb_ptw_rsp_flags [`SOCKET_SIZE];

    // One bit/value per core, for iTLB
    wire [`SOCKET_SIZE-1:0]  per_core_itlb_ptw_req_valid;
    wire [`SOCKET_SIZE-1:0]  per_core_itlb_ptw_req_ready;
    wire [`XLEN-1:0]         per_core_itlb_ptw_req_vaddr [`SOCKET_SIZE];
    wire [`SOCKET_SIZE-1:0]  per_core_itlb_ptw_rsp_valid;
    wire [`SOCKET_SIZE-1:0]  per_core_itlb_ptw_rsp_ready;
    wire [`XLEN-1:0]         per_core_itlb_ptw_rsp_vaddr [`SOCKET_SIZE];
    wire [`XLEN-1:0]         per_core_itlb_ptw_rsp_paddr [`SOCKET_SIZE];
    wire [7:0]               per_core_itlb_ptw_rsp_flags [`SOCKET_SIZE];

    // Connect per-core dTLB/iTLB miss/fill wires to socket-level PTW ports.
    // PTW requestor index: dTLB of core i → i, iTLB of core i → SOCKET_SIZE+i
    for (genvar i = 0; i < `SOCKET_SIZE; i++) begin : g_ptw_wire
        // dTLB slot i
        assign ptw_miss_valid[i]                   = per_core_dtlb_ptw_req_valid[i];
        assign per_core_dtlb_ptw_req_ready[i]      = ptw_miss_ready[i];
        assign ptw_miss_vaddr[i]                   = per_core_dtlb_ptw_req_vaddr[i];
        assign per_core_dtlb_ptw_rsp_valid[i]      = ptw_fill_valid[i];
        assign ptw_fill_ready[i]                   = per_core_dtlb_ptw_rsp_ready[i];
        assign per_core_dtlb_ptw_rsp_vaddr[i]      = ptw_fill_vaddr[i];
        assign per_core_dtlb_ptw_rsp_paddr[i]      = ptw_fill_paddr[i];
        assign per_core_dtlb_ptw_rsp_flags[i]      = ptw_fill_flags[i];
        // iTLB slot SOCKET_SIZE+i
        assign ptw_miss_valid[`SOCKET_SIZE + i]               = per_core_itlb_ptw_req_valid[i];
        assign per_core_itlb_ptw_req_ready[i]                 = ptw_miss_ready[`SOCKET_SIZE + i];
        assign ptw_miss_vaddr[`SOCKET_SIZE + i]               = per_core_itlb_ptw_req_vaddr[i];
        assign per_core_itlb_ptw_rsp_valid[i]                 = ptw_fill_valid[`SOCKET_SIZE + i];
        assign ptw_fill_ready[`SOCKET_SIZE + i]               = per_core_itlb_ptw_rsp_ready[i];
        assign per_core_itlb_ptw_rsp_vaddr[i]                 = ptw_fill_vaddr[`SOCKET_SIZE + i];
        assign per_core_itlb_ptw_rsp_paddr[i]                 = ptw_fill_paddr[`SOCKET_SIZE + i];
        assign per_core_itlb_ptw_rsp_flags[i]                 = ptw_fill_flags[`SOCKET_SIZE + i];
    end

`endif // VM_ENABLE

endmodule
