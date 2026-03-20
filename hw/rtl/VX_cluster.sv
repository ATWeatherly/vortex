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

module VX_cluster import VX_gpu_pkg::*; #(
    parameter CLUSTER_ID = 0,
    parameter `STRING INSTANCE_ID = ""
) (
    `SCOPE_IO_DECL

    // Clock
    input  wire                 clk,
    input  wire                 reset,

`ifdef PERF_ENABLE
    input sysmem_perf_t         sysmem_perf,
`endif

    // DCRs
    VX_dcr_bus_if.slave         dcr_bus_if,

    // Memory
    VX_mem_bus_if.master        mem_bus_if [`L2_MEM_PORTS],

    // Status
    output wire                 busy,

`ifdef VM_ENABLE
    // PTW miss requests from all sockets — routed up to device-level shared PTW
    output wire [NUM_SOCKETS*`SOCKET_SIZE*2-1:0] ptw_miss_valid,
    output wire [31:0]                            ptw_miss_vaddr [NUM_SOCKETS*`SOCKET_SIZE*2],
    input  wire [NUM_SOCKETS*`SOCKET_SIZE*2-1:0] ptw_miss_ready,
    // PTW fill responses coming back down from device-level shared PTW
    input  wire [NUM_SOCKETS*`SOCKET_SIZE*2-1:0] ptw_fill_valid,
    output wire [NUM_SOCKETS*`SOCKET_SIZE*2-1:0] ptw_fill_ready,
    input  wire [31:0]                            ptw_fill_vaddr [NUM_SOCKETS*`SOCKET_SIZE*2],
    input  wire [31:0]                            ptw_fill_paddr [NUM_SOCKETS*`SOCKET_SIZE*2],
    input  wire [7:0]                             ptw_fill_flags [NUM_SOCKETS*`SOCKET_SIZE*2],
    // PTW memory port: threaded from socket 0 up to device-level PTW
    VX_mem_bus_if.slave                           ptw_mem_if
`endif

`ifdef PERF_ENABLE
`ifdef VM_ENABLE
    // PTW perf counters from device-level PTW, passed down to all sockets/cores
    , input wire [PERF_CTR_BITS-1:0]  ptw_latency_in
    , input wire [PERF_CTR_BITS-1:0]  pwc_hits_in
    , input wire [PERF_CTR_BITS-1:0]  pwc_misses_in
`endif
`endif
);

`ifdef SCOPE
    localparam scope_socket = 0;
    `SCOPE_IO_SWITCH (NUM_SOCKETS);
`endif

`ifdef PERF_ENABLE
    cache_perf_t l2_perf;
    sysmem_perf_t sysmem_perf_tmp;
    always @(*) begin
        sysmem_perf_tmp = sysmem_perf;
        sysmem_perf_tmp.l2cache = l2_perf;
    end
`endif

`ifdef GBAR_ENABLE

    VX_gbar_bus_if per_socket_gbar_bus_if[NUM_SOCKETS]();
    VX_gbar_bus_if gbar_bus_if();

    VX_gbar_arb #(
        .NUM_REQS (NUM_SOCKETS),
        .OUT_BUF  ((NUM_SOCKETS > 2) ? 1 : 0) // bgar_unit has no backpressure
    ) gbar_arb (
        .clk        (clk),
        .reset      (reset),
        .bus_in_if  (per_socket_gbar_bus_if),
        .bus_out_if (gbar_bus_if)
    );

    VX_gbar_unit #(
        .INSTANCE_ID (`SFORMATF(("gbar%0d", CLUSTER_ID)))
    ) gbar_unit (
        .clk         (clk),
        .reset       (reset),
        .gbar_bus_if (gbar_bus_if)
    );

`endif

    VX_mem_bus_if #(
        .DATA_SIZE (`L1_LINE_SIZE),
        .TAG_WIDTH (L1_MEM_ARB_TAG_WIDTH)
    ) per_socket_mem_bus_if[NUM_SOCKETS * `L1_MEM_PORTS]();

    `RESET_RELAY (l2_reset, reset);

    VX_cache_wrap #(
        .INSTANCE_ID    (`SFORMATF(("%s-l2cache", INSTANCE_ID))),
        .CACHE_SIZE     (`L2_CACHE_SIZE),
        .LINE_SIZE      (`L2_LINE_SIZE),
        .NUM_BANKS      (`L2_NUM_BANKS),
        .NUM_WAYS       (`L2_NUM_WAYS),
        .WORD_SIZE      (L2_WORD_SIZE),
        .NUM_REQS       (L2_NUM_REQS),
        .MEM_PORTS      (`L2_MEM_PORTS),
        .CRSQ_SIZE      (`L2_CRSQ_SIZE),
        .MSHR_SIZE      (`L2_MSHR_SIZE),
        .MRSQ_SIZE      (`L2_MRSQ_SIZE),
        .MREQ_SIZE      (`L2_WRITEBACK ? `L2_MSHR_SIZE : `L2_MREQ_SIZE),
        .TAG_WIDTH      (L2_TAG_WIDTH),
        .WRITE_ENABLE   (1),
        .WRITEBACK      (`L2_WRITEBACK),
        .DIRTY_BYTES    (`L2_DIRTYBYTES),
        .REPL_POLICY    (`L2_REPL_POLICY),
        .CORE_OUT_BUF   (3),
        .MEM_OUT_BUF    (3),
        .NC_ENABLE      (1),
        .PASSTHRU       (!`L2_ENABLED)
    ) l2cache (
        .clk            (clk),
        .reset          (l2_reset),
    `ifdef PERF_ENABLE
        .cache_perf     (l2_perf),
    `endif
        .core_bus_if    (per_socket_mem_bus_if),
        .mem_bus_if     (mem_bus_if)
    );

    ///////////////////////////////////////////////////////////////////////////

    wire [NUM_SOCKETS-1:0] per_socket_busy;

`ifdef VM_ENABLE
    localparam SOCKET_PTW_REQS = `SOCKET_SIZE * 2;

    wire [SOCKET_PTW_REQS-1:0]  per_socket_ptw_miss_valid [NUM_SOCKETS];
    wire [SOCKET_PTW_REQS-1:0]  per_socket_ptw_miss_ready [NUM_SOCKETS];
    wire [31:0]                      per_socket_ptw_miss_vaddr [NUM_SOCKETS][SOCKET_PTW_REQS];
    wire [SOCKET_PTW_REQS-1:0]  per_socket_ptw_fill_valid [NUM_SOCKETS];
    wire [SOCKET_PTW_REQS-1:0]  per_socket_ptw_fill_ready [NUM_SOCKETS];
    wire [31:0]                      per_socket_ptw_fill_vaddr [NUM_SOCKETS][SOCKET_PTW_REQS];
    wire [31:0]                      per_socket_ptw_fill_paddr [NUM_SOCKETS][SOCKET_PTW_REQS];
    wire [7:0]                       per_socket_ptw_fill_flags [NUM_SOCKETS][SOCKET_PTW_REQS];

    // PTW memory interface per socket:
    //   socket 0 is connected to the cluster ptw_mem_if pass-through port.
    //   sockets 1..N-1 are tied off (they never drive PTW mem traffic).
    localparam PTW_MEM_TAG_WIDTH_CL = `MAX(DCACHE_TAG_WIDTH_BASE + DCACHE_TLB_SOURCE_BITS, `CLOG2(`PTW_SIZE));

    VX_mem_bus_if #(
        .DATA_SIZE (DCACHE_WORD_SIZE),
        .TAG_WIDTH (PTW_MEM_TAG_WIDTH_CL)
    ) per_socket_ptw_mem_if[NUM_SOCKETS]();

    `ASSIGN_VX_MEM_BUS_IF (per_socket_ptw_mem_if[0], ptw_mem_if);

    for (genvar s = 1; s < NUM_SOCKETS; s++) begin : g_ptw_mem_tie
        assign per_socket_ptw_mem_if[s].req_valid = 1'b0;
        assign per_socket_ptw_mem_if[s].req_data  = '0;
        assign per_socket_ptw_mem_if[s].rsp_ready = 1'b1;
    end
`endif // VM_ENABLE

    // Generate all sockets
    for (genvar socket_id = 0; socket_id < NUM_SOCKETS; ++socket_id) begin : g_sockets

        `RESET_RELAY (socket_reset, reset);

        VX_dcr_bus_if socket_dcr_bus_if();
        wire is_base_dcr_addr = (dcr_bus_if.write_addr >= `VX_DCR_BASE_STATE_BEGIN && dcr_bus_if.write_addr < `VX_DCR_BASE_STATE_END);
        `BUFFER_DCR_BUS_IF (socket_dcr_bus_if, dcr_bus_if, is_base_dcr_addr, (NUM_SOCKETS > 1))

        VX_socket #(
            .SOCKET_ID ((CLUSTER_ID * NUM_SOCKETS) + socket_id),
            .INSTANCE_ID (`SFORMATF(("%s-socket%0d", INSTANCE_ID, socket_id)))
        ) socket (
            `SCOPE_IO_BIND  (scope_socket+socket_id)

            .clk            (clk),
            .reset          (socket_reset),

        `ifdef PERF_ENABLE
            .sysmem_perf    (sysmem_perf_tmp),
        `endif

            .dcr_bus_if     (socket_dcr_bus_if),

            .mem_bus_if     (per_socket_mem_bus_if[socket_id * `L1_MEM_PORTS +: `L1_MEM_PORTS]),

        `ifdef GBAR_ENABLE
            .gbar_bus_if    (per_socket_gbar_bus_if[socket_id]),
        `endif

        `ifdef VM_ENABLE
            .ptw_miss_valid (per_socket_ptw_miss_valid[socket_id]),
            .ptw_miss_vaddr (per_socket_ptw_miss_vaddr[socket_id]),
            .ptw_miss_ready (per_socket_ptw_miss_ready[socket_id]),
            .ptw_fill_valid (per_socket_ptw_fill_valid[socket_id]),
            .ptw_fill_ready (per_socket_ptw_fill_ready[socket_id]),
            .ptw_fill_vaddr (per_socket_ptw_fill_vaddr[socket_id]),
            .ptw_fill_paddr (per_socket_ptw_fill_paddr[socket_id]),
            .ptw_fill_flags (per_socket_ptw_fill_flags[socket_id]),
            .ptw_mem_if     (per_socket_ptw_mem_if[socket_id]),
        `endif

        `ifdef PERF_ENABLE
        `ifdef VM_ENABLE
            .ptw_latency_in (ptw_latency_in),
            .pwc_hits_in    (pwc_hits_in),
            .pwc_misses_in  (pwc_misses_in),
        `endif
        `endif

            .busy           (per_socket_busy[socket_id])
        );
    end

    `BUFFER_EX(busy, (| per_socket_busy), 1'b1, 1, (NUM_SOCKETS > 1));

`ifdef VM_ENABLE
    // Flatten per-socket miss/fill into cluster-level port arrays.
    // Layout: socket s occupies indices [s*SOCKET_SIZE*2 .. (s+1)*SOCKET_SIZE*2-1]
    for (genvar s = 0; s < NUM_SOCKETS; s++) begin : g_cluster_ptw_wire
        for (genvar r = 0; r < SOCKET_PTW_REQS; r++) begin : g_req
            assign ptw_miss_valid[s * SOCKET_PTW_REQS + r]                          = per_socket_ptw_miss_valid[s][r];
            assign per_socket_ptw_miss_ready[s][r]                                   = ptw_miss_ready[s * SOCKET_PTW_REQS + r];
            assign ptw_miss_vaddr[s * SOCKET_PTW_REQS + r]                           = per_socket_ptw_miss_vaddr[s][r];
            assign per_socket_ptw_fill_valid[s][r]                                   = ptw_fill_valid[s * SOCKET_PTW_REQS + r];
            assign ptw_fill_ready[s * SOCKET_PTW_REQS + r]                           = per_socket_ptw_fill_ready[s][r];
            assign per_socket_ptw_fill_vaddr[s][r]                                   = ptw_fill_vaddr[s * SOCKET_PTW_REQS + r];
            assign per_socket_ptw_fill_paddr[s][r]                                   = ptw_fill_paddr[s * SOCKET_PTW_REQS + r];
            assign per_socket_ptw_fill_flags[s][r]                                   = ptw_fill_flags[s * SOCKET_PTW_REQS + r];
        end
    end
`endif // VM_ENABLE

endmodule
