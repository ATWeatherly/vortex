// Copyright 2024
// PTW: SV32 multi-threaded shared page table walker (HPCA14 Design 3)
// Supports PTW_SIZE concurrent walks from NUM_REQUESTORS TLBs.
// Each walk slot tracks its own state independently; slots share one memory port.

`include "VX_define.vh"
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */

module VX_mmu_ptw import VX_gpu_pkg::*; #(
    parameter DATA_SIZE      = DCACHE_WORD_SIZE,
    parameter TAG_WIDTH      = 8,
    parameter ADDR_WIDTH     = DCACHE_ADDR_WIDTH,
    parameter FLAGS_WIDTH    = MEM_FLAGS_WIDTH,
    parameter PTW_SIZE       = 8,   // concurrent walk slots
    parameter NUM_REQUESTORS = 2    // number of TLBs feeding this PTW
) (
    input wire clk,
    input wire reset,

    input wire [31:0] satp,

    // Miss request inputs (one per requestor TLB)
    input  wire [NUM_REQUESTORS-1:0]        miss_valid,
    output wire [NUM_REQUESTORS-1:0]        miss_ready,
    input  wire [31:0]                      miss_vaddr [NUM_REQUESTORS],

    // Fill response outputs (one per requestor TLB)
    output wire [NUM_REQUESTORS-1:0]        fill_valid,
    input  wire [NUM_REQUESTORS-1:0]        fill_ready,
    output wire [31:0]                      fill_vaddr [NUM_REQUESTORS],
    output wire [31:0]                      fill_paddr [NUM_REQUESTORS],
    output wire [7:0]                       fill_flags [NUM_REQUESTORS],

    // Shared memory port for page table reads
    VX_mem_bus_if.master ptw_mem_if,

`ifdef PERF_ENABLE
    output wire [PERF_CTR_BITS-1:0] perf_ptw_latency
`else
    output wire perf_ptw_latency_placeholder
`endif
);

    localparam DATA_WIDTH       = DATA_SIZE * 8;
    localparam SLOT_BITS        = `CLOG2(PTW_SIZE);
    localparam REQ_BITS         = `UP(`CLOG2(NUM_REQUESTORS));

    // SV32 constants
    localparam VPN_WIDTH        = 20;
    localparam PPN_WIDTH        = VPN_WIDTH;
    localparam PAGE_OFFSET_BITS = 12;
    localparam VPN_LEVEL_BITS   = 10;
    localparam PTE_SHIFT        = 2;  // log2(4 bytes per PTE)
    localparam ADDR_SHIFT       = `CLOG2(DATA_SIZE);
    localparam NUM_WORDS        = DATA_SIZE / 4;
    localparam SEL_BITS         = `CLOG2(NUM_WORDS);

    // =========================================================================
    // Per-slot state
    // =========================================================================

    typedef enum logic [2:0] {
        PTW_IDLE    = 3'd0,
        PTW_L1_REQ  = 3'd1,
        PTW_L1_RESP = 3'd2,
        PTW_L0_REQ  = 3'd3,
        PTW_L0_RESP = 3'd4,
        PTW_FILL    = 3'd5
    } ptw_state_t;

    ptw_state_t         slot_state      [PTW_SIZE];
    reg [31:0]          slot_vaddr      [PTW_SIZE];
    reg [PPN_WIDTH-1:0] slot_l1_ppn     [PTW_SIZE];
    reg [PPN_WIDTH-1:0] slot_ppn        [PTW_SIZE];
    reg [7:0]           slot_flags      [PTW_SIZE];
    reg [31:0]          slot_pte_addr_r [PTW_SIZE];
    reg [REQ_BITS-1:0]  slot_src        [PTW_SIZE];  // which requestor owns this slot

    // =========================================================================
    // Miss arbitration: rotating priority among requestors, assign to free slot
    // =========================================================================

    wire [PTW_SIZE-1:0] slot_idle;
    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_idle
        assign slot_idle[s] = (slot_state[s] == PTW_IDLE);
    end
    wire any_slot_free = |slot_idle;

    // Priority-encode lowest free slot (last writer wins = highest index wins)
    reg [SLOT_BITS-1:0] free_slot;
    always_comb begin
        free_slot = '0;
        for (int s = PTW_SIZE-1; s >= 0; s--) begin
            if (slot_idle[s]) free_slot = SLOT_BITS'(s);
        end
    end

    // Rotating priority miss selection
    reg [REQ_BITS-1:0] miss_rr_base;
    reg [REQ_BITS-1:0] selected_req;
    reg                miss_grant;

    always_comb begin
        selected_req = '0;
        miss_grant   = 1'b0;
        for (int i = 0; i < NUM_REQUESTORS; i++) begin
            int idx;
            idx = (int'(miss_rr_base) + i) % NUM_REQUESTORS;
            if (miss_valid[idx] && any_slot_free && !miss_grant) begin
                selected_req = REQ_BITS'(idx);
                miss_grant   = 1'b1;
            end
        end
    end

    for (genvar r = 0; r < NUM_REQUESTORS; r++) begin : g_miss_ready
        assign miss_ready[r] = miss_grant && (selected_req == REQ_BITS'(r));
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            miss_rr_base <= '0;
        end else if (miss_grant) begin
            miss_rr_base <= REQ_BITS'((int'(selected_req) + 1) % NUM_REQUESTORS);
        end
    end

    // =========================================================================
    // Memory request arbitration: rotating priority among slots wanting mem
    // =========================================================================

    wire [PTW_SIZE-1:0] slot_wants_mem;
    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_wants_mem
        assign slot_wants_mem[s] = (slot_state[s] == PTW_L1_REQ) ||
                                   (slot_state[s] == PTW_L0_REQ);
    end

    reg [SLOT_BITS-1:0] mem_rr_base;
    reg [SLOT_BITS-1:0] mem_req_slot;
    reg                 mem_req_grant;

    always_comb begin
        mem_req_slot  = '0;
        mem_req_grant = 1'b0;
        for (int i = 0; i < PTW_SIZE; i++) begin
            int idx;
            idx = (int'(mem_rr_base) + i) % PTW_SIZE;
            if (slot_wants_mem[idx] && !mem_req_grant) begin
                mem_req_slot  = SLOT_BITS'(idx);
                mem_req_grant = 1'b1;
            end
        end
    end

    wire mem_req_fire = ptw_mem_if.req_valid && ptw_mem_if.req_ready;
    wire mem_rsp_fire = ptw_mem_if.rsp_valid && ptw_mem_if.rsp_ready;
    wire [SLOT_BITS-1:0] rsp_slot = ptw_mem_if.rsp_data.tag[SLOT_BITS-1:0];

    always_ff @(posedge clk) begin
        if (reset) begin
            mem_rr_base <= '0;
        end else if (mem_req_fire) begin
            mem_rr_base <= SLOT_BITS'((int'(mem_req_slot) + 1) % PTW_SIZE);
        end
    end

    // =========================================================================
    // Per-slot state machines
    // =========================================================================

    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_slots

        wire [VPN_LEVEL_BITS-1:0] vpn1 = slot_vaddr[s][31:22];
        wire [VPN_LEVEL_BITS-1:0] vpn0 = slot_vaddr[s][21:12];

        wire [31:0] l1_pte_addr = {satp[PPN_WIDTH-1:0], {PAGE_OFFSET_BITS{1'b0}}} +
                                  {{(32-VPN_LEVEL_BITS-PTE_SHIFT){1'b0}}, vpn1, {PTE_SHIFT{1'b0}}};
        wire [31:0] l0_pte_addr = {slot_l1_ppn[s], {PAGE_OFFSET_BITS{1'b0}}} +
                                  {{(32-VPN_LEVEL_BITS-PTE_SHIFT){1'b0}}, vpn0, {PTE_SHIFT{1'b0}}};

        // PTE extraction from response data (shared bus; only used when this_rsp)
        wire [31:0] pte_data;
        if (NUM_WORDS > 1) begin : g_pte_sel
            wire [SEL_BITS-1:0] word_sel = slot_pte_addr_r[s][SEL_BITS+1:2];
            assign pte_data = ptw_mem_if.rsp_data.data[word_sel * 32 +: 32];
        end else begin : g_pte_direct
            assign pte_data = ptw_mem_if.rsp_data.data[31:0];
        end

        wire [PPN_WIDTH-1:0] pte_ppn   = pte_data[29:10];
        wire [7:0]           pte_flags = pte_data[7:0];

        wire slot_alloc        = miss_grant && (free_slot == SLOT_BITS'(s));
        wire this_mem_req_fire = mem_req_fire && (mem_req_slot == SLOT_BITS'(s));
        wire this_rsp          = mem_rsp_fire && (rsp_slot == SLOT_BITS'(s));
        wire this_fill_ack     = fill_valid[slot_src[s]] && fill_ready[slot_src[s]]
                                 && (slot_state[s] == PTW_FILL);

        always_ff @(posedge clk) begin
            if (reset) begin
                slot_state[s]      <= PTW_IDLE;
                slot_vaddr[s]      <= '0;
                slot_l1_ppn[s]     <= '0;
                slot_ppn[s]        <= '0;
                slot_flags[s]      <= '0;
                slot_pte_addr_r[s] <= '0;
                slot_src[s]        <= '0;
            end else begin
                case (slot_state[s])
                    PTW_IDLE: begin
                        if (slot_alloc) begin
                            slot_state[s] <= PTW_L1_REQ;
                            slot_vaddr[s] <= miss_vaddr[selected_req];
                            slot_src[s]   <= selected_req;
                        end
                    end
                    PTW_L1_REQ: begin
                        if (this_mem_req_fire) begin
                            slot_state[s]      <= PTW_L1_RESP;
                            slot_pte_addr_r[s] <= l1_pte_addr;
                        end
                    end
                    PTW_L1_RESP: begin
                        if (this_rsp) begin
                            slot_state[s]  <= PTW_L0_REQ;
                            slot_l1_ppn[s] <= pte_ppn;
                        end
                    end
                    PTW_L0_REQ: begin
                        if (this_mem_req_fire) begin
                            slot_state[s]      <= PTW_L0_RESP;
                            slot_pte_addr_r[s] <= l0_pte_addr;
                        end
                    end
                    PTW_L0_RESP: begin
                        if (this_rsp) begin
                            slot_state[s] <= PTW_FILL;
                            slot_ppn[s]   <= pte_ppn;
                            slot_flags[s] <= pte_flags;
                        end
                    end
                    PTW_FILL: begin
                        if (this_fill_ack) begin
                            slot_state[s] <= PTW_IDLE;
                        end
                    end
                    default: slot_state[s] <= PTW_IDLE;
                endcase
            end
        end
    end

    // =========================================================================
    // Fill output: route completed slots back to their requestor
    // =========================================================================

    for (genvar r = 0; r < NUM_REQUESTORS; r++) begin : g_fill

        wire [PTW_SIZE-1:0] slot_fill_for_r;
        for (genvar s = 0; s < PTW_SIZE; s++) begin : g_fill_match
            assign slot_fill_for_r[s] = (slot_state[s] == PTW_FILL) &&
                                        (slot_src[s] == REQ_BITS'(r));
        end

        reg [31:0] fill_vaddr_w, fill_paddr_w;
        reg [7:0]  fill_flags_w;

        always_comb begin
            fill_vaddr_w = '0;
            fill_paddr_w = '0;
            fill_flags_w = '0;
            for (int s = 0; s < PTW_SIZE; s++) begin
                if (slot_fill_for_r[s]) begin
                    fill_vaddr_w = slot_vaddr[s];
                    fill_paddr_w = {slot_ppn[s], slot_vaddr[s][PAGE_OFFSET_BITS-1:0]};
                    fill_flags_w = slot_flags[s];
                end
            end
        end

        assign fill_valid[r] = |slot_fill_for_r;
        assign fill_vaddr[r] = fill_vaddr_w;
        assign fill_paddr[r] = fill_paddr_w;
        assign fill_flags[r] = fill_flags_w;
    end

    // =========================================================================
    // Memory interface
    // =========================================================================

    wire [31:0] req_pte_addr =
        (slot_state[mem_req_slot] == PTW_L1_REQ) ?
            ({satp[PPN_WIDTH-1:0], {PAGE_OFFSET_BITS{1'b0}}} +
             {{(32-VPN_LEVEL_BITS-PTE_SHIFT){1'b0}},
              slot_vaddr[mem_req_slot][31:22], {PTE_SHIFT{1'b0}}})
        :
            ({slot_l1_ppn[mem_req_slot], {PAGE_OFFSET_BITS{1'b0}}} +
             {{(32-VPN_LEVEL_BITS-PTE_SHIFT){1'b0}},
              slot_vaddr[mem_req_slot][21:12], {PTE_SHIFT{1'b0}}});

    wire [PTW_SIZE-1:0] slot_awaiting_rsp;
    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_rsp_ready
        assign slot_awaiting_rsp[s] = (slot_state[s] == PTW_L1_RESP) ||
                                      (slot_state[s] == PTW_L0_RESP);
    end

    assign ptw_mem_if.req_valid       = mem_req_grant;
    assign ptw_mem_if.req_data.rw     = 1'b0;
    assign ptw_mem_if.req_data.addr   = req_pte_addr[31:ADDR_SHIFT];
    assign ptw_mem_if.req_data.data   = '0;
    assign ptw_mem_if.req_data.byteen = {DATA_SIZE{1'b1}};
    assign ptw_mem_if.req_data.flags  = '0;
    assign ptw_mem_if.req_data.tag    = TAG_WIDTH'(mem_req_slot);  // slot id for response routing
    assign ptw_mem_if.rsp_ready       = |slot_awaiting_rsp;

    // =========================================================================
    // Performance counters
    // =========================================================================

`ifdef PERF_ENABLE
    wire [PTW_SIZE-1:0] slot_active;
    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_active
        assign slot_active[s] = (slot_state[s] != PTW_IDLE);
    end

    reg [PERF_CTR_BITS-1:0] perf_ptw_latency_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            perf_ptw_latency_r <= '0;
        end else if (|slot_active) begin
            perf_ptw_latency_r <= perf_ptw_latency_r + PERF_CTR_BITS'(1);
        end
    end
    assign perf_ptw_latency = perf_ptw_latency_r;
`else
    assign perf_ptw_latency_placeholder = 1'b0;
`endif

endmodule
