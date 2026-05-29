// Copyright 2024
// PTW: Generic N-level shared page table walker (HPCA14 Design 3)
// Supports PTW_SIZE concurrent walks. Handles 2-level (SV32) and 3-level (SV39).
// Single central FSM; slot level counter drives generic walk without mode-specific states.

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

    input wire [`XLEN-1:0] satp,

    // Miss request inputs (one per requestor TLB)
    input  wire [NUM_REQUESTORS-1:0]  miss_valid,
    output wire [NUM_REQUESTORS-1:0]  miss_ready,
    input  wire [`XLEN-1:0]           miss_vaddr [NUM_REQUESTORS],

    // Fill response outputs (one per requestor TLB)
    output wire [NUM_REQUESTORS-1:0]  fill_valid,
    input  wire [NUM_REQUESTORS-1:0]  fill_ready,
    output wire [`XLEN-1:0]           fill_vaddr [NUM_REQUESTORS],
    output wire [`XLEN-1:0]           fill_paddr [NUM_REQUESTORS],
    output wire [7:0]                 fill_flags [NUM_REQUESTORS],

    // Shared memory port for page table reads
    VX_mem_bus_if.master ptw_mem_if,

`ifdef PERF_ENABLE
    output wire [PERF_CTR_BITS-1:0] perf_ptw_latency,
    output wire [PERF_CTR_BITS-1:0] perf_pwc_hits,
    output wire [PERF_CTR_BITS-1:0] perf_pwc_misses,
    output wire [PERF_CTR_BITS-1:0] perf_pwc2_hits,
    output wire [PERF_CTR_BITS-1:0] perf_pwc2_misses
`else
    output wire perf_ptw_latency_placeholder
`endif
);

    localparam DATA_WIDTH     = DATA_SIZE * 8;
    localparam SLOT_BITS      = `CLOG2(PTW_SIZE);
    localparam REQ_BITS       = `UP(`CLOG2(NUM_REQUESTORS));
    localparam ADDR_SHIFT     = `CLOG2(DATA_SIZE);

    // Address-mode constants derived from VX_config.vh defines
    localparam PPN_WIDTH      = `MEM_ADDR_WIDTH - `MEM_PAGE_LOG2_SIZE; // 20 (SV32) or 36 (SV39)
    localparam PT_LEVELS      = `PT_LEVEL;                              // 2 (SV32) or 3 (SV39)
    localparam LEVEL_BITS     = `CLOG2(PT_LEVELS + 1);
    localparam VPN_LEVEL_BITS = (PT_LEVELS == 3) ? 9 : 10;             // 9 (SV39) or 10 (SV32)
    localparam PTE_SHIFT      = `CLOG2(`PTE_SIZE);                     // 2 (SV32) or 3 (SV39)
    localparam PTE_DATA_BITS  = `PTE_SIZE * 8;                         // 32 (SV32) or 64 (SV39)
    localparam PAGE_OFFSET    = `MEM_PAGE_LOG2_SIZE;                   // 12
    localparam NUM_PTES       = DATA_SIZE / `PTE_SIZE;                 // PTEs per cache line
    localparam SEL_BITS       = `CLOG2(NUM_PTES);
    localparam PTE_ADDR_PAD   = `MEM_ADDR_WIDTH - VPN_LEVEL_BITS - PTE_SHIFT; // zero-pad in PTE addr

    // PWC caches the top-level (level PT_LEVELS-1) PTE, keyed by vpn[PT_LEVELS-1]
    localparam PWC_VPN_BITS   = VPN_LEVEL_BITS;
    localparam PWC_ENTRIES    = 1 << PWC_VPN_BITS;

    // =========================================================================
    // Per-slot state
    // =========================================================================

    typedef enum logic [1:0] {
        PTW_IDLE     = 2'd0,
        PTW_MEM_REQ  = 2'd1,
        PTW_MEM_RESP = 2'd2,
        PTW_FILL     = 2'd3
    } ptw_state_t;

    ptw_state_t               slot_state      [PTW_SIZE];
    reg [`XLEN-1:0]           slot_vaddr      [PTW_SIZE];
    reg [PPN_WIDTH-1:0]       slot_cur_ppn    [PTW_SIZE]; // base PPN for current level
    reg [LEVEL_BITS-1:0]      slot_level      [PTW_SIZE]; // current level (PT_LEVELS-1 → 0)
    reg [PPN_WIDTH-1:0]       slot_ppn        [PTW_SIZE]; // leaf PPN captured at level 0
    reg [7:0]                 slot_flags      [PTW_SIZE];
    reg [`MEM_ADDR_WIDTH-1:0] slot_pte_addr_r [PTW_SIZE]; // byte addr of pending PTE fetch
    reg [REQ_BITS-1:0]        slot_src        [PTW_SIZE]; // which requestor owns this slot

    // =========================================================================
    // Miss arbitration: rotating priority, assign to lowest free slot
    // =========================================================================

    wire [PTW_SIZE-1:0] slot_idle;
    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_idle
        assign slot_idle[s] = (slot_state[s] == PTW_IDLE);
    end
    wire any_slot_free = |slot_idle;

    reg [SLOT_BITS-1:0] free_slot;
    always_comb begin
        free_slot = '0;
        for (int s = PTW_SIZE-1; s >= 0; s--) begin
            if (slot_idle[s]) free_slot = SLOT_BITS'(s);
        end
    end

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

    // =========================================================================
    // Memory request arbitration: rotating priority among slots in MEM_REQ
    // =========================================================================

    wire [PTW_SIZE-1:0] slot_wants_mem;
    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_wants_mem
        assign slot_wants_mem[s] = (slot_state[s] == PTW_MEM_REQ);
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

    // =========================================================================
    // PTE address: {cur_ppn, 12'b0} + {vpn[level], PTE_SHIFT zeros}
    // vpn[l] = vaddr[l*VPN_LEVEL_BITS + PAGE_OFFSET +: VPN_LEVEL_BITS]
    // =========================================================================

    wire [VPN_LEVEL_BITS-1:0] req_vpn =
        slot_vaddr[mem_req_slot][slot_level[mem_req_slot] * VPN_LEVEL_BITS + PAGE_OFFSET +: VPN_LEVEL_BITS];

    wire [`MEM_ADDR_WIDTH-1:0] req_pte_addr =
        {slot_cur_ppn[mem_req_slot], {PAGE_OFFSET{1'b0}}} +
        {{PTE_ADDR_PAD{1'b0}}, req_vpn, {PTE_SHIFT{1'b0}}};

    // =========================================================================
    // PTE decode: extract PTE_DATA_BITS word from the cache-line response
    // word_sel picks which PTE within the line using byte-address bits
    // =========================================================================

    wire [PTE_DATA_BITS-1:0] rsp_pte_data;
    if (NUM_PTES > 1) begin : g_pte_sel
        // bits [PTE_SHIFT+SEL_BITS-1 : PTE_SHIFT] = PTE index within cache line
        wire [SEL_BITS-1:0] word_sel =
            slot_pte_addr_r[rsp_slot][PTE_SHIFT + SEL_BITS - 1 : PTE_SHIFT];
        assign rsp_pte_data = ptw_mem_if.rsp_data.data[word_sel * PTE_DATA_BITS +: PTE_DATA_BITS];
    end else begin : g_pte_direct
        assign rsp_pte_data = ptw_mem_if.rsp_data.data[PTE_DATA_BITS-1:0];
    end

    // PPN lives at bits [10 +: PPN_WIDTH] of the PTE (RISC-V PTE layout)
    wire [PPN_WIDTH-1:0] rsp_pte_ppn   = rsp_pte_data[10 +: PPN_WIDTH];
    wire [7:0]           rsp_pte_flags = rsp_pte_data[7:0];

    // =========================================================================
    // Page Walk Cache (HPCA14 Design 3)
    // Caches the PPN returned by the top-level (level = PT_LEVELS-1) fetch.
    // Key = (satp_ppn, vpn[PT_LEVELS-1]); Value = next-level base PPN.
    // PWC hit -> slot starts at level PT_LEVELS-2, skipping one memory fetch.
    // =========================================================================

    wire                   pwc_hit;
    wire [PPN_WIDTH-1:0]   pwc_cached_ppn;

    // top-level VPN: vpn[PT_LEVELS-1]
    localparam TOP_VPN_LSB = (PT_LEVELS - 1) * VPN_LEVEL_BITS + PAGE_OFFSET;
    // middle-level VPN: vpn[PT_LEVELS-2] (meaningful only when PT_LEVELS==3, i.e. SV39 vpn[1])
    localparam MID_VPN_LSB = (PT_LEVELS - 2) * VPN_LEVEL_BITS + PAGE_OFFSET;
    wire [PWC_VPN_BITS-1:0] pwc_lookup_vpn =
        miss_vaddr[selected_req][TOP_VPN_LSB +: VPN_LEVEL_BITS];
    wire [PPN_WIDTH-1:0]    pwc_lookup_satp_ppn = satp[PPN_WIDTH-1:0];

    wire pwc_fill_valid =
        mem_rsp_fire &&
        (slot_state[rsp_slot] == PTW_MEM_RESP) &&
        (slot_level[rsp_slot] == LEVEL_BITS'(PT_LEVELS - 1));
    wire [PWC_VPN_BITS-1:0] pwc_fill_vpn =
        slot_vaddr[rsp_slot][TOP_VPN_LSB +: VPN_LEVEL_BITS];
    wire [PPN_WIDTH-1:0]    pwc_fill_satp_ppn = satp[PPN_WIDTH-1:0];

    VX_mmu_pwc #(
        .PPN_WIDTH   (PPN_WIDTH),
        .VPN_BITS    (PWC_VPN_BITS),
        .NUM_ENTRIES (PWC_ENTRIES)
    ) pwc (
        .clk             (clk),
        .reset           (reset),
        .lookup_satp_ppn (pwc_lookup_satp_ppn),
        .lookup_vpn1     (pwc_lookup_vpn),
        .hit             (pwc_hit),
        .l1_ppn          (pwc_cached_ppn),
        .fill_valid      (pwc_fill_valid),
        .fill_satp_ppn   (pwc_fill_satp_ppn),
        .fill_vpn1       (pwc_fill_vpn),
        .fill_l1_ppn     (rsp_pte_ppn)
    );

    // =========================================================================
    // Page Walk Cache 2 (SV39 only)
    // Caches the PPN from the level-1 fetch: key = (l2_ppn, vpn[1]).
    // Only consulted when PWC1 also hits so l2_ppn is available combinatorially.
    // A double hit reduces a 3-fetch SV39 walk to a single fetch.
    // =========================================================================

    wire               pwc2_hit;
    wire [PPN_WIDTH-1:0] pwc2_cached_ppn;

    if (PT_LEVELS == 3) begin : g_pwc2
        wire [VPN_LEVEL_BITS-1:0] pwc2_lookup_vpn =
            miss_vaddr[selected_req][MID_VPN_LSB +: VPN_LEVEL_BITS];
        wire [PPN_WIDTH-1:0]      pwc2_lookup_tag = pwc_cached_ppn; // l2_ppn from PWC1

        wire pwc2_fill_valid =
            mem_rsp_fire &&
            (slot_state[rsp_slot] == PTW_MEM_RESP) &&
            (slot_level[rsp_slot] == LEVEL_BITS'(1));
        wire [VPN_LEVEL_BITS-1:0] pwc2_fill_vpn =
            slot_vaddr[rsp_slot][MID_VPN_LSB +: VPN_LEVEL_BITS];
        wire [PPN_WIDTH-1:0]      pwc2_fill_tag = slot_cur_ppn[rsp_slot]; // l2_ppn of this slot

        VX_mmu_pwc #(
            .PPN_WIDTH   (PPN_WIDTH),
            .VPN_BITS    (VPN_LEVEL_BITS),
            .NUM_ENTRIES (1 << VPN_LEVEL_BITS)
        ) pwc2_inst (
            .clk             (clk),
            .reset           (reset),
            .lookup_satp_ppn (pwc2_lookup_tag),
            .lookup_vpn1     (pwc2_lookup_vpn),
            .hit             (pwc2_hit),
            .l1_ppn          (pwc2_cached_ppn),
            .fill_valid      (pwc2_fill_valid),
            .fill_satp_ppn   (pwc2_fill_tag),
            .fill_vpn1       (pwc2_fill_vpn),
            .fill_l1_ppn     (rsp_pte_ppn)
        );
    end else begin : g_pwc2_disabled
        assign pwc2_hit        = 1'b0;
        assign pwc2_cached_ppn = '0;
    end

    // =========================================================================
    // Central state machine
    // =========================================================================

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int s = 0; s < PTW_SIZE; s++) begin
                slot_state[s]      <= PTW_IDLE;
                slot_vaddr[s]      <= '0;
                slot_cur_ppn[s]    <= '0;
                slot_level[s]      <= '0;
                slot_ppn[s]        <= '0;
                slot_flags[s]      <= '0;
                slot_pte_addr_r[s] <= '0;
                slot_src[s]        <= '0;
            end
            miss_rr_base <= '0;
            mem_rr_base  <= '0;
        end else begin

            // IDLE → MEM_REQ: new miss accepted, choose starting level
            if (miss_grant) begin
                slot_vaddr[free_slot] <= miss_vaddr[selected_req];
                slot_src[free_slot]   <= selected_req;
                slot_state[free_slot] <= PTW_MEM_REQ;
                if (pwc_hit && pwc2_hit) begin
                    // Double hit: skip both intermediate levels (SV39 only)
                    slot_level[free_slot]   <= LEVEL_BITS'(PT_LEVELS - 3);
                    slot_cur_ppn[free_slot] <= pwc2_cached_ppn;
                end else if (pwc_hit) begin
                    // PWC1 hit: skip top-level fetch
                    slot_level[free_slot]   <= LEVEL_BITS'(PT_LEVELS - 2);
                    slot_cur_ppn[free_slot] <= pwc_cached_ppn;
                end else begin
                    // Full walk from root
                    slot_level[free_slot]   <= LEVEL_BITS'(PT_LEVELS - 1);
                    slot_cur_ppn[free_slot] <= satp[PPN_WIDTH-1:0];
                end
                miss_rr_base <= REQ_BITS'((int'(selected_req) + 1) % NUM_REQUESTORS);
            end

            // MEM_REQ → MEM_RESP: memory request fired, save PTE byte address
            if (mem_req_fire) begin
                slot_state[mem_req_slot]      <= PTW_MEM_RESP;
                slot_pte_addr_r[mem_req_slot] <= req_pte_addr;
                mem_rr_base <= SLOT_BITS'((int'(mem_req_slot) + 1) % PTW_SIZE);
            end

            // MEM_RESP: non-leaf → descend; leaf → FILL
            if (mem_rsp_fire) begin
                if (slot_level[rsp_slot] > 0) begin
                    slot_state[rsp_slot]   <= PTW_MEM_REQ;
                    slot_cur_ppn[rsp_slot] <= rsp_pte_ppn;
                    slot_level[rsp_slot]   <= LEVEL_BITS'(int'(slot_level[rsp_slot]) - 1);
                end else begin
                    slot_state[rsp_slot] <= PTW_FILL;
                    slot_ppn[rsp_slot]   <= rsp_pte_ppn;
                    slot_flags[rsp_slot] <= rsp_pte_flags;
                end
            end

            // FILL → IDLE: fill acknowledged by TLB
            for (int s = 0; s < PTW_SIZE; s++) begin
                if ((slot_state[s] == PTW_FILL) &&
                    fill_valid[slot_src[s]] && fill_ready[slot_src[s]]) begin
                    slot_state[s] <= PTW_IDLE;
                end
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

        reg [`XLEN-1:0] fill_vaddr_w, fill_paddr_w;
        reg [7:0]       fill_flags_w;

        always_comb begin
            fill_vaddr_w = '0;
            fill_paddr_w = '0;
            fill_flags_w = '0;
            for (int s = 0; s < PTW_SIZE; s++) begin
                if (slot_fill_for_r[s]) begin
                    fill_vaddr_w = slot_vaddr[s];
                    fill_paddr_w = `XLEN'({slot_ppn[s], slot_vaddr[s][PAGE_OFFSET-1:0]});
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

    wire [PTW_SIZE-1:0] slot_awaiting_rsp;
    for (genvar s = 0; s < PTW_SIZE; s++) begin : g_rsp_ready
        assign slot_awaiting_rsp[s] = (slot_state[s] == PTW_MEM_RESP);
    end

    assign ptw_mem_if.req_valid       = mem_req_grant;
    assign ptw_mem_if.req_data.rw     = 1'b0;
    assign ptw_mem_if.req_data.addr   = req_pte_addr[`MEM_ADDR_WIDTH-1:ADDR_SHIFT];
    assign ptw_mem_if.req_data.data   = '0;
    assign ptw_mem_if.req_data.byteen = {DATA_SIZE{1'b1}};
    assign ptw_mem_if.req_data.flags  = '0;
    assign ptw_mem_if.req_data.tag    = TAG_WIDTH'(mem_req_slot);
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
    reg [PERF_CTR_BITS-1:0] perf_pwc_hits_r;
    reg [PERF_CTR_BITS-1:0] perf_pwc_misses_r;
    reg [PERF_CTR_BITS-1:0] perf_pwc2_hits_r;
    reg [PERF_CTR_BITS-1:0] perf_pwc2_misses_r;
    always_ff @(posedge clk) begin
        if (reset) begin
            perf_ptw_latency_r  <= '0;
            perf_pwc_hits_r     <= '0;
            perf_pwc_misses_r   <= '0;
            perf_pwc2_hits_r    <= '0;
            perf_pwc2_misses_r  <= '0;
        end else begin
            if (|slot_active)
                perf_ptw_latency_r <= perf_ptw_latency_r + PERF_CTR_BITS'(1);
            if (miss_grant && pwc_hit)
                perf_pwc_hits_r <= perf_pwc_hits_r + PERF_CTR_BITS'(1);
            if (miss_grant && !pwc_hit)
                perf_pwc_misses_r <= perf_pwc_misses_r + PERF_CTR_BITS'(1);
            if (miss_grant && pwc_hit && pwc2_hit)
                perf_pwc2_hits_r <= perf_pwc2_hits_r + PERF_CTR_BITS'(1);
            if (miss_grant && pwc_hit && !pwc2_hit)
                perf_pwc2_misses_r <= perf_pwc2_misses_r + PERF_CTR_BITS'(1);
        end
    end
    assign perf_ptw_latency  = perf_ptw_latency_r;
    assign perf_pwc_hits     = perf_pwc_hits_r;
    assign perf_pwc_misses   = perf_pwc_misses_r;
    assign perf_pwc2_hits    = perf_pwc2_hits_r;
    assign perf_pwc2_misses  = perf_pwc2_misses_r;
`else
    assign perf_ptw_latency_placeholder = 1'b0;
`endif

endmodule
