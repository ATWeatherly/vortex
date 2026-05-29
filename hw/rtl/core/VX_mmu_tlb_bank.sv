// Copyright 2024
// TLB bank: CAM-based address translation for one bank of a banked TLB.
//
// This module holds the per-bank CAM array, replacement (MRU) logic, the
// lookup/miss/replay FSM, and the miss/fill handshake for a single bank. It
// processes one (already arbitrated) request stream at a time. The banking
// wrapper (VX_mmu_tlb) fans NUM_REQS lanes across NUM_BANKS of these via a
// crossbar so that up to NUM_BANKS translations proceed concurrently, and a
// miss stalled in one bank does not block hits in the others.
//
// Source-lane encoding: the wrapper passes the originating LSU lane in `req_sel`
// and this module folds it into the low bits of the outgoing tag (TAG_WIDTH_OUT
// = TAG_WIDTH_IN + SOURCE_BITS) so the response path can route results back.

`include "VX_define.vh"
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off UNUSEDSIGNAL */

module VX_mmu_tlb_bank import VX_gpu_pkg::*; #(
    parameter DATA_SIZE      = DCACHE_WORD_SIZE,
    parameter ADDR_WIDTH     = DCACHE_ADDR_WIDTH,
    parameter TAG_WIDTH_IN   = DCACHE_TAG_WIDTH_BASE,
    parameter SOURCE_BITS    = 1,
    parameter FLAGS_WIDTH    = MEM_FLAGS_WIDTH,
    parameter TLB_SIZE       = 32,
    // Derived widths (parameters may reference earlier parameters)
    parameter DATA_WIDTH     = DATA_SIZE * 8,
    parameter TAG_WIDTH_OUT  = TAG_WIDTH_IN + SOURCE_BITS,
    parameter REQ_DATAW_IN   = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_SIZE + FLAGS_WIDTH + TAG_WIDTH_IN,
    parameter REQ_DATAW_OUT  = 1 + ADDR_WIDTH + DATA_WIDTH + DATA_SIZE + FLAGS_WIDTH + TAG_WIDTH_OUT
) (
    input wire clk,
    input wire reset,

    // Single (arbitrated) request stream in
    input  wire                     req_valid,
    input  wire [REQ_DATAW_IN-1:0]  req_data,
    input  wire [SOURCE_BITS-1:0]   req_sel,   // originating LSU lane
    output wire                     req_ready,

    // Single translated request stream out (tag carries source bits)
    output wire                     out_valid,
    output wire [REQ_DATAW_OUT-1:0] out_data,
    input  wire                     out_ready,

    // PTW miss/fill interface (arbitrated/serialized in the wrapper)
    output wire             miss_valid,
    input  wire             miss_ready,
    output wire [`XLEN-1:0] miss_vaddr,

    input  wire             fill_valid,
    output wire             fill_ready,
    input  wire [`XLEN-1:0] fill_vaddr,
    input  wire [`XLEN-1:0] fill_paddr,
    input  wire [7:0]       fill_flags,

`ifdef PERF_ENABLE
    output wire [PERF_CTR_BITS-1:0] perf_tlb_reads,
    output wire [PERF_CTR_BITS-1:0] perf_tlb_hits,
    output wire [PERF_CTR_BITS-1:0] perf_tlb_misses,
    output wire [PERF_CTR_BITS-1:0] perf_tlb_evictions,
    output wire [PERF_CTR_BITS-1:0] perf_ptw_walks
`else
    output wire             perf_placeholder
`endif
);

    localparam TLB_INDEX_BITS   = `CLOG2(TLB_SIZE);
    localparam PAGE_OFFSET_BITS = 12 - `CLOG2(DATA_SIZE);
`ifdef XLEN_32
    // SV32: VPN = 20 bits (vpn[1]:vpn[0] = 10+10), 4MB superpage
    localparam VPN_WIDTH            = 20;
    localparam PPN_WIDTH            = `MEM_ADDR_WIDTH - 12; // 20 bits
    localparam VPN_LEVEL_BITS       = 10;
    localparam SUPERPAGE_OFFSET_BITS = 22 - `CLOG2(DATA_SIZE); // 4MB page offset
`else
    // SV39: VPN = 27 bits (vpn[2]:vpn[1]:vpn[0] = 9+9+9), 2MB/1GB superpages
    localparam VPN_WIDTH            = 27;
    localparam PPN_WIDTH            = `MEM_ADDR_WIDTH - 12; // 36 bits for MEM_ADDR_WIDTH=48
    localparam VPN_LEVEL_BITS       = 9;
    localparam SUPERPAGE_OFFSET_BITS = 21 - `CLOG2(DATA_SIZE); // 2MB page offset
    localparam GIGAPAGE_OFFSET_BITS  = 30 - `CLOG2(DATA_SIZE); // 1GB page offset
`endif

    typedef struct packed {
        logic                 valid;
        logic                 mru;
        logic [1:0]           page_level;
        logic [VPN_WIDTH-1:0] vpn;
        logic [PPN_WIDTH-1:0] ppn;
        logic [7:0]           flags;
    } tlb_entry_t;

    tlb_entry_t tlb_entries [TLB_SIZE-1:0];

    typedef enum logic [1:0] {
        TLB_IDLE,
        TLB_READY,
        TLB_PTW_WAIT,
        TLB_REPLAY
    } tlb_state_t;

    tlb_state_t state;

    reg [REQ_DATAW_IN-1:0]   miss_buffer;
    reg [SOURCE_BITS-1:0]    miss_sel;
    reg [`XLEN-1:0]          miss_fill_paddr;
    reg miss_sent;
    reg [TLB_INDEX_BITS-1:0] victim_index;

    localparam ADDR_LSB_IN = TAG_WIDTH_IN + FLAGS_WIDTH + DATA_SIZE + DATA_WIDTH;
    localparam ADDR_LSB    = TAG_WIDTH_OUT + FLAGS_WIDTH + DATA_SIZE + DATA_WIDTH;

    wire use_miss_buffer = (state == TLB_REPLAY);
    wire [REQ_DATAW_IN-1:0] lookup_data = use_miss_buffer ? miss_buffer : req_data;
    wire [SOURCE_BITS-1:0]  lookup_sel  = use_miss_buffer ? miss_sel : req_sel;
    wire [ADDR_WIDTH-1:0] lookup_addr = lookup_data[ADDR_LSB_IN +: ADDR_WIDTH];
    // Extract VPN_WIDTH bits starting at PAGE_OFFSET_BITS (equivalent to byte addr bits [VPN_WIDTH+11:12])
    wire [VPN_WIDTH-1:0] lookup_vpn = lookup_addr[VPN_WIDTH + PAGE_OFFSET_BITS - 1 : PAGE_OFFSET_BITS];
    wire [TAG_WIDTH_IN-1:0] lookup_tag = lookup_data[TAG_WIDTH_IN-1:0];
    wire [TAG_WIDTH_OUT-1:0] lookup_tag_encoded;

    VX_bits_insert #(
        .N   (TAG_WIDTH_IN),
        .S   (SOURCE_BITS),
        .POS (0)
    ) tag_encode (
        .data_in  (lookup_tag),
        .ins_in   (lookup_sel),
        .data_out (lookup_tag_encoded)
    );

    wire [REQ_DATAW_OUT-1:0] lookup_data_encoded = {
        lookup_data[REQ_DATAW_IN-1:TAG_WIDTH_IN],
        lookup_tag_encoded
    };

    // CAM Lookup
`ifdef XLEN_32
    // SV32: level 0=4KB, level 1=4MB superpage, level 2=BARE bypass
    function automatic [VPN_WIDTH-1:0] vpn_mask(input [1:0] level);
        case (level)
            2'd0:    return 20'hFFFFF;  // 4KB: all 20 VPN bits
            2'd1:    return 20'hFFC00;  // 4MB: only vpn[1] (top 10 bits)
            default: return 20'h00000;  // BARE / unused
        endcase
    endfunction
`else
    // SV39: level 0=4KB, level 1=2MB, level 2=1GB
    function automatic [VPN_WIDTH-1:0] vpn_mask(input [1:0] level);
        case (level)
            2'd0:    return 27'h7FFFFFF;  // 4KB: all 27 VPN bits
            2'd1:    return 27'h7FFFE00;  // 2MB: vpn[2]:vpn[1] (top 18 bits)
            2'd2:    return 27'h7FC0000;  // 1GB: vpn[2] only (top 9 bits)
            default: return 27'h0000000;
        endcase
    endfunction
`endif

    wire [TLB_SIZE-1:0] cam_hit;
    for (genvar i = 0; i < TLB_SIZE; i++) begin : g_cam
        wire [VPN_WIDTH-1:0] mask_i = vpn_mask(tlb_entries[i].page_level);
        assign cam_hit[i] = tlb_entries[i].valid &&
                            ((tlb_entries[i].vpn & mask_i) == (lookup_vpn & mask_i));
    end

    wire tlb_hit = |cam_hit;

    reg [TLB_INDEX_BITS-1:0] hit_index;
    always_comb begin
        hit_index = '0;
        for (int j = TLB_SIZE-1; j >= 0; j--) begin
            if (cam_hit[j]) hit_index = j[TLB_INDEX_BITS-1:0];
        end
    end

    // Victim Selection (MRU-based)
    reg [TLB_INDEX_BITS-1:0] victim_candidate;
    reg found_invalid;
    wire all_mru;

    always_comb begin
        victim_candidate = '0;
        found_invalid = 1'b0;
        for (int j = TLB_SIZE-1; j >= 0; j--) begin
            if (!tlb_entries[j].valid) begin
                victim_candidate = j[TLB_INDEX_BITS-1:0];
                found_invalid = 1'b1;
            end
        end
        if (!found_invalid) begin
            for (int j = TLB_SIZE-1; j >= 0; j--) begin
                if (tlb_entries[j].valid && !tlb_entries[j].mru)
                    victim_candidate = j[TLB_INDEX_BITS-1:0];
            end
        end
    end

    wire [TLB_SIZE-1:0] entry_mru;
    for (genvar i = 0; i < TLB_SIZE; i++) begin : g_mru_check
        assign entry_mru[i] = tlb_entries[i].valid ? tlb_entries[i].mru : 1'b0;
    end
    assign all_mru = &entry_mru;

    // Address Translation
    wire [PPN_WIDTH-1:0] hit_ppn   = tlb_entries[hit_index].ppn;
    wire [1:0]           hit_level = tlb_entries[hit_index].page_level;

    reg [ADDR_WIDTH-1:0] cam_translated_addr;
    always_comb begin
`ifdef XLEN_32
        // SV32: level 0=4KB, level 1=4MB superpage
        case (hit_level)
            2'd0:    cam_translated_addr = {hit_ppn, lookup_addr[PAGE_OFFSET_BITS-1:0]};
            2'd1:    cam_translated_addr = {hit_ppn[PPN_WIDTH-1:VPN_LEVEL_BITS], lookup_addr[SUPERPAGE_OFFSET_BITS-1:0]};
            default: cam_translated_addr = lookup_addr;
        endcase
`else
        // SV39: level 0=4KB, level 1=2MB, level 2=1GB
        case (hit_level)
            2'd0:    cam_translated_addr = {hit_ppn, lookup_addr[PAGE_OFFSET_BITS-1:0]};
            2'd1:    cam_translated_addr = {hit_ppn[PPN_WIDTH-1:VPN_LEVEL_BITS], lookup_addr[SUPERPAGE_OFFSET_BITS-1:0]};
            2'd2:    cam_translated_addr = {hit_ppn[PPN_WIDTH-1:2*VPN_LEVEL_BITS], lookup_addr[GIGAPAGE_OFFSET_BITS-1:0]};
            default: cam_translated_addr = lookup_addr;
        endcase
`endif
    end

    // PPN is in fill_paddr[MEM_ADDR_WIDTH-1:12]; combine with sub-page offset from request
    wire [ADDR_WIDTH-1:0] replay_paddr = {miss_fill_paddr[`MEM_ADDR_WIDTH-1:12], lookup_addr[PAGE_OFFSET_BITS-1:0]};
    wire [ADDR_WIDTH-1:0] translated_addr = use_miss_buffer ? replay_paddr : cam_translated_addr;

    // State Machine
    wire input_handshake = req_valid && req_ready;
    wire output_handshake;

    always_ff @(posedge clk) begin
        if (reset) begin
            state <= TLB_IDLE;
            miss_buffer <= '0;
            miss_sel <= '0;
            miss_fill_paddr <= '0;
            miss_sent <= 1'b0;
            victim_index <= '0;
            for (int i = 0; i < TLB_SIZE; i++) begin
                tlb_entries[i].valid      <= 1'b0;
                tlb_entries[i].mru        <= 1'b0;
                tlb_entries[i].page_level <= 2'd0;
                tlb_entries[i].vpn        <= '0;
                tlb_entries[i].ppn        <= '0;
                tlb_entries[i].flags      <= '0;
            end
        end else begin
            case (state)
                TLB_IDLE: begin
                    state <= TLB_READY;
                end

                TLB_READY: begin
                    if (input_handshake) begin
                        if (tlb_hit) begin
                            tlb_entries[hit_index].mru <= 1'b1;
                            if (all_mru) begin
                                for (int i = 0; i < TLB_SIZE; i++) begin
                                    if (i[TLB_INDEX_BITS-1:0] != hit_index)
                                        tlb_entries[i].mru <= 1'b0;
                                end
                            end
                        end else begin
                            miss_buffer <= req_data;
                            miss_sel <= req_sel;
                            victim_index <= victim_candidate;
                            state <= TLB_PTW_WAIT;
                        end
                    end
                end

                TLB_PTW_WAIT: begin
                    if (miss_valid && miss_ready) miss_sent <= 1'b1;

                    if (fill_valid && fill_ready) begin
                        miss_fill_paddr <= fill_paddr;
                        tlb_entries[victim_index].valid      <= 1'b1;
                        tlb_entries[victim_index].mru        <= 1'b1;
                        tlb_entries[victim_index].page_level <= 2'd0;
                        tlb_entries[victim_index].vpn        <= fill_vaddr[VPN_WIDTH+11:12];
                        tlb_entries[victim_index].ppn        <= fill_paddr[`MEM_ADDR_WIDTH-1:12];
                        tlb_entries[victim_index].flags      <= fill_flags;

                        if (all_mru) begin
                            for (int i = 0; i < TLB_SIZE; i++) begin
                                if (i[TLB_INDEX_BITS-1:0] != victim_index)
                                    tlb_entries[i].mru <= 1'b0;
                            end
                        end
                        state <= TLB_REPLAY;
                        miss_sent <= 1'b0;
                    end
                end

                TLB_REPLAY: begin
                    if (output_handshake) state <= TLB_READY;
                end

                default: state <= TLB_IDLE;
            endcase
        end
    end

    // Control Signals
    assign req_ready = (state == TLB_READY) && out_ready;

    assign out_valid = (state == TLB_READY && input_handshake && tlb_hit) ||
                       (state == TLB_REPLAY);

    assign output_handshake = out_valid && out_ready;

    assign out_data = {
        lookup_data_encoded[REQ_DATAW_OUT-1],
        translated_addr,
        lookup_data_encoded[ADDR_LSB-1:0]
    };

    // Miss/Fill Interface
    wire [ADDR_WIDTH-1:0] miss_buffer_addr = miss_buffer[ADDR_LSB_IN +: ADDR_WIDTH];
    wire [VPN_WIDTH-1:0] miss_buffer_vpn = miss_buffer_addr[VPN_WIDTH + PAGE_OFFSET_BITS - 1 : PAGE_OFFSET_BITS];

    assign miss_valid = (state == TLB_PTW_WAIT) && !miss_sent;
    // Reconstruct full virtual address: zero-extend VPN into XLEN, page-align
    assign miss_vaddr = `XLEN'({miss_buffer_vpn, {12{1'b0}}});
    assign fill_ready = (state == TLB_PTW_WAIT) && miss_sent;

    // Performance Counters
`ifdef PERF_ENABLE
    reg [PERF_CTR_BITS-1:0] perf_reads_r;
    reg [PERF_CTR_BITS-1:0] perf_hits_r;
    reg [PERF_CTR_BITS-1:0] perf_misses_r;
    reg [PERF_CTR_BITS-1:0] perf_evictions_r;
    reg [PERF_CTR_BITS-1:0] perf_walks_r;
    wire victim_was_valid = tlb_entries[victim_index].valid;

    always @(posedge clk) begin
        if (reset) begin
            perf_reads_r     <= '0;
            perf_hits_r      <= '0;
            perf_misses_r    <= '0;
            perf_evictions_r <= '0;
            perf_walks_r     <= '0;
        end else begin
            if (state == TLB_READY && input_handshake)
                perf_reads_r <= perf_reads_r + PERF_CTR_BITS'(1);
            if (state == TLB_READY && input_handshake && tlb_hit)
                perf_hits_r <= perf_hits_r + PERF_CTR_BITS'(1);
            if (miss_valid && miss_ready)
                perf_misses_r <= perf_misses_r + PERF_CTR_BITS'(1);
            if (fill_valid && fill_ready)
                perf_walks_r <= perf_walks_r + PERF_CTR_BITS'(1);
            if (fill_valid && fill_ready && victim_was_valid)
                perf_evictions_r <= perf_evictions_r + PERF_CTR_BITS'(1);
        end
    end

    assign perf_tlb_reads     = perf_reads_r;
    assign perf_tlb_hits      = perf_hits_r;
    assign perf_tlb_misses    = perf_misses_r;
    assign perf_tlb_evictions = perf_evictions_r;
    assign perf_ptw_walks     = perf_walks_r;
`else
    assign perf_placeholder = 1'b0;
`endif

endmodule
