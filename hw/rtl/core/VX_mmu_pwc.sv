// Copyright 2024
// PWC: Page Walk Cache (HPCA14 Design 3)
// Generic direct-mapped cache for non-leaf PTW intermediate entries.
// Key = (parent_ppn [tag], child_vpn [index]); Value = child node PPN.
//
// SV32: one instance caches the L1→L0 pointer (vpn[1] index, satp_ppn tag).
//   1024 entries × (1+20+20) bits ≈ 5 KB.
// SV39: two instances — one for L2→L1 (vpn[2] index, satp_ppn tag) and one
//   for L1→L0 (vpn[1] index, L2_ppn tag). Each 512 entries × (1+36+36) bits.
//   A double hit reduces a 3-fetch walk to 1 fetch.
//
// Lookup is combinatorial (zero-cycle hit path).
// Fill is registered (one cycle after the corresponding memory response).
// The cache is fully invalidated on reset; no explicit flush port is needed
// because SATP is constant for the lifetime of a kernel execution.

`include "VX_define.vh"

module VX_mmu_pwc #(
    parameter PPN_WIDTH   = 20,   // physical page number width (SV32: 20 bits)
    parameter VPN_BITS    = 10,   // vpn1 field width (SV32: 10 bits)
    parameter NUM_ENTRIES = 1024  // 2^VPN_BITS; vpn1 is the direct index
) (
    input wire clk,
    input wire reset,

    // Lookup (combinatorial — result valid same cycle as inputs)
    input  wire [PPN_WIDTH-1:0] lookup_satp_ppn,
    input  wire [VPN_BITS-1:0]  lookup_vpn1,
    output wire                  hit,
    output wire [PPN_WIDTH-1:0]  l1_ppn,

    // Fill (registered — write occurs on the rising edge after fill_valid)
    input wire                   fill_valid,
    input wire [PPN_WIDTH-1:0]   fill_satp_ppn,
    input wire [VPN_BITS-1:0]    fill_vpn1,
    input wire [PPN_WIDTH-1:0]   fill_l1_ppn
);

    reg                 valid_r [NUM_ENTRIES];
    reg [PPN_WIDTH-1:0] tags_r  [NUM_ENTRIES];  // satp_ppn tag per entry
    reg [PPN_WIDTH-1:0] data_r  [NUM_ENTRIES];  // cached l1_ppn per entry

    // Combinatorial lookup: index by vpn1, compare tag against satp_ppn
    assign hit    = valid_r[lookup_vpn1] && (tags_r[lookup_vpn1] == lookup_satp_ppn);
    assign l1_ppn = data_r[lookup_vpn1];

    // Fill: write new entry on L1 memory response
    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < NUM_ENTRIES; i++) begin
                valid_r[i] <= 1'b0;
            end
        end else if (fill_valid) begin
            valid_r[fill_vpn1] <= 1'b1;
            tags_r[fill_vpn1]  <= fill_satp_ppn;
            data_r[fill_vpn1]  <= fill_l1_ppn;
        end
    end

endmodule
