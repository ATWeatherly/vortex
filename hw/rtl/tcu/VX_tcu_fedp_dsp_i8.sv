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

// DSP-centric int8 dot-product pair (VX_CFG_TCU_DSP_I8).
//
// Computes TWO FEDP results that share the same B column (two consecutive
// rows i = 2p, 2p+1 of the TC_M x TC_N grid):
//     d_val0 = c_val0 + sum_e a_row0[e] * b_col[e]
//     d_val1 = c_val1 + sum_e a_row1[e] * b_col[e]      (int32, wrapping)
// with ONE DSP48E2 per element pair: the two activations of an element are
// packed into the 30-bit A port SEP bits apart and multiplied by the shared
// weight byte on the 18-bit B port,
//     P = (a1 * 2^SEP + a0) * b = (a1*b) * 2^SEP + (a0*b),
// so the sum over the NE elements is  S1 * 2^SEP + S0.  With SEP = 21 the low
// field holds |S0| <= NE * 2^14 (signed) or NE * 255^2 (unsigned) for NE <= 16
// without touching the high field, and the extraction is exact:
//     S0 = sext(P[SEP-1:0])            (zext when unsigned)
//     S1 = (P >>> SEP) + P[SEP-1]      (borrow correction; no term when unsigned)
// Integer formats only (TCU_I8_ID / TCU_U8_ID); other formats return 0.
// Bit-exact with the TFR int8 path, which also produces exact int32 (its
// norm_round stage reconstructs two's complement and adds C's high bits).
//
// Latency (LATENCY = `VX_CFG_TCU_LATENCY, 4 or 5):
//   S1 operand pack register (DSP A/B input registers)
//   S2[.S3] product register(s) (DSP M [+ P] registers; 2 stages at LATENCY 5)
//   S4 partial sums of 4 products (registered)
//   S5 final sum + field extraction + C add (registered output)
module VX_tcu_fedp_dsp_i8 import VX_tcu_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter LATENCY = 0,
    parameter N = TCU_TC_K          // words per row (4 int8 elements each)
) (
    input  wire clk,
    input  wire reset,
    input  wire enable,
    input  wire [4:0]          fmt_s,
    input  wire [4:0]          fmt_d,
    input  wire [N-1:0][31:0]  a_row0,
    input  wire [N-1:0][31:0]  a_row1,
    input  wire [N-1:0][31:0]  b_col,
    input  wire [31:0]         c_val0,
    input  wire [31:0]         c_val1,
    output wire [31:0]         d_val0,
    output wire [31:0]         d_val1
);
    `UNUSED_SPARAM (INSTANCE_ID)
    `UNUSED_VAR (fmt_d)
    `UNUSED_VAR (reset)

    localparam NE  = 4 * N;         // int8 elements per row = DSP48E2 count
    localparam SEP = 21;            // packed-field separation (bits)
    localparam AW  = 30;            // DSP48E2 A port
    localparam BW  = 18;            // DSP48E2 B port
    localparam PW  = 48;            // DSP48E2 P
    localparam NG  = (NE + 3) / 4;  // partial-sum groups of 4

    // Field bound: |S0| < 2^(SEP-1) (signed) / S0 < 2^SEP (unsigned) needs NE * 2^16 <= 2^SEP.
    `STATIC_ASSERT (NE <= 16, ("VX_tcu_fedp_dsp_i8: NE=%0d exceeds the packed-field bound (SEP=%0d)", NE, SEP))

    localparam PROD_STAGES   = LATENCY - 3;   // 1 (LATENCY 4) or 2 (LATENCY 5)
    `STATIC_ASSERT (PROD_STAGES == 1 || PROD_STAGES == 2,
        ("VX_tcu_fedp_dsp_i8: unsupported LATENCY=%0d (expected 4 or 5)", LATENCY))

    // -----------------------------------------------------------------------
    // S1: classify + pack operands (registered: becomes the DSP A/B input regs)
    // -----------------------------------------------------------------------
    wire is_int    = tcu_fmt_is_int(fmt_s);
    wire is_signed = tcu_fmt_is_signed_int(fmt_s[TCU_FMT_WIDTH-2:0]);
    wire is_i8     = is_int && (fmt_s[TCU_FMT_WIDTH-2:0] == TCU_I8_ID[TCU_FMT_WIDTH-2:0]
                             || fmt_s[TCU_FMT_WIDTH-2:0] == TCU_U8_ID[TCU_FMT_WIDTH-2:0]);

    logic signed [AW-1:0] a_pack_r [NE];
    logic signed [BW-1:0] b_ext_r  [NE];
    logic                 s1_signed_r, s1_valid_r;
    logic [31:0]          s1_c0_r, s1_c1_r;

    for (genvar e = 0; e < NE; ++e) begin : g_pack
        wire [7:0] raw_a0 = a_row0[e/4][(e%4)*8 +: 8];
        wire [7:0] raw_a1 = a_row1[e/4][(e%4)*8 +: 8];
        wire [7:0] raw_b  = b_col [e/4][(e%4)*8 +: 8];
        wire signed [8:0] s_a0 = is_signed ? $signed({raw_a0[7], raw_a0}) : $signed({1'b0, raw_a0});
        wire signed [8:0] s_a1 = is_signed ? $signed({raw_a1[7], raw_a1}) : $signed({1'b0, raw_a1});
        wire signed [8:0] s_b  = is_signed ? $signed({raw_b[7],  raw_b})  : $signed({1'b0, raw_b});
        // A = a1 * 2^SEP + a0 : low SEP bits = a0 (sign-extended), high bits = a1 + (a0 < 0 ? -1 : 0)
        wire [SEP-1:0]    a_lo = {{(SEP-9){s_a0[8]}}, s_a0};
        wire [AW-SEP-1:0] a_hi = (AW-SEP)'(s_a1) - (AW-SEP)'({{(AW-SEP-1){1'b0}}, s_a0[8]});
        always_ff @(posedge clk) begin
            if (enable) begin
                a_pack_r[e] <= $signed({a_hi, a_lo});
                b_ext_r[e]  <= $signed(BW'(s_b));
            end
        end
    end

    always_ff @(posedge clk) begin
        if (enable) begin
            s1_signed_r <= is_signed;
            s1_valid_r  <= is_i8;
            s1_c0_r     <= c_val0;
            s1_c1_r     <= c_val1;
        end
    end

    // -----------------------------------------------------------------------
    // S2[.S3]: one 30x18 signed product per element (DSP48E2 M [+ P] register)
    // -----------------------------------------------------------------------
    logic signed [PW-1:0] prod_r [NE];
    for (genvar e = 0; e < NE; ++e) begin : g_mul
    `ifdef SYNTHESIS
        (* use_dsp = "yes" *)
    `endif
        logic signed [PW-1:0] prod_m;
        always_ff @(posedge clk) begin
            if (enable) begin
                prod_m <= a_pack_r[e] * b_ext_r[e];
            end
        end
        if (PROD_STAGES == 2) begin : g_preg
            always_ff @(posedge clk) begin
                if (enable) prod_r[e] <= prod_m;
            end
        end else begin : g_nopreg
            assign prod_r[e] = prod_m;
        end
    end

    wire                 s3_signed, s3_valid;
    wire [31:0]          s3_c0, s3_c1;
    VX_pipe_register #(
        .DATAW (2 + 64),
        .DEPTH (PROD_STAGES)
    ) pipe_meta_prod (
        .clk      (clk),
        .reset    (1'b0),
        .enable   (enable),
        .data_in  ({s1_signed_r, s1_valid_r, s1_c0_r, s1_c1_r}),
        .data_out ({s3_signed,   s3_valid,   s3_c0,   s3_c1})
    );

    // -----------------------------------------------------------------------
    // S4: partial sums of up to 4 products (registered)
    // -----------------------------------------------------------------------
    logic signed [PW-1:0] psum_r [NG];
    for (genvar g = 0; g < NG; ++g) begin : g_psum
        logic signed [PW-1:0] psum;
        always_comb begin
            psum = '0;
            for (int q = 0; q < 4; ++q) begin
                if (g*4 + q < NE) psum = psum + prod_r[g*4 + q];
            end
        end
        always_ff @(posedge clk) begin
            if (enable) psum_r[g] <= psum;
        end
    end

    wire                 s4_signed, s4_valid;
    wire [31:0]          s4_c0, s4_c1;
    VX_pipe_register #(
        .DATAW (2 + 64),
        .DEPTH (1)
    ) pipe_meta_psum (
        .clk      (clk),
        .reset    (1'b0),
        .enable   (enable),
        .data_in  ({s3_signed, s3_valid, s3_c0, s3_c1}),
        .data_out ({s4_signed, s4_valid, s4_c0, s4_c1})
    );

    // -----------------------------------------------------------------------
    // S5: final sum, field extraction, C add (registered output)
    // -----------------------------------------------------------------------
    logic signed [PW-1:0] total;
    always_comb begin
        total = '0;
        for (int g = 0; g < NG; ++g) total = total + psum_r[g];
    end

    wire [SEP-1:0]    f_lo  = total[SEP-1:0];
    wire [PW-SEP-1:0] f_hi  = total[PW-1:SEP];           // arithmetic shift: total is signed
    wire [31:0] s0 = s4_signed ? 32'($signed(f_lo)) : 32'(f_lo);
    wire [31:0] s1 = s4_signed ? (32'($signed(f_hi)) + 32'(f_lo[SEP-1])) : 32'($signed(f_hi));

    logic [31:0] d0_r, d1_r;
    always_ff @(posedge clk) begin
        if (enable) begin
            d0_r <= s4_valid ? (s4_c0 + s0) : 32'b0;
            d1_r <= s4_valid ? (s4_c1 + s1) : 32'b0;
        end
    end

    assign d_val0 = d0_r;
    assign d_val1 = d1_r;

    // No format assertion here: the FEDP pipe has no valid qualifier (the core
    // gates results through its header FIFO), so fmt_s carries idle/random
    // values between micro-ops. Non-int8 formats simply produce 0.

endmodule
