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

// One DSP48E2 MACC cell of the 512-MAC tensor block (balanced-design spec phase 3, option (a)).
//
//   P <= (gs ? 0 : P) + A * B      when vld (P holds otherwise)
//
// A = two packed int8 activations (27-bit, fields 18 bits apart, see VX_tcu_mac_block),
// B = one sign/zero-extended weight byte (18-bit). The block guarantees at most 4
// products are accumulated per packed group, which keeps the two packed fields
// exact (|S0| <= 4 * 2^14 < 2^17 signed, <= 4 * 255^2 < 2^18 unsigned).
//
// Register map onto the DSP48E2: A1/B1 = a_r1/b_r1, A2/B2 = a_r2/b_r2, M = m_r,
// P = p_r with the Z-mux driven by gs (0 or P) and CEP = vld. Latency from a/b to
// p_r: 4 clocks (A1, A2, M, P). The vld/gs controls must be presented aligned with
// the operands' M stage, i.e. delayed 3 clocks by the caller.
module VX_tcu_mac_dsp (
    input  wire               clk,
    input  wire signed [26:0] a,
    input  wire signed [17:0] b,
    input  wire               vld,   // aligned with m_r (operands entered 3 clocks earlier)
    input  wire               gs,    // first product of a packed group: restart P
    output wire signed [47:0] p
);
`ifdef SYNTHESIS
    (* use_dsp = "yes" *)
`endif
    logic signed [26:0] a_r1, a_r2;
    logic signed [17:0] b_r1, b_r2;
    logic signed [44:0] m_r;
    logic signed [47:0] p_r;

    always_ff @(posedge clk) begin
        a_r1 <= a;
        b_r1 <= b;
        a_r2 <= a_r1;
        b_r2 <= b_r1;
        m_r  <= a_r2 * b_r2;
        if (vld) begin
            p_r <= (gs ? 48'sd0 : p_r) + 48'(m_r);
        end
    end

    assign p = p_r;

endmodule
