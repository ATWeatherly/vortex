// Copyright © 2019-2026
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

// AXI4-Lite slave shim that lets a Zynq PS drive Vortex_axi's raw DCR
// sideband and reset/busy handshake. Register map (byte offsets):
//   0x00 CTRL      [0] vx_reset   RW, resets to 1 (Vortex held in reset at POR)
//   0x04 STATUS    [0] busy, [1] dcr_rsp_seen   RO
//   0x08 DCR_ADDR  [11:0]         RW
//   0x0C DCR_WDATA [31:0]         W: pulses a 1-cycle DCR write at DCR_ADDR
//   0x10 DCR_RDATA [31:0]         RO: last dcr_rsp_data observed
//   0x14 START                    W: pulses vx_start for 1 cycle (KMU launch)
//   0x1C MAGIC     0x56585A37     RO ("VXZ7")

module vortex_axil_shim #(
    parameter DCR_ADDR_BITS = 12,
    parameter DCR_DATA_BITS = 32
) (
    input  wire        clk,
    input  wire        resetn,

    // AXI4-Lite slave
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [7:0]  s_axi_awaddr,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    output wire [1:0]  s_axi_bresp,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    input  wire [7:0]  s_axi_araddr,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,

    // Vortex control
    output wire                     vx_reset,
    output wire                     vx_start,
    input  wire                     vx_busy,
    output wire                     dcr_req_valid,
    output wire                     dcr_req_rw,
    output wire [DCR_ADDR_BITS-1:0] dcr_req_addr,
    output wire [DCR_DATA_BITS-1:0] dcr_req_data,
    input  wire                     dcr_rsp_valid,
    input  wire [DCR_DATA_BITS-1:0] dcr_rsp_data
);
    localparam MAGIC = 32'h56585A37;

    reg vx_reset_r;
    reg [DCR_ADDR_BITS-1:0] dcr_addr_r;
    reg [DCR_DATA_BITS-1:0] dcr_wdata_r;
    reg [DCR_DATA_BITS-1:0] dcr_rdata_r;
    reg dcr_rsp_seen_r;
    reg dcr_wr_pulse_r;
    reg start_pulse_r;

    // single-outstanding write channel
    reg aw_ready_r, w_ready_r, b_valid_r;
    reg [7:0] awaddr_r;
    reg aw_got_r, w_got_r;
    reg [31:0] wdata_r;

    // read channel
    reg ar_ready_r, r_valid_r;
    reg [31:0] rdata_r;

    wire do_write = aw_got_r && w_got_r && !b_valid_r;

    always @(posedge clk) begin
        if (!resetn) begin
            vx_reset_r     <= 1'b1;
            dcr_addr_r     <= '0;
            dcr_wdata_r    <= '0;
            dcr_rdata_r    <= '0;
            dcr_rsp_seen_r <= 1'b0;
            dcr_wr_pulse_r <= 1'b0;
            start_pulse_r  <= 1'b0;
            aw_ready_r     <= 1'b1;
            w_ready_r      <= 1'b1;
            b_valid_r      <= 1'b0;
            aw_got_r       <= 1'b0;
            w_got_r        <= 1'b0;
            awaddr_r       <= '0;
            wdata_r        <= '0;
            ar_ready_r     <= 1'b1;
            r_valid_r      <= 1'b0;
            rdata_r        <= '0;
        end else begin
            dcr_wr_pulse_r <= 1'b0;
            start_pulse_r  <= 1'b0;

            if (dcr_rsp_valid) begin
                dcr_rdata_r    <= dcr_rsp_data;
                dcr_rsp_seen_r <= 1'b1;
            end

            // write address
            if (s_axi_awvalid && aw_ready_r) begin
                awaddr_r   <= s_axi_awaddr;
                aw_got_r   <= 1'b1;
                aw_ready_r <= 1'b0;
            end
            // write data
            if (s_axi_wvalid && w_ready_r) begin
                wdata_r   <= s_axi_wdata;
                w_got_r   <= 1'b1;
                w_ready_r <= 1'b0;
            end
            // commit
            if (do_write) begin
                case (awaddr_r[7:2])
                    6'h00: vx_reset_r <= wdata_r[0];
                    6'h02: dcr_addr_r <= wdata_r[DCR_ADDR_BITS-1:0];
                    6'h03: begin
                        dcr_wdata_r    <= wdata_r;
                        dcr_wr_pulse_r <= 1'b1;
                    end
                    6'h05: start_pulse_r <= 1'b1;
                    default:;
                endcase
                b_valid_r <= 1'b1;
                aw_got_r  <= 1'b0;
                w_got_r   <= 1'b0;
            end
            if (b_valid_r && s_axi_bready) begin
                b_valid_r  <= 1'b0;
                aw_ready_r <= 1'b1;
                w_ready_r  <= 1'b1;
            end

            // read
            if (s_axi_arvalid && ar_ready_r) begin
                case (s_axi_araddr[7:2])
                    6'h00: rdata_r <= {31'd0, vx_reset_r};
                    6'h01: rdata_r <= {30'd0, dcr_rsp_seen_r, vx_busy};
                    6'h02: rdata_r <= {{(32-DCR_ADDR_BITS){1'b0}}, dcr_addr_r};
                    6'h03: rdata_r <= dcr_wdata_r;
                    6'h04: rdata_r <= dcr_rdata_r;
                    6'h07: rdata_r <= MAGIC;
                    default: rdata_r <= 32'hDEADC0DE;
                endcase
                r_valid_r  <= 1'b1;
                ar_ready_r <= 1'b0;
            end
            if (r_valid_r && s_axi_rready) begin
                r_valid_r  <= 1'b0;
                ar_ready_r <= 1'b1;
            end
        end
    end

    assign s_axi_awready = aw_ready_r;
    assign s_axi_wready  = w_ready_r;
    assign s_axi_bvalid  = b_valid_r;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_arready = ar_ready_r;
    assign s_axi_rvalid  = r_valid_r;
    assign s_axi_rdata   = rdata_r;
    assign s_axi_rresp   = 2'b00;

    assign vx_reset      = vx_reset_r;
    assign vx_start      = start_pulse_r;
    assign dcr_req_valid = dcr_wr_pulse_r;
    assign dcr_req_rw    = 1'b1;
    assign dcr_req_addr  = dcr_addr_r;
    assign dcr_req_data  = dcr_wdata_r;

endmodule
