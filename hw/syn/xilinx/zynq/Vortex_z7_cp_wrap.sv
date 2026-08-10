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

`include "VX_define.vh"
`include "vortex_afu.vh"

// Zynq-7000 integration top WITH the Command Processor: VX_afu_wrap
// (AXI-lite ctrl incl. the CP register window + arbitrated device-memory
// master + host-memory master for the CP command ring) adapted to the PS:
//   s_axi_ctrl  <- M_AXI_GP0 (64K window)
//   m_axi_mem   -> S_AXI_HP0 (device memory: kernel/buffers/IO in DDR)
//   m_axi_host  -> S_AXI_HP0 (command ring in host DDR)
// The AFU masters emit only addr/id/len; burst/size/cache/prot/qos are
// driven here with full-width INCR constants (matching what the XRT
// platforms infer).
module Vortex_z7_cp_wrap #(
    parameter C_M_AXI_MEM_DATA_WIDTH = 64,
    parameter C_M_AXI_MEM_ADDR_WIDTH = 32,
    parameter C_M_AXI_MEM_ID_WIDTH   = 32
) (
    input  wire                                 clk,
    input  wire                                 resetn,

    // AXI4-Lite control slave (VX_afu_ctrl + CP window)
    input  wire                                 s_axi_ctrl_awvalid,
    output wire                                 s_axi_ctrl_awready,
    input  wire [15:0]                          s_axi_ctrl_awaddr,
    input  wire                                 s_axi_ctrl_wvalid,
    output wire                                 s_axi_ctrl_wready,
    input  wire [31:0]                          s_axi_ctrl_wdata,
    input  wire [3:0]                           s_axi_ctrl_wstrb,
    output wire                                 s_axi_ctrl_bvalid,
    input  wire                                 s_axi_ctrl_bready,
    output wire [1:0]                           s_axi_ctrl_bresp,
    input  wire                                 s_axi_ctrl_arvalid,
    output wire                                 s_axi_ctrl_arready,
    input  wire [15:0]                          s_axi_ctrl_araddr,
    output wire                                 s_axi_ctrl_rvalid,
    input  wire                                 s_axi_ctrl_rready,
    output wire [31:0]                          s_axi_ctrl_rdata,
    output wire [1:0]                           s_axi_ctrl_rresp,

    // AXI4 device-memory master
    output wire                                 m_axi_mem_awvalid,
    input  wire                                 m_axi_mem_awready,
    output wire [C_M_AXI_MEM_ADDR_WIDTH-1:0]    m_axi_mem_awaddr,
    output wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_mem_awid,
    output wire [7:0]                           m_axi_mem_awlen,
    output wire [2:0]                           m_axi_mem_awsize,
    output wire [1:0]                           m_axi_mem_awburst,
    output wire [1:0]                           m_axi_mem_awlock,
    output wire [3:0]                           m_axi_mem_awcache,
    output wire [2:0]                           m_axi_mem_awprot,
    output wire [3:0]                           m_axi_mem_awqos,
    output wire                                 m_axi_mem_wvalid,
    input  wire                                 m_axi_mem_wready,
    output wire [C_M_AXI_MEM_DATA_WIDTH-1:0]    m_axi_mem_wdata,
    output wire [C_M_AXI_MEM_DATA_WIDTH/8-1:0]  m_axi_mem_wstrb,
    output wire                                 m_axi_mem_wlast,
    output wire                                 m_axi_mem_arvalid,
    input  wire                                 m_axi_mem_arready,
    output wire [C_M_AXI_MEM_ADDR_WIDTH-1:0]    m_axi_mem_araddr,
    output wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_mem_arid,
    output wire [7:0]                           m_axi_mem_arlen,
    output wire [2:0]                           m_axi_mem_arsize,
    output wire [1:0]                           m_axi_mem_arburst,
    output wire [1:0]                           m_axi_mem_arlock,
    output wire [3:0]                           m_axi_mem_arcache,
    output wire [2:0]                           m_axi_mem_arprot,
    output wire [3:0]                           m_axi_mem_arqos,
    input  wire                                 m_axi_mem_rvalid,
    output wire                                 m_axi_mem_rready,
    input  wire [C_M_AXI_MEM_DATA_WIDTH-1:0]    m_axi_mem_rdata,
    input  wire                                 m_axi_mem_rlast,
    input  wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_mem_rid,
    input  wire [1:0]                           m_axi_mem_rresp,
    input  wire                                 m_axi_mem_bvalid,
    output wire                                 m_axi_mem_bready,
    input  wire [1:0]                           m_axi_mem_bresp,
    input  wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_mem_bid,

    // AXI4 host-memory master (CP command ring)
    output wire                                 m_axi_host_awvalid,
    input  wire                                 m_axi_host_awready,
    output wire [C_M_AXI_MEM_ADDR_WIDTH-1:0]    m_axi_host_awaddr,
    output wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_host_awid,
    output wire [7:0]                           m_axi_host_awlen,
    output wire [2:0]                           m_axi_host_awsize,
    output wire [1:0]                           m_axi_host_awburst,
    output wire [1:0]                           m_axi_host_awlock,
    output wire [3:0]                           m_axi_host_awcache,
    output wire [2:0]                           m_axi_host_awprot,
    output wire [3:0]                           m_axi_host_awqos,
    output wire                                 m_axi_host_wvalid,
    input  wire                                 m_axi_host_wready,
    output wire [C_M_AXI_MEM_DATA_WIDTH-1:0]    m_axi_host_wdata,
    output wire [C_M_AXI_MEM_DATA_WIDTH/8-1:0]  m_axi_host_wstrb,
    output wire                                 m_axi_host_wlast,
    output wire                                 m_axi_host_arvalid,
    input  wire                                 m_axi_host_arready,
    output wire [C_M_AXI_MEM_ADDR_WIDTH-1:0]    m_axi_host_araddr,
    output wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_host_arid,
    output wire [7:0]                           m_axi_host_arlen,
    output wire [2:0]                           m_axi_host_arsize,
    output wire [1:0]                           m_axi_host_arburst,
    output wire [1:0]                           m_axi_host_arlock,
    output wire [3:0]                           m_axi_host_arcache,
    output wire [2:0]                           m_axi_host_arprot,
    output wire [3:0]                           m_axi_host_arqos,
    input  wire                                 m_axi_host_rvalid,
    output wire                                 m_axi_host_rready,
    input  wire [C_M_AXI_MEM_DATA_WIDTH-1:0]    m_axi_host_rdata,
    input  wire                                 m_axi_host_rlast,
    input  wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_host_rid,
    input  wire [1:0]                           m_axi_host_rresp,
    input  wire                                 m_axi_host_bvalid,
    output wire                                 m_axi_host_bready,
    input  wire [1:0]                           m_axi_host_bresp,
    input  wire [C_M_AXI_MEM_ID_WIDTH-1:0]      m_axi_host_bid,

    output wire                                 interrupt
);
    localparam [2:0] AXSIZE_FULL = 3'($clog2(C_M_AXI_MEM_DATA_WIDTH / 8));

    // AFU masters emit INCR full-width bursts; drive the sidebands here.
    assign m_axi_mem_awsize   = AXSIZE_FULL;
    assign m_axi_mem_awburst  = 2'b01;
    assign m_axi_mem_awlock   = 2'b00;
    assign m_axi_mem_awcache  = 4'b0011;
    assign m_axi_mem_awprot   = 3'b000;
    assign m_axi_mem_awqos    = 4'b0000;
    assign m_axi_mem_arsize   = AXSIZE_FULL;
    assign m_axi_mem_arburst  = 2'b01;
    assign m_axi_mem_arlock   = 2'b00;
    assign m_axi_mem_arcache  = 4'b0011;
    assign m_axi_mem_arprot   = 3'b000;
    assign m_axi_mem_arqos    = 4'b0000;

    assign m_axi_host_awsize  = AXSIZE_FULL;
    assign m_axi_host_awburst = 2'b01;
    assign m_axi_host_awlock  = 2'b00;
    assign m_axi_host_awcache = 4'b0011;
    assign m_axi_host_awprot  = 3'b000;
    assign m_axi_host_awqos   = 4'b0000;
    assign m_axi_host_arsize  = AXSIZE_FULL;
    assign m_axi_host_arburst = 2'b01;
    assign m_axi_host_arlock  = 2'b00;
    assign m_axi_host_arcache = 4'b0011;
    assign m_axi_host_arprot  = 3'b000;
    assign m_axi_host_arqos   = 4'b0000;

    VX_afu_wrap #(
        .C_S_AXI_CTRL_ADDR_WIDTH (16),
        .C_S_AXI_CTRL_DATA_WIDTH (32),
        .C_M_AXI_MEM_ID_WIDTH    (C_M_AXI_MEM_ID_WIDTH),
        .C_M_AXI_MEM_DATA_WIDTH  (C_M_AXI_MEM_DATA_WIDTH),
        .C_M_AXI_MEM_ADDR_WIDTH  (C_M_AXI_MEM_ADDR_WIDTH),
        .C_M_AXI_MEM_NUM_BANKS   (1)
    ) afu (
        .clk                 (clk),
        .reset               (~resetn),

        .m_axi_mem_0_awvalid (m_axi_mem_awvalid),
        .m_axi_mem_0_awready (m_axi_mem_awready),
        .m_axi_mem_0_awaddr  (m_axi_mem_awaddr),
        .m_axi_mem_0_awid    (m_axi_mem_awid),
        .m_axi_mem_0_awlen   (m_axi_mem_awlen),
        .m_axi_mem_0_wvalid  (m_axi_mem_wvalid),
        .m_axi_mem_0_wready  (m_axi_mem_wready),
        .m_axi_mem_0_wdata   (m_axi_mem_wdata),
        .m_axi_mem_0_wstrb   (m_axi_mem_wstrb),
        .m_axi_mem_0_wlast   (m_axi_mem_wlast),
        .m_axi_mem_0_arvalid (m_axi_mem_arvalid),
        .m_axi_mem_0_arready (m_axi_mem_arready),
        .m_axi_mem_0_araddr  (m_axi_mem_araddr),
        .m_axi_mem_0_arid    (m_axi_mem_arid),
        .m_axi_mem_0_arlen   (m_axi_mem_arlen),
        .m_axi_mem_0_rvalid  (m_axi_mem_rvalid),
        .m_axi_mem_0_rready  (m_axi_mem_rready),
        .m_axi_mem_0_rdata   (m_axi_mem_rdata),
        .m_axi_mem_0_rlast   (m_axi_mem_rlast),
        .m_axi_mem_0_rid     (m_axi_mem_rid),
        .m_axi_mem_0_rresp   (m_axi_mem_rresp),
        .m_axi_mem_0_bvalid  (m_axi_mem_bvalid),
        .m_axi_mem_0_bready  (m_axi_mem_bready),
        .m_axi_mem_0_bresp   (m_axi_mem_bresp),
        .m_axi_mem_0_bid     (m_axi_mem_bid),

        `AXI_HOST_ARGS,

        .s_axi_ctrl_awvalid  (s_axi_ctrl_awvalid),
        .s_axi_ctrl_awready  (s_axi_ctrl_awready),
        .s_axi_ctrl_awaddr   (s_axi_ctrl_awaddr),
        .s_axi_ctrl_wvalid   (s_axi_ctrl_wvalid),
        .s_axi_ctrl_wready   (s_axi_ctrl_wready),
        .s_axi_ctrl_wdata    (s_axi_ctrl_wdata),
        .s_axi_ctrl_wstrb    (s_axi_ctrl_wstrb),
        .s_axi_ctrl_arvalid  (s_axi_ctrl_arvalid),
        .s_axi_ctrl_arready  (s_axi_ctrl_arready),
        .s_axi_ctrl_araddr   (s_axi_ctrl_araddr),
        .s_axi_ctrl_rvalid   (s_axi_ctrl_rvalid),
        .s_axi_ctrl_rready   (s_axi_ctrl_rready),
        .s_axi_ctrl_rdata    (s_axi_ctrl_rdata),
        .s_axi_ctrl_rresp    (s_axi_ctrl_rresp),
        .s_axi_ctrl_bvalid   (s_axi_ctrl_bvalid),
        .s_axi_ctrl_bready   (s_axi_ctrl_bready),
        .s_axi_ctrl_bresp    (s_axi_ctrl_bresp),

        .interrupt           (interrupt)
    );

endmodule
