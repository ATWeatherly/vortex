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

// Zynq-7000 integration top: vortex_axil_shim (PS control via AXI4-Lite on
// M_AXI_GP0) + Vortex_axi with its per-bank array ports flattened to a single
// AXI4 master (PS DDR via S_AXI_HP0). Vortex is held in reset by the shim's
// CTRL[0] (power-on 1) until the host has programmed the startup DCRs.
module Vortex_z7_wrap #(
    parameter C_M_AXI_MEM_DATA_WIDTH = 64,
    parameter C_M_AXI_MEM_ADDR_WIDTH = 32,
    parameter C_M_AXI_MEM_ID_WIDTH   = 32
) (
    input  wire                                 clk,
    input  wire                                 resetn,

    // AXI4-Lite control slave (vortex_axil_shim register file)
    input  wire                                 s_axi_ctrl_awvalid,
    output wire                                 s_axi_ctrl_awready,
    input  wire [7:0]                           s_axi_ctrl_awaddr,
    input  wire                                 s_axi_ctrl_wvalid,
    output wire                                 s_axi_ctrl_wready,
    input  wire [31:0]                          s_axi_ctrl_wdata,
    input  wire [3:0]                           s_axi_ctrl_wstrb,
    output wire                                 s_axi_ctrl_bvalid,
    input  wire                                 s_axi_ctrl_bready,
    output wire [1:0]                           s_axi_ctrl_bresp,
    input  wire                                 s_axi_ctrl_arvalid,
    output wire                                 s_axi_ctrl_arready,
    input  wire [7:0]                           s_axi_ctrl_araddr,
    output wire                                 s_axi_ctrl_rvalid,
    input  wire                                 s_axi_ctrl_rready,
    output wire [31:0]                          s_axi_ctrl_rdata,
    output wire [1:0]                           s_axi_ctrl_rresp,

    // AXI4 memory master
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

    output wire                                 vx_busy
);

    wire                          shim_vx_reset;
    wire                          shim_vx_start;
    wire                          dcr_req_valid;
    wire                          dcr_req_rw;
    wire [`VX_DCR_ADDR_BITS-1:0]  dcr_req_addr;
    wire [`VX_DCR_DATA_BITS-1:0]  dcr_req_data;
    wire                          dcr_rsp_valid;
    wire [`VX_DCR_DATA_BITS-1:0]  dcr_rsp_data;

    vortex_axil_shim #(
        .DCR_ADDR_BITS (`VX_DCR_ADDR_BITS),
        .DCR_DATA_BITS (`VX_DCR_DATA_BITS)
    ) shim (
        .clk            (clk),
        .resetn         (resetn),
        .s_axi_awvalid  (s_axi_ctrl_awvalid),
        .s_axi_awready  (s_axi_ctrl_awready),
        .s_axi_awaddr   (s_axi_ctrl_awaddr),
        .s_axi_wvalid   (s_axi_ctrl_wvalid),
        .s_axi_wready   (s_axi_ctrl_wready),
        .s_axi_wdata    (s_axi_ctrl_wdata),
        .s_axi_wstrb    (s_axi_ctrl_wstrb),
        .s_axi_bvalid   (s_axi_ctrl_bvalid),
        .s_axi_bready   (s_axi_ctrl_bready),
        .s_axi_bresp    (s_axi_ctrl_bresp),
        .s_axi_arvalid  (s_axi_ctrl_arvalid),
        .s_axi_arready  (s_axi_ctrl_arready),
        .s_axi_araddr   (s_axi_ctrl_araddr),
        .s_axi_rvalid   (s_axi_ctrl_rvalid),
        .s_axi_rready   (s_axi_ctrl_rready),
        .s_axi_rdata    (s_axi_ctrl_rdata),
        .s_axi_rresp    (s_axi_ctrl_rresp),
        .vx_reset       (shim_vx_reset),
        .vx_start       (shim_vx_start),
        .vx_busy        (vx_busy),
        .dcr_req_valid  (dcr_req_valid),
        .dcr_req_rw     (dcr_req_rw),
        .dcr_req_addr   (dcr_req_addr),
        .dcr_req_data   (dcr_req_data),
        .dcr_rsp_valid  (dcr_rsp_valid),
        .dcr_rsp_data   (dcr_rsp_data)
    );

    wire vx_reset = ~resetn || shim_vx_reset;

    wire                                m_axi_awvalid_a [1];
    wire                                m_axi_awready_a [1];
    wire [C_M_AXI_MEM_ADDR_WIDTH-1:0]   m_axi_awaddr_a [1];
    wire [C_M_AXI_MEM_ID_WIDTH-1:0]     m_axi_awid_a [1];
    wire [7:0]                          m_axi_awlen_a [1];
    wire [2:0]                          m_axi_awsize_a [1];
    wire [1:0]                          m_axi_awburst_a [1];
    wire [1:0]                          m_axi_awlock_a [1];
    wire [3:0]                          m_axi_awcache_a [1];
    wire [2:0]                          m_axi_awprot_a [1];
    wire [3:0]                          m_axi_awqos_a [1];
    wire [3:0]                          m_axi_awregion_a [1];
    wire                                m_axi_wvalid_a [1];
    wire                                m_axi_wready_a [1];
    wire [C_M_AXI_MEM_DATA_WIDTH-1:0]   m_axi_wdata_a [1];
    wire [C_M_AXI_MEM_DATA_WIDTH/8-1:0] m_axi_wstrb_a [1];
    wire                                m_axi_wlast_a [1];
    wire                                m_axi_bvalid_a [1];
    wire                                m_axi_bready_a [1];
    wire [C_M_AXI_MEM_ID_WIDTH-1:0]     m_axi_bid_a [1];
    wire [1:0]                          m_axi_bresp_a [1];
    wire                                m_axi_arvalid_a [1];
    wire                                m_axi_arready_a [1];
    wire [C_M_AXI_MEM_ADDR_WIDTH-1:0]   m_axi_araddr_a [1];
    wire [C_M_AXI_MEM_ID_WIDTH-1:0]     m_axi_arid_a [1];
    wire [7:0]                          m_axi_arlen_a [1];
    wire [2:0]                          m_axi_arsize_a [1];
    wire [1:0]                          m_axi_arburst_a [1];
    wire [1:0]                          m_axi_arlock_a [1];
    wire [3:0]                          m_axi_arcache_a [1];
    wire [2:0]                          m_axi_arprot_a [1];
    wire [3:0]                          m_axi_arqos_a [1];
    wire [3:0]                          m_axi_arregion_a [1];
    wire                                m_axi_rvalid_a [1];
    wire                                m_axi_rready_a [1];
    wire [C_M_AXI_MEM_DATA_WIDTH-1:0]   m_axi_rdata_a [1];
    wire                                m_axi_rlast_a [1];
    wire [C_M_AXI_MEM_ID_WIDTH-1:0]     m_axi_rid_a [1];
    wire [1:0]                          m_axi_rresp_a [1];

    assign m_axi_mem_awvalid   = m_axi_awvalid_a[0];
    assign m_axi_awready_a[0]  = m_axi_mem_awready;
    assign m_axi_mem_awaddr    = m_axi_awaddr_a[0];
    assign m_axi_mem_awid      = m_axi_awid_a[0];
    assign m_axi_mem_awlen     = m_axi_awlen_a[0];
    assign m_axi_mem_awsize    = m_axi_awsize_a[0];
    assign m_axi_mem_awburst   = m_axi_awburst_a[0];
    assign m_axi_mem_awlock    = m_axi_awlock_a[0];
    assign m_axi_mem_awcache   = m_axi_awcache_a[0];
    assign m_axi_mem_awprot    = m_axi_awprot_a[0];
    assign m_axi_mem_awqos     = m_axi_awqos_a[0];

    assign m_axi_mem_wvalid    = m_axi_wvalid_a[0];
    assign m_axi_wready_a[0]   = m_axi_mem_wready;
    assign m_axi_mem_wdata     = m_axi_wdata_a[0];
    assign m_axi_mem_wstrb     = m_axi_wstrb_a[0];
    assign m_axi_mem_wlast     = m_axi_wlast_a[0];

    assign m_axi_bvalid_a[0]   = m_axi_mem_bvalid;
    assign m_axi_mem_bready    = m_axi_bready_a[0];
    assign m_axi_bresp_a[0]    = m_axi_mem_bresp;
    assign m_axi_bid_a[0]      = m_axi_mem_bid;

    assign m_axi_mem_arvalid   = m_axi_arvalid_a[0];
    assign m_axi_arready_a[0]  = m_axi_mem_arready;
    assign m_axi_mem_araddr    = m_axi_araddr_a[0];
    assign m_axi_mem_arid      = m_axi_arid_a[0];
    assign m_axi_mem_arlen     = m_axi_arlen_a[0];
    assign m_axi_mem_arsize    = m_axi_arsize_a[0];
    assign m_axi_mem_arburst   = m_axi_arburst_a[0];
    assign m_axi_mem_arlock    = m_axi_arlock_a[0];
    assign m_axi_mem_arcache   = m_axi_arcache_a[0];
    assign m_axi_mem_arprot    = m_axi_arprot_a[0];
    assign m_axi_mem_arqos     = m_axi_arqos_a[0];

    assign m_axi_rvalid_a[0]   = m_axi_mem_rvalid;
    assign m_axi_mem_rready    = m_axi_rready_a[0];
    assign m_axi_rdata_a[0]    = m_axi_mem_rdata;
    assign m_axi_rlast_a[0]    = m_axi_mem_rlast;
    assign m_axi_rid_a[0]      = m_axi_mem_rid;
    assign m_axi_rresp_a[0]    = m_axi_mem_rresp;

    `UNUSED_VAR (m_axi_awregion_a[0])
    `UNUSED_VAR (m_axi_arregion_a[0])

    Vortex_axi #(
        .AXI_DATA_WIDTH (C_M_AXI_MEM_DATA_WIDTH),
        .AXI_ADDR_WIDTH (C_M_AXI_MEM_ADDR_WIDTH),
        .AXI_TID_WIDTH  (C_M_AXI_MEM_ID_WIDTH)
    ) vortex (
        .clk            (clk),
        .reset          (vx_reset),
        .start          (shim_vx_start),

        .m_axi_awvalid  (m_axi_awvalid_a),
        .m_axi_awready  (m_axi_awready_a),
        .m_axi_awaddr   (m_axi_awaddr_a),
        .m_axi_awid     (m_axi_awid_a),
        .m_axi_awlen    (m_axi_awlen_a),
        .m_axi_awsize   (m_axi_awsize_a),
        .m_axi_awburst  (m_axi_awburst_a),
        .m_axi_awlock   (m_axi_awlock_a),
        .m_axi_awcache  (m_axi_awcache_a),
        .m_axi_awprot   (m_axi_awprot_a),
        .m_axi_awqos    (m_axi_awqos_a),
        .m_axi_awregion (m_axi_awregion_a),

        .m_axi_wvalid   (m_axi_wvalid_a),
        .m_axi_wready   (m_axi_wready_a),
        .m_axi_wdata    (m_axi_wdata_a),
        .m_axi_wstrb    (m_axi_wstrb_a),
        .m_axi_wlast    (m_axi_wlast_a),

        .m_axi_bvalid   (m_axi_bvalid_a),
        .m_axi_bready   (m_axi_bready_a),
        .m_axi_bid      (m_axi_bid_a),
        .m_axi_bresp    (m_axi_bresp_a),

        .m_axi_arvalid  (m_axi_arvalid_a),
        .m_axi_arready  (m_axi_arready_a),
        .m_axi_araddr   (m_axi_araddr_a),
        .m_axi_arid     (m_axi_arid_a),
        .m_axi_arlen    (m_axi_arlen_a),
        .m_axi_arsize   (m_axi_arsize_a),
        .m_axi_arburst  (m_axi_arburst_a),
        .m_axi_arlock   (m_axi_arlock_a),
        .m_axi_arcache  (m_axi_arcache_a),
        .m_axi_arprot   (m_axi_arprot_a),
        .m_axi_arqos    (m_axi_arqos_a),
        .m_axi_arregion (m_axi_arregion_a),

        .m_axi_rvalid   (m_axi_rvalid_a),
        .m_axi_rready   (m_axi_rready_a),
        .m_axi_rdata    (m_axi_rdata_a),
        .m_axi_rid      (m_axi_rid_a),
        .m_axi_rresp    (m_axi_rresp_a),
        .m_axi_rlast    (m_axi_rlast_a),

        .dcr_req_valid  (dcr_req_valid),
        .dcr_req_rw     (dcr_req_rw),
        .dcr_req_addr   (dcr_req_addr),
        .dcr_req_data   (dcr_req_data),
        .dcr_rsp_valid  (dcr_rsp_valid),
        .dcr_rsp_data   (dcr_rsp_data),

        .busy           (vx_busy)
    );

endmodule
