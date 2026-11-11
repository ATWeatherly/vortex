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

`include "VX_platform.vh"

// ============================================================================
// VX_stream_to_axi — one tile-stream port to one read-only AXI4 master.
//
// A stream request names a BURST_SIZE granule and the contiguous span of
// DATA_WIDTH beats wanted inside it; it becomes ONE INCR burst from the first
// to the last wanted beat (arlen = span - 1). Read IDs come from a
// TAG_BUFFER_SIZE-entry index buffer that also holds the stream tag and the
// first beat index, so at most TAG_BUFFER_SIZE bursts are outstanding per
// master (the U50 platform accepts 16 per port) and responses may return in
// any order between IDs. Every R beat is forwarded upstream as its own
// response carrying the stream tag and the beat index; no reassembly.
// The write channels are not present. All channels are register-sliced.
// ============================================================================

`TRACING_OFF
module VX_stream_to_axi #(
    parameter DATA_WIDTH      = 512,
    parameter BURST_SIZE      = 256,
    parameter ADDR_WIDTH_IN   = 25,   // granule-addressable
    parameter ADDR_WIDTH_OUT  = 33,   // byte-addressable
    parameter TAG_WIDTH_IN    = 8,
    parameter TAG_WIDTH_OUT   = 8,
    parameter TAG_BUFFER_SIZE = 16,
    parameter DATA_SIZE       = DATA_WIDTH / 8,
    parameter NUM_BEATS       = BURST_SIZE / DATA_SIZE,
    parameter BEAT_W          = `UP(`CLOG2(NUM_BEATS))
) (
    input  wire                      clk,
    input  wire                      reset,

    // stream request
    input  wire                      str_req_valid,
    input  wire [ADDR_WIDTH_IN-1:0]  str_req_addr,
    input  wire [NUM_BEATS-1:0]      str_req_beaten,
    input  wire [TAG_WIDTH_IN-1:0]   str_req_tag,
    output wire                      str_req_ready,

    // stream response (one beat each)
    output wire                      str_rsp_valid,
    output wire [DATA_WIDTH-1:0]     str_rsp_data,
    output wire [BEAT_W-1:0]         str_rsp_beat,
    output wire [TAG_WIDTH_IN-1:0]   str_rsp_tag,
    input  wire                      str_rsp_ready,

    // AXI read address channel
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    output wire [ADDR_WIDTH_OUT-1:0] m_axi_araddr,
    output wire [TAG_WIDTH_OUT-1:0]  m_axi_arid,
    output wire [7:0]                m_axi_arlen,
    output wire [2:0]                m_axi_arsize,
    output wire [1:0]                m_axi_arburst,
    output wire [1:0]                m_axi_arlock,
    output wire [3:0]                m_axi_arcache,
    output wire [2:0]                m_axi_arprot,
    output wire [3:0]                m_axi_arqos,
    output wire [3:0]                m_axi_arregion,

    // AXI read data channel
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [DATA_WIDTH-1:0]     m_axi_rdata,
    input  wire                      m_axi_rlast,
    input  wire [TAG_WIDTH_OUT-1:0]  m_axi_rid,
    input  wire [1:0]                m_axi_rresp
);
    localparam LOG2_DATA_SIZE  = `CLOG2(DATA_SIZE);
    localparam LOG2_BURST_SIZE = `CLOG2(BURST_SIZE);
    localparam TBUF_ADDRW      = `CLOG2(TAG_BUFFER_SIZE);
    localparam TBUF_DATAW      = TAG_WIDTH_IN + BEAT_W;

    `STATIC_ASSERT((TAG_WIDTH_OUT >= TBUF_ADDRW), ("AXI id too narrow for the tag buffer: current=%0d, needed=%0d", TAG_WIDTH_OUT, TBUF_ADDRW))
    `STATIC_ASSERT((ADDR_WIDTH_OUT >= ADDR_WIDTH_IN + LOG2_BURST_SIZE), ("invalid address widths"))
    `STATIC_ASSERT(`IS_POW2(NUM_BEATS), ("NUM_BEATS must be a power of 2"))

    // ---- span of the incoming request ----
    reg [BEAT_W-1:0] req_first_beat, req_last_beat;
    always @(*) begin
        req_first_beat = '0;
        req_last_beat  = '0;
        for (int b = NUM_BEATS - 1; b >= 0; --b) begin
            if (str_req_beaten[b]) req_first_beat = BEAT_W'(b);
        end
        for (int b = 0; b < NUM_BEATS; ++b) begin
            if (str_req_beaten[b]) req_last_beat = BEAT_W'(b);
        end
    end
    wire [7:0] req_arlen = 8'(req_last_beat - req_first_beat);

    // ---- tag buffer: stream tag + first beat per outstanding burst ----
    wire [TBUF_ADDRW-1:0] tbuf_waddr;
    wire                  tbuf_full;
    wire [TBUF_ADDRW-1:0] tbuf_raddr;
    wire [TBUF_DATAW-1:0] tbuf_rdata;
    wire                  tbuf_release;

    wire ar_fire_in;
    VX_index_buffer #(
        .DATAW (TBUF_DATAW),
        .SIZE  (TAG_BUFFER_SIZE)
    ) tag_buf (
        .clk        (clk),
        .reset      (reset),
        .acquire_en (ar_fire_in),
        .write_addr (tbuf_waddr),
        .write_data ({str_req_tag, req_first_beat}),
        .read_addr  (tbuf_raddr),
        .read_data  (tbuf_rdata),
        .release_en (tbuf_release),
        .full       (tbuf_full),
        `UNUSED_PIN (empty)
    );

    // ---- AR channel (register slice) ----
    localparam AR_DATAW = ADDR_WIDTH_OUT + TBUF_ADDRW + 8;
    wire [ADDR_WIDTH_OUT-1:0] req_araddr = (ADDR_WIDTH_OUT'(str_req_addr) << LOG2_BURST_SIZE)
                                         | (ADDR_WIDTH_OUT'(req_first_beat) << LOG2_DATA_SIZE);
    wire ar_ready_in;
    assign ar_fire_in    = str_req_valid && ~tbuf_full && ar_ready_in;
    assign str_req_ready = ~tbuf_full && ar_ready_in;

    wire [AR_DATAW-1:0] ar_data_out;
    VX_elastic_buffer #(
        .DATAW   (AR_DATAW),
        .SIZE    (2),
        .OUT_REG (1)
    ) ar_buf (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (str_req_valid && ~tbuf_full),
        .ready_in  (ar_ready_in),
        .data_in   ({req_araddr, tbuf_waddr, req_arlen}),
        .valid_out (m_axi_arvalid),
        .ready_out (m_axi_arready),
        .data_out  (ar_data_out)
    );
    wire [TBUF_ADDRW-1:0] ar_id_out;
    assign {m_axi_araddr, ar_id_out, m_axi_arlen} = ar_data_out;
    assign m_axi_arid    = TAG_WIDTH_OUT'(ar_id_out);
    assign m_axi_arsize  = 3'(LOG2_DATA_SIZE);
    assign m_axi_arburst = 2'b01;   // INCR
    assign m_axi_arlock  = 2'b00;
    assign m_axi_arcache = 4'b0000;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;
    assign m_axi_arregion= 4'b0000;

    // ---- R channel: skid, then per-ID beat counting ----
    localparam R_DATAW = DATA_WIDTH + TAG_WIDTH_OUT + 1;
    wire                  r_valid;
    wire                  r_ready;
    wire [DATA_WIDTH-1:0] r_data;
    wire [TAG_WIDTH_OUT-1:0] r_id;
    wire                  r_last;
    VX_elastic_buffer #(
        .DATAW   (R_DATAW),
        .SIZE    (2),
        .OUT_REG (1)
    ) r_buf (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (m_axi_rvalid),
        .ready_in  (m_axi_rready),
        .data_in   ({m_axi_rdata, m_axi_rid, m_axi_rlast}),
        .valid_out (r_valid),
        .ready_out (r_ready),
        .data_out  ({r_data, r_id, r_last})
    );
    `RUNTIME_ASSERT(~(m_axi_rvalid && m_axi_rresp != 0), ("*** AXI stream read response error"))

    // Beat index within the granule = first beat (from the tag buffer) + the
    // number of beats of this ID already delivered.
    reg [TAG_BUFFER_SIZE-1:0][BEAT_W-1:0] beat_cnt_r;
    wire [TBUF_ADDRW-1:0] r_slot = TBUF_ADDRW'(r_id);
    assign tbuf_raddr = r_slot;
    wire [TAG_WIDTH_IN-1:0] r_tag;
    wire [BEAT_W-1:0]       r_first;
    assign {r_tag, r_first} = tbuf_rdata;

    wire r_fire = r_valid && r_ready;
    assign str_rsp_valid = r_valid;
    assign str_rsp_data  = r_data;
    assign str_rsp_tag   = r_tag;
    assign str_rsp_beat  = r_first + beat_cnt_r[r_slot];
    assign r_ready       = str_rsp_ready;
    assign tbuf_release  = r_fire && r_last;

    always @(posedge clk) begin
        if (reset) begin
            beat_cnt_r <= '0;
        end else begin
            if (ar_fire_in) begin
                beat_cnt_r[tbuf_waddr] <= '0;
            end
            if (r_fire && ~r_last) begin
                beat_cnt_r[r_slot] <= beat_cnt_r[r_slot] + BEAT_W'(1);
            end
        end
    end

    `UNUSED_VAR (m_axi_rid)

endmodule
`TRACING_ON
