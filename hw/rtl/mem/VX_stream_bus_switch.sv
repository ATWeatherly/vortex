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

// 1 -> N tile-stream bus switch: requests are steered to output `bus_sel`
// (the TSU worker derives it from the address bits that select the HBM
// pseudo-channel); beat responses from the N masters merge back round-robin.
module VX_stream_bus_switch import VX_gpu_pkg::*; #(
    parameter NUM_OUTPUTS = 1,
    parameter BURST_SIZE  = 256,
    parameter BEAT_SIZE   = 64,
    parameter TAG_WIDTH   = UUID_WIDTH + 1,
    parameter ADDR_WIDTH  = `VX_CFG_TSU_ADDR_WIDTH - `CLOG2(BURST_SIZE),   // = VX_stream_bus_if default
    parameter REQ_OUT_BUF = 0,
    parameter RSP_OUT_BUF = 0,
    parameter SEL_W       = `UP(`CLOG2(NUM_OUTPUTS))
) (
    input wire                  clk,
    input wire                  reset,
    input wire [SEL_W-1:0]      bus_sel,
    VX_stream_bus_if.slave      bus_in_if,
    VX_stream_bus_if.master     bus_out_if [NUM_OUTPUTS]
);
    localparam NUM_BEATS  = BURST_SIZE / BEAT_SIZE;
    localparam BEAT_WIDTH = `UP(`CLOG2(NUM_BEATS));
    localparam REQ_DATAW  = ADDR_WIDTH + NUM_BEATS + TAG_WIDTH;
    localparam RSP_DATAW  = BEAT_SIZE * 8 + BEAT_WIDTH + TAG_WIDTH;

    // ---- requests: 1 -> N by bus_sel ----
    wire [0:0]                 req_valid_in;
    wire [0:0][REQ_DATAW-1:0]  req_data_in;
    wire [0:0]                 req_ready_in;
    wire [0:0][SEL_W-1:0]      req_sel_in;

    wire [NUM_OUTPUTS-1:0]                req_valid_out;
    wire [NUM_OUTPUTS-1:0][REQ_DATAW-1:0] req_data_out;
    wire [NUM_OUTPUTS-1:0]                req_ready_out;

    assign req_valid_in[0] = bus_in_if.req_valid;
    assign req_data_in[0]  = {bus_in_if.req_data.addr, bus_in_if.req_data.beaten, bus_in_if.req_data.tag};
    assign req_sel_in[0]   = bus_sel;
    assign bus_in_if.req_ready = req_ready_in[0];

    VX_stream_xbar #(
        .NUM_INPUTS  (1),
        .NUM_OUTPUTS (NUM_OUTPUTS),
        .DATAW       (REQ_DATAW),
        .OUT_BUF     (REQ_OUT_BUF)
    ) req_xbar (
        .clk       (clk),
        .reset     (reset),
        .sel_in    (req_sel_in),
        .valid_in  (req_valid_in),
        .data_in   (req_data_in),
        .ready_in  (req_ready_in),
        .valid_out (req_valid_out),
        .data_out  (req_data_out),
        .ready_out (req_ready_out),
        `UNUSED_PIN (sel_out),
        `UNUSED_PIN (collisions)
    );

    for (genvar i = 0; i < NUM_OUTPUTS; ++i) begin : g_req_out
        assign bus_out_if[i].req_valid = req_valid_out[i];
        assign {bus_out_if[i].req_data.addr, bus_out_if[i].req_data.beaten, bus_out_if[i].req_data.tag} = req_data_out[i];
        assign req_ready_out[i] = bus_out_if[i].req_ready;
    end

    // ---- responses: N -> 1 round-robin ----
    wire [NUM_OUTPUTS-1:0]                rsp_valid_in;
    wire [NUM_OUTPUTS-1:0][RSP_DATAW-1:0] rsp_data_in;
    wire [NUM_OUTPUTS-1:0]                rsp_ready_in;

    wire [0:0]                 rsp_valid_out;
    wire [0:0][RSP_DATAW-1:0]  rsp_data_out;
    wire [0:0]                 rsp_ready_out;

    for (genvar i = 0; i < NUM_OUTPUTS; ++i) begin : g_rsp_in
        assign rsp_valid_in[i] = bus_out_if[i].rsp_valid;
        assign rsp_data_in[i]  = {bus_out_if[i].rsp_data.data, bus_out_if[i].rsp_data.beat, bus_out_if[i].rsp_data.tag};
        assign bus_out_if[i].rsp_ready = rsp_ready_in[i];
    end

    VX_stream_arb #(
        .NUM_INPUTS  (NUM_OUTPUTS),
        .NUM_OUTPUTS (1),
        .DATAW       (RSP_DATAW),
        .ARBITER     ("R"),
        .OUT_BUF     (RSP_OUT_BUF)
    ) rsp_arb (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (rsp_valid_in),
        .data_in   (rsp_data_in),
        .ready_in  (rsp_ready_in),
        .valid_out (rsp_valid_out),
        .data_out  (rsp_data_out),
        .ready_out (rsp_ready_out),
        `UNUSED_PIN (sel_out)
    );

    assign bus_in_if.rsp_valid = rsp_valid_out[0];
    assign {bus_in_if.rsp_data.data, bus_in_if.rsp_data.beat, bus_in_if.rsp_data.tag} = rsp_data_out[0];
    assign rsp_ready_out[0] = bus_in_if.rsp_ready;

endmodule
