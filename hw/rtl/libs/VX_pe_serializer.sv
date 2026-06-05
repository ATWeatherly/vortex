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

`TRACING_OFF
module VX_pe_serializer #(
    parameter NUM_LANES      = 1,
    parameter NUM_PES        = 1,
    parameter LATENCY        = 1,
    parameter DATA_IN_WIDTH  = 1,
    parameter DATA_OUT_WIDTH = 1,
    parameter TAG_WIDTH      = 0,
    parameter PE_REG         = 0,
    parameter OUT_BUF        = 0,
    // FREE_RUN=1 (requires NUM_LANES==NUM_PES) selects the free-running PE scheme:
    // the PEs are NEVER clock-gated (pe_enable held high); backpressure is handled
    // by credit-gating the INPUT and buffering results in an output FIFO. This
    // removes the data-dependent shared clock-enable net (and the enable-vs-valid
    // skew it causes across the PE array) that the legacy path relies on.
    parameter FREE_RUN       = 0
) (
    input wire                          clk,
    input wire                          reset,

    // input
    input wire                          valid_in,
    input wire [NUM_LANES-1:0][DATA_IN_WIDTH-1:0] data_in,
    input wire [TAG_WIDTH-1:0]          tag_in,
    output wire                         ready_in,

    // PE
    output wire                         pe_enable,
    output wire [NUM_PES-1:0][DATA_IN_WIDTH-1:0] pe_data_out,
    input wire [NUM_PES-1:0][DATA_OUT_WIDTH-1:0] pe_data_in,

    // output
    output wire                         valid_out,
    output wire [NUM_LANES-1:0][DATA_OUT_WIDTH-1:0] data_out,
    output wire [TAG_WIDTH-1:0]         tag_out,
    input wire                          ready_out
);
    localparam FREE_RUN_EFF = (FREE_RUN != 0) && (NUM_LANES == NUM_PES);

    if (FREE_RUN_EFF) begin : g_freerun

        // ----------------------------------------------------------------
        // Free-running scheme (NUM_LANES == NUM_PES).
        //
        //  - PEs clock every cycle (pe_enable == 1): no data-dependent enable
        //    net fanning across the array, so nothing for the far lane to see
        //    skewed vs. the valid tracker.
        //  - valid/tag tracker is depth=LATENCY and ALWAYS enabled, so it stays
        //    in lockstep with the (also always-clocking) PE pipelines by
        //    construction.
        //  - backpressure is handled by reserving an output-FIFO slot per
        //    accepted op (a credit) and draining results into that FIFO, never
        //    by freezing the datapath.
        // ----------------------------------------------------------------

        // Output FIFO must hold every in-flight + buffered result. Size it to the
        // pipeline depth plus slack so a continuously-ready consumer never starves
        // the credit pool (full 1-op/cycle throughput).
        localparam CREDITS = LATENCY + 4;
        localparam CW      = `CLOG2(CREDITS + 1);

        `UNUSED_PARAM (PE_REG)
        `UNUSED_PARAM (OUT_BUF)

        wire fire = valid_in & ready_in;
        wire pop;

        // PEs run free; operands are presented combinationally every cycle. Cycles
        // that are not a real "fire" produce results that the tracker marks invalid.
        assign pe_enable   = 1'b1;
        assign pe_data_out = data_in;

        // valid + tag tracker: depth = LATENCY, always enabled -> aligns with the
        // PE result that emerges LATENCY cycles after the operands were presented.
        wire                  res_valid;
        wire [TAG_WIDTH-1:0]  res_tag;

        VX_shift_register #(
            .DATAW  (1 + TAG_WIDTH),
            .DEPTH  (LATENCY),
            .RESETW (1)
        ) tracker (
            .clk      (clk),
            .reset    (reset),
            .enable   (1'b1),
            .data_in  ({fire,      tag_in}),
            .data_out ({res_valid, res_tag})
        );

        // credit counter: one output-FIFO slot reserved per accepted op, released
        // when the result is consumed downstream. Guarantees the FIFO never drops a
        // result and bounds the in-flight count.
        reg [CW-1:0] credits;
        always @(posedge clk) begin
            if (reset)
                credits <= CW'(CREDITS);
            else
                credits <= credits - CW'(fire) + CW'(pop);
        end
        assign ready_in = (credits != 0);

        // output FIFO: absorbs the fixed-latency results, presents an elastic
        // (valid/ready) interface downstream.
        wire fifo_empty;
        VX_fifo_queue #(
            .DATAW (NUM_LANES * DATA_OUT_WIDTH + TAG_WIDTH),
            .DEPTH (CREDITS),
            .OUT_REG (0)
        ) out_fifo (
            .clk       (clk),
            .reset     (reset),
            .push      (res_valid),
            .pop       (pop),
            .data_in   ({pe_data_in, res_tag}),
            .data_out  ({data_out,   tag_out}),
            .empty     (fifo_empty),
            `UNUSED_PIN (alm_empty),
            `UNUSED_PIN (full),
            `UNUSED_PIN (alm_full),
            `UNUSED_PIN (size)
        );

        assign valid_out = ~fifo_empty;
        assign pop       = valid_out & ready_out;

    end else begin : g_legacy

        // ----------------------------------------------------------------
        // Legacy scheme (unchanged): shared pe_enable gates the PE pipelines and
        // the centralized valid tracker. Retained for batched mode (NUM_PES <
        // NUM_LANES) and for FREE_RUN=0 callers (div/sqrt/cvt/ncp).
        // ----------------------------------------------------------------

        wire                    valid_out_u;
        wire [NUM_LANES-1:0][DATA_OUT_WIDTH-1:0] data_out_u;
        wire [TAG_WIDTH-1:0]    tag_out_u;
        wire                    ready_out_u;

        wire [NUM_PES-1:0][DATA_IN_WIDTH-1:0] pe_data_out_w;
        wire pe_valid_in;
        wire [TAG_WIDTH-1:0] pe_tag_in;
        wire enable;

        VX_shift_register #(
            .DATAW  (1 + TAG_WIDTH),
            .DEPTH  (PE_REG + LATENCY),
            .RESETW (1)
        ) shift_reg (
            .clk      (clk),
            .reset    (reset),
            .enable   (enable),
            .data_in  ({valid_in,    tag_in}),
            .data_out ({pe_valid_in, pe_tag_in})
        );

        VX_pipe_register #(
            .DATAW  (NUM_PES * DATA_IN_WIDTH),
            .DEPTH  (PE_REG)
        ) pe_data_reg (
            .clk      (clk),
            .reset    (reset),
            .enable   (enable),
            .data_in  (pe_data_out_w),
            .data_out (pe_data_out)
        );

        assign pe_enable = enable;

        if (NUM_LANES != NUM_PES) begin : g_serialize

            localparam BATCH_SIZE = NUM_LANES / NUM_PES;
            localparam BATCH_SIZEW = `LOG2UP(BATCH_SIZE);

            reg [BATCH_SIZEW-1:0] batch_in_idx, batch_out_idx;
            reg batch_in_done, batch_out_done;

            for (genvar i = 0; i < NUM_PES; ++i) begin : g_pe_data_out_w
                assign pe_data_out_w[i] = data_in[batch_in_idx * NUM_PES + i];
            end

            always @(posedge clk) begin
                if (reset) begin
                    batch_in_idx   <= '0;
                    batch_out_idx  <= '0;
                    batch_in_done  <= 0;
                    batch_out_done <= 0;
                end else if (enable) begin
                    batch_in_idx   <= batch_in_idx + BATCH_SIZEW'(valid_in);
                    batch_out_idx  <= batch_out_idx + BATCH_SIZEW'(pe_valid_in);
                    batch_in_done  <= valid_in && (batch_in_idx == BATCH_SIZEW'(BATCH_SIZE-2));
                    batch_out_done <= pe_valid_in && (batch_out_idx == BATCH_SIZEW'(BATCH_SIZE-2));
                end
            end

            reg [BATCH_SIZE-1:0][(NUM_PES * DATA_OUT_WIDTH)-1:0] data_out_r, data_out_n;

            always @(*) begin
                data_out_n = data_out_r;
                if (pe_valid_in) begin
                    data_out_n[batch_out_idx] = pe_data_in;
                end
            end

            always @(posedge clk) begin
                data_out_r <= data_out_n;
            end

            assign enable      = ready_out_u || ~valid_out_u;
            assign ready_in    = enable && batch_in_done;

            assign valid_out_u = batch_out_done;
            assign data_out_u  = data_out_n;
            assign tag_out_u   = pe_tag_in;

        end else begin : g_passthru

            assign pe_data_out_w = data_in;

            assign enable      = ready_out_u || ~pe_valid_in;
            assign ready_in    = enable;

            assign valid_out_u = pe_valid_in;
            assign data_out_u  = pe_data_in;
            assign tag_out_u   = pe_tag_in;

        end

        VX_elastic_buffer #(
            .DATAW   (NUM_LANES * DATA_OUT_WIDTH + TAG_WIDTH),
            .SIZE    (`TO_OUT_BUF_SIZE(OUT_BUF)),
            .OUT_REG (`TO_OUT_BUF_REG(OUT_BUF))
        ) out_buf (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (valid_out_u),
            .ready_in  (ready_out_u),
            .data_in   ({data_out_u, tag_out_u}),
            .data_out  ({data_out, tag_out}),
            .valid_out (valid_out),
            .ready_out (ready_out)
        );

    end

endmodule
`TRACING_ON
