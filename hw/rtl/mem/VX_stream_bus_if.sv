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

// Tile stream bus (TSU): a read-only burst request/response bus.
//
// A request names one BURST_SIZE-aligned granule of global memory and the
// contiguous span of BEAT_SIZE beats inside it that are wanted (`beaten`).
// The AXI bridge turns it into one INCR burst from the first to the last set
// beat, so a partial row costs a shorter burst, never a wider one. Responses
// come back one beat at a time, tagged with the request tag and the beat index
// (beats of one request arrive in order; different requests may interleave),
// so no 256 B reassembly buffer exists anywhere on the path.
interface VX_stream_bus_if import VX_gpu_pkg::*; #(
    parameter BURST_SIZE = 256,
    parameter BEAT_SIZE  = 64,
    parameter TAG_WIDTH  = UUID_WIDTH + 1,
    parameter ADDR_WIDTH = `VX_CFG_MEM_ADDR_WIDTH - `CLOG2(BURST_SIZE),
    parameter NUM_BEATS  = BURST_SIZE / BEAT_SIZE,
    parameter BEAT_WIDTH = `UP(`CLOG2(NUM_BEATS))
) ();

    typedef struct packed {
        logic [ADDR_WIDTH-1:0]  addr;    // granule address (byte address >> log2(BURST_SIZE))
        logic [NUM_BEATS-1:0]   beaten;  // requested beats, a contiguous span
        logic [TAG_WIDTH-1:0]   tag;
    } req_data_t;

    typedef struct packed {
        logic [BEAT_SIZE*8-1:0] data;
        logic [BEAT_WIDTH-1:0]  beat;    // index of this beat inside the granule
        logic [TAG_WIDTH-1:0]   tag;
    } rsp_data_t;

    logic       req_valid;
    req_data_t  req_data;
    logic       req_ready;

    logic       rsp_valid;
    rsp_data_t  rsp_data;
    logic       rsp_ready;

    modport master (
        output req_valid,
        output req_data,
        input  req_ready,
        input  rsp_valid,
        input  rsp_data,
        output rsp_ready
    );

    modport slave (
        input  req_valid,
        input  req_data,
        output req_ready,
        output rsp_valid,
        output rsp_data,
        input  rsp_ready
    );

endinterface
