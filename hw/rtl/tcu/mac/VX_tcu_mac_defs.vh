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

// Minimal, self-contained macro set for the MAC block (VX_tcu_mac_*), so the block synthesises and unit-tests
// without the core's include tree. Every macro is guarded: inside the core VX_define.vh's definitions win.

`ifndef VX_TCU_MAC_DEFS_VH
`define VX_TCU_MAC_DEFS_VH

`ifndef CLOG2
`define CLOG2(x) $clog2(x)
`endif

`ifdef SIMULATION
  `ifndef STATIC_ASSERT
  `define STATIC_ASSERT(cond, msg) \
      /* verilator lint_off GENUNNAMED */ \
      initial if (!(cond)) begin \
          $error msg; \
      end \
      /* verilator lint_on GENUNNAMED */
  `endif
  `ifndef RUNTIME_ASSERT
  `define RUNTIME_ASSERT(cond, msg) \
      always @(posedge clk) begin \
          if (!reset) begin \
              assert(cond) else $error msg; \
          end \
      end
  `endif
`else
  `ifndef STATIC_ASSERT
  `define STATIC_ASSERT(cond, msg)
  `endif
  `ifndef RUNTIME_ASSERT
  `define RUNTIME_ASSERT(cond, msg)
  `endif
`endif

`ifndef USE_FAST_BRAM
  `ifdef VIVADO
  `define USE_FAST_BRAM (* ram_style = "distributed" *)
  `else
  `define USE_FAST_BRAM
  `endif
`endif

`endif // VX_TCU_MAC_DEFS_VH
