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

module VX_tcu_unit import VX_gpu_pkg::*, VX_tcu_pkg::*; #(
    parameter `STRING INSTANCE_ID = ""
) (
    `SCOPE_IO_DECL

    input wire              clk,
    input wire              reset,

`ifdef PERF_ENABLE
    output tcu_perf_t       tcu_perf,
`endif

`ifdef TCU_LMEM_PORT_ENABLE
    // Bank-parallel LMEM port (WGMMA tile buffers and/or the sequencer's tacc.st)
    VX_mem_bus_if.master     tcu_lmem_if,
`endif

`ifdef TCU_MEMCLIENT_ENABLE
    // TCU_LD / sequencer slot-load memory client connection to VX_lsu_scheduler at VX_core.
    VX_lsu_sched_if.master  tcu_mem_if,
`endif

`ifdef VX_CFG_TCU_SEQ_ENABLE
    // tfence: core-wide store drain
    input wire              store_drained,
    // twait / tfence release of the wstalled warp
    VX_sched_unlock_if.master sched_unlock_if,
`ifdef VX_CFG_TSU_ENABLE
    // phase-4 W-slot landing (TSU)
    input  wire [VX_dxa_pkg::TSU_WL_LANES-1:0] wl_valid,
    output wire [VX_dxa_pkg::TSU_WL_LANES-1:0] wl_ready,
    input  VX_dxa_pkg::dxa_wland_beat_t [VX_dxa_pkg::TSU_WL_LANES-1:0] wl_beat,
    input  wire                         wl_done_valid,
    input  VX_dxa_pkg::dxa_wland_done_t wl_done,
`endif
`endif

    // Inputs
    VX_dispatch_if.slave    dispatch_if [`VX_CFG_ISSUE_WIDTH],

    // Outputs
    VX_commit_if.master     commit_if [`VX_CFG_ISSUE_WIDTH]
);
    localparam BLOCK_SIZE = `VX_CFG_NUM_TCU_BLOCKS;
    localparam NUM_LANES  = `VX_CFG_NUM_TCU_LANES;

    `STATIC_ASSERT (BLOCK_SIZE == `VX_CFG_ISSUE_WIDTH, ("must be full issue execution"));
    `STATIC_ASSERT (NUM_LANES == `VX_CFG_NUM_THREADS, ("must be full warp execution"));
    `SCOPE_IO_SWITCH (BLOCK_SIZE);

    VX_execute_if #(
        .data_t (tcu_execute_t)
    ) per_block_execute_if[BLOCK_SIZE]();

    VX_lane_dispatch #(
        .BLOCK_SIZE (BLOCK_SIZE),
        .NUM_LANES  (NUM_LANES),
        .OUT_BUF    (3)
    ) lane_dispatch (
        .clk        (clk),
        .reset      (reset),
        .dispatch_if(dispatch_if),
        .execute_if (per_block_execute_if)
    );

    VX_result_if #(
        .data_t (tcu_result_t)
    ) per_block_result_if[BLOCK_SIZE]();

`ifdef VX_CFG_TCU_SEQ_ONLY
    // -----------------------------------------------------------------------
    // Sequencer-only tensor unit (VX_CFG_TCU_SEQ_ONLY): every TCU op is a sequencer command (the decoder
    // does not produce WMMA / WGMMA / TCU_LD), so the lane-dispatch slices feed VX_tcu_seq directly and its
    // commit packets are the only results. No VX_tcu_core FEDP blocks, tile buffers or LMEM arbiter.
    // -----------------------------------------------------------------------
    wire [BLOCK_SIZE-1:0] seq_cmd_valid, seq_cmd_ready;
    tcu_execute_t         seq_cmd_data [BLOCK_SIZE];
    wire [BLOCK_SIZE-1:0] seq_res_valid, seq_res_ready;
    tcu_result_t          seq_res_data [BLOCK_SIZE];

    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_seq_only
        assign seq_cmd_valid[bi] = per_block_execute_if[bi].valid;
        assign seq_cmd_data[bi]  = per_block_execute_if[bi].data;
        assign per_block_execute_if[bi].ready = seq_cmd_ready[bi];
        assign per_block_result_if[bi].valid = seq_res_valid[bi];
        assign per_block_result_if[bi].data  = seq_res_data[bi];
        assign seq_res_ready[bi] = per_block_result_if[bi].ready;
        `RUNTIME_ASSERT(~(per_block_execute_if[bi].valid && per_block_execute_if[bi].data.op_type != INST_TCU_SEQ),
            ("%t: *** %s: non-sequencer TCU op %0d on a VX_CFG_TCU_SEQ_ONLY build (block %0d)", $time, INSTANCE_ID,
             per_block_execute_if[bi].data.op_type, bi))
    end

    VX_tcu_seq #(
        .INSTANCE_ID (`SFORMATF(("%s-seq", INSTANCE_ID)))
    ) tcu_seq (
        .clk           (clk),
        .reset         (reset),
        .cmd_valid     (seq_cmd_valid),
        .cmd_data      (seq_cmd_data),
        .cmd_ready     (seq_cmd_ready),
        .res_valid     (seq_res_valid),
        .res_data      (seq_res_data),
        .res_ready     (seq_res_ready),
        .unlock_valid  (sched_unlock_if.valid),
        .unlock_wid    (sched_unlock_if.wid),
        .store_drained (store_drained),
    `ifdef VX_CFG_TSU_ENABLE
        .wl_valid      (wl_valid),
        .wl_ready      (wl_ready),
        .wl_beat       (wl_beat),
        .wl_done_valid (wl_done_valid),
        .wl_done       (wl_done),
    `endif
        .mem_if        (tcu_mem_if),
        .lmem_if       (tcu_lmem_if)
    );

    VX_lane_gather #(
        .BLOCK_SIZE (BLOCK_SIZE),
        .NUM_LANES  (NUM_LANES),
        .OUT_BUF    (3)
    ) lane_gather (
        .clk       (clk),
        .reset     (reset),
        .result_if (per_block_result_if),
        .commit_if (commit_if)
    );

`ifdef PERF_ENABLE
    // legacy-datapath counters read as zero (the sequencer has its own counter class)
    assign tcu_perf = '0;
`endif

`else // !VX_CFG_TCU_SEQ_ONLY

    // -----------------------------------------------------------------------
    // Split each per_block_execute_if between two consumers:
    //   - VX_tcu_agu: handles INST_TCU_LD (warp-level memory load).
    //   - VX_tcu_core: handles every MMA op_type.
    // The ready signal is muxed by op_type so only one consumer drives at a time.
    // -----------------------------------------------------------------------
    VX_execute_if #(
        .data_t (tcu_execute_t)
    ) core_execute_if[BLOCK_SIZE]();

    VX_result_if #(
        .data_t (tcu_result_t)
    ) core_result_if[BLOCK_SIZE]();

`ifdef TCU_META_ENABLE
    wire [BLOCK_SIZE-1:0]    agu_ld_valid;
    wire [BLOCK_SIZE-1:0]    agu_ld_ready;
    tcu_execute_t            agu_ld_data [BLOCK_SIZE];

    wire [BLOCK_SIZE-1:0]    agu_result_valid;
    tcu_result_t             agu_result_data [BLOCK_SIZE];
    wire [BLOCK_SIZE-1:0]    agu_result_ready;
`endif

`ifdef VX_CFG_TCU_SEQ_ENABLE
    // Sequencer commands (INST_TCU_SEQ) leave the WMMA/WGMMA path before the AGU/core split.
    VX_execute_if #(
        .data_t (tcu_execute_t)
    ) mma_execute_if[BLOCK_SIZE]();

    wire [BLOCK_SIZE-1:0] seq_cmd_valid, seq_cmd_ready;
    tcu_execute_t         seq_cmd_data [BLOCK_SIZE];
    wire [BLOCK_SIZE-1:0] seq_res_valid, seq_res_ready;
    tcu_result_t          seq_res_data [BLOCK_SIZE];

    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_seq_split
        wire is_seq = (per_block_execute_if[bi].data.op_type == INST_TCU_SEQ);
        assign seq_cmd_valid[bi] = per_block_execute_if[bi].valid && is_seq;
        assign seq_cmd_data[bi]  = per_block_execute_if[bi].data;
        assign mma_execute_if[bi].valid = per_block_execute_if[bi].valid && !is_seq;
        assign mma_execute_if[bi].data  = per_block_execute_if[bi].data;
        assign per_block_execute_if[bi].ready = is_seq ? seq_cmd_ready[bi] : mma_execute_if[bi].ready;
    end
    `define TCU_MMA_EXEC_IF mma_execute_if
`else
    `define TCU_MMA_EXEC_IF per_block_execute_if
`endif

    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_split
    `ifdef TCU_META_ENABLE
        wire is_tcu_ld = (`TCU_MMA_EXEC_IF[bi].data.op_type == INST_TCU_LD);

        // To AGU when TCU_LD
        assign agu_ld_valid[bi]    = `TCU_MMA_EXEC_IF[bi].valid && is_tcu_ld;
        assign agu_ld_data[bi]     = `TCU_MMA_EXEC_IF[bi].data;

        // To tcu_core when NOT TCU_LD
        assign core_execute_if[bi].valid = `TCU_MMA_EXEC_IF[bi].valid && !is_tcu_ld;
        assign core_execute_if[bi].data  = `TCU_MMA_EXEC_IF[bi].data;

        // Parent .ready: route to AGU on TCU_LD, otherwise to tcu_core
        assign `TCU_MMA_EXEC_IF[bi].ready = is_tcu_ld
            ? agu_ld_ready[bi]
            : core_execute_if[bi].ready;
    `else
        // No sparse: pass-through to tcu_core
        assign core_execute_if[bi].valid = `TCU_MMA_EXEC_IF[bi].valid;
        assign core_execute_if[bi].data  = `TCU_MMA_EXEC_IF[bi].data;
        assign `TCU_MMA_EXEC_IF[bi].ready = core_execute_if[bi].ready;
    `endif
    end
    `undef TCU_MMA_EXEC_IF

    // -----------------------------------------------------------------------
    // Result_if merge: AGU result and tcu_core result are mutually exclusive
    // in time per block; OR-mux with priority arbiter (AGU wins: TCU_LD is rare).
    // -----------------------------------------------------------------------
`ifdef TCU_META_ENABLE
    // AGU wins same-cycle conflicts; tcu_core stalls (ready=0) and retries next cycle.
    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_result_merge
        assign per_block_result_if[bi].valid = agu_result_valid[bi] || core_result_if[bi].valid;
        assign per_block_result_if[bi].data  = agu_result_valid[bi]
            ? agu_result_data[bi]
            : core_result_if[bi].data;
        assign agu_result_ready[bi]    = per_block_result_if[bi].ready;
        assign core_result_if[bi].ready = per_block_result_if[bi].ready && !agu_result_valid[bi];
    end
`elsif VX_CFG_TCU_SEQ_ENABLE
    // Sequencer commit packets win same-cycle conflicts (one per command; rare next to MMA uops).
    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_result_merge_seq
        assign per_block_result_if[bi].valid = seq_res_valid[bi] || core_result_if[bi].valid;
        assign per_block_result_if[bi].data  = seq_res_valid[bi] ? seq_res_data[bi] : core_result_if[bi].data;
        assign seq_res_ready[bi]        = per_block_result_if[bi].ready;
        assign core_result_if[bi].ready = per_block_result_if[bi].ready && !seq_res_valid[bi];
    end
`else
    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_result_passthru
        assign per_block_result_if[bi].valid = core_result_if[bi].valid;
        assign per_block_result_if[bi].data  = core_result_if[bi].data;
        assign core_result_if[bi].ready      = per_block_result_if[bi].ready;
    end
`endif

`ifdef VX_CFG_TCU_SEQ_ENABLE
    // -----------------------------------------------------------------------
    // TCU-side sequencer (tgemm ISA). Shares the TCU LMEM port with the WGMMA
    // tile buffers through a 2:1 arbiter (sequencer = input 1).
    // -----------------------------------------------------------------------
    VX_mem_bus_if #(
        .DATA_SIZE  (`VX_CFG_LMEM_NUM_BANKS * LSU_WORD_SIZE),
        .TAG_WIDTH  (TCU_LMEM_WG_TAG_W),
        .ATTR_WIDTH (LMEM_DMA_ATTR_W),
        .ADDR_WIDTH (`VX_CFG_LMEM_LOG_SIZE - `CLOG2(LSU_WORD_SIZE) - `CLOG2(`VX_CFG_LMEM_NUM_BANKS))
    ) tcu_lmem_in_if[2]();

    VX_mem_bus_if #(
        .DATA_SIZE  (`VX_CFG_LMEM_NUM_BANKS * LSU_WORD_SIZE),
        .TAG_WIDTH  (TCU_LMEM_TAG_W),
        .ATTR_WIDTH (LMEM_DMA_ATTR_W),
        .ADDR_WIDTH (`VX_CFG_LMEM_LOG_SIZE - `CLOG2(LSU_WORD_SIZE) - `CLOG2(`VX_CFG_LMEM_NUM_BANKS))
    ) tcu_lmem_arb_out_if[1]();

    VX_mem_bus_arb #(
        .NUM_INPUTS  (2),
        .NUM_OUTPUTS (1),
        .DATA_SIZE   (`VX_CFG_LMEM_NUM_BANKS * LSU_WORD_SIZE),
        .TAG_WIDTH   (TCU_LMEM_WG_TAG_W),
        .ATTR_WIDTH  (LMEM_DMA_ATTR_W),
        .ADDR_WIDTH  (`VX_CFG_LMEM_LOG_SIZE - `CLOG2(LSU_WORD_SIZE) - `CLOG2(`VX_CFG_LMEM_NUM_BANKS)),
        .ARBITER     ("R")
    ) tcu_lmem_arb (
        .clk        (clk),
        .reset      (reset),
        .bus_in_if  (tcu_lmem_in_if),
        .bus_out_if (tcu_lmem_arb_out_if)
    );

    `ASSIGN_VX_MEM_BUS_IF (tcu_lmem_if, tcu_lmem_arb_out_if[0]);

    VX_tcu_seq #(
        .INSTANCE_ID (`SFORMATF(("%s-seq", INSTANCE_ID)))
    ) tcu_seq (
        .clk           (clk),
        .reset         (reset),
        .cmd_valid     (seq_cmd_valid),
        .cmd_data      (seq_cmd_data),
        .cmd_ready     (seq_cmd_ready),
        .res_valid     (seq_res_valid),
        .res_data      (seq_res_data),
        .res_ready     (seq_res_ready),
        .unlock_valid  (sched_unlock_if.valid),
        .unlock_wid    (sched_unlock_if.wid),
        .store_drained (store_drained),
    `ifdef VX_CFG_TSU_ENABLE
        .wl_valid      (wl_valid),
        .wl_ready      (wl_ready),
        .wl_beat       (wl_beat),
        .wl_done_valid (wl_done_valid),
        .wl_done       (wl_done),
    `endif
        .mem_if        (tcu_mem_if),
        .lmem_if       (tcu_lmem_in_if[1])
    );
    `define TCU_WG_LMEM_IF tcu_lmem_in_if[0]
`else
    `define TCU_WG_LMEM_IF tcu_lmem_if
`endif

    // -----------------------------------------------------------------------
    // WGMMA feature (orchestrator): VX_tcu_tbuf + VX_tcu_lockstep + perf.
    // -----------------------------------------------------------------------

`ifdef VX_CFG_TCU_WGMMA_ENABLE
    wire [BLOCK_SIZE-1:0]                                          exec_valid_w;
    wire [BLOCK_SIZE-1:0]                                          exec_ready_w;
    tcu_execute_t                                                  exec_data_w [BLOCK_SIZE];
    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_obs
        assign exec_valid_w[bi]    = core_execute_if[bi].valid;
        assign exec_ready_w[bi]    = core_execute_if[bi].ready;
        assign exec_data_w[bi]     = core_execute_if[bi].data;
    end

    wire [BLOCK_SIZE-1:0][TCU_WG_A_DATA_SIZE-1:0][`VX_CFG_XLEN-1:0] tbuf_rs1_data;
    wire [BLOCK_SIZE-1:0][TCU_WG_RS2_WIDTH-1:0][`VX_CFG_XLEN-1:0] tbuf_rs2_data;
    wire [BLOCK_SIZE-1:0]                                         tbuf_ready_eff;

`ifdef PERF_ENABLE
    tcu_perf_t wg_perf;
    assign tcu_perf.tbuf_stalls     = wg_perf.tbuf_stalls;
    assign tcu_perf.tbuf_cache_hits = wg_perf.tbuf_cache_hits;
    assign tcu_perf.lmem_reads      = wg_perf.lmem_reads;
    assign tcu_perf.wgmma_instrs    = wg_perf.wgmma_instrs;
    assign tcu_perf.wgmma_stalls    = wg_perf.wgmma_stalls;
    `UNUSED_VAR ({wg_perf.uops, wg_perf.exec_stalls, wg_perf.busy})
    `UNUSED_VAR ({wg_perf.wg_lockstep_stalls, wg_perf.wg_tbuf_stalls, wg_perf.wg_credit_stalls,
                  wg_perf.wg_commit_stalls, wg_perf.wg_starve, wg_perf.wg_idle})
    wire [BLOCK_SIZE-1:0] perf_cta_conflict_w;
    wire [BLOCK_SIZE-1:0] perf_in_expansion_w;
`endif

    VX_tcu_wgmma #(
        .INSTANCE_ID (`SFORMATF(("%s-wgmma", INSTANCE_ID))),
        .BLOCK_SIZE  (BLOCK_SIZE)
    ) wgmma (
        .clk            (clk),
        .reset          (reset),
    `ifdef PERF_ENABLE
        .tcu_perf          (wg_perf),
        .perf_cta_conflict (perf_cta_conflict_w),
        .perf_in_expansion (perf_in_expansion_w),
    `endif
        .exec_valid     (exec_valid_w),
        .exec_ready     (exec_ready_w),
        .exec_data      (exec_data_w),
        .tcu_lmem_if    (`TCU_WG_LMEM_IF),
        .tbuf_rs1_data  (tbuf_rs1_data),
        .tbuf_rs2_data  (tbuf_rs2_data),
        .tbuf_ready_eff (tbuf_ready_eff)
    );

`else // !VX_CFG_TCU_WGMMA_ENABLE

`ifdef PERF_ENABLE
    assign tcu_perf.tbuf_stalls     = '0;
    assign tcu_perf.tbuf_cache_hits = '0;
    assign tcu_perf.lmem_reads      = '0;
    assign tcu_perf.wgmma_instrs    = '0;
    assign tcu_perf.wgmma_stalls    = '0;
    assign tcu_perf.wg_lockstep_stalls = '0;
    assign tcu_perf.wg_tbuf_stalls     = '0;
    assign tcu_perf.wg_credit_stalls   = '0;
    assign tcu_perf.wg_commit_stalls   = '0;
    assign tcu_perf.wg_starve          = '0;
    assign tcu_perf.wg_idle            = '0;
`endif

`endif // VX_CFG_TCU_WGMMA_ENABLE

`ifdef PERF_ENABLE
    wire [BLOCK_SIZE-1:0] perf_credit_full_w;   // per block, from VX_tcu_core
`endif

    // -----------------------------------------------------------------------
    // VX_tcu_agu — warp-level AGU for TCU_LD instructions.
    // Drives meta_wr signals broadcast to every tcu_core so wmma_sp on
    // any block sees the loaded metadata.
    // -----------------------------------------------------------------------
`ifdef TCU_META_ENABLE
    // The AGU streams NUM_TCU_LANES operands over the LSU memory client, whose
    // request/response mask and data are sized by NUM_LSU_LANES; the two lane
    // counts must match for the per-lane completion tracking to be correct.
    `STATIC_ASSERT (NUM_LANES == `VX_CFG_NUM_LSU_LANES, ("VX_tcu_agu requires NUM_TCU_LANES == NUM_LSU_LANES"));

    wire                                              agu_meta_wr_en;
    wire [NW_WIDTH-1:0]                               agu_meta_wr_wid;
    wire [4:0]                                        agu_meta_wr_idx;
    wire [TCU_BLOCK_CAP-1:0][`VX_CFG_XLEN-1:0]        agu_meta_wr_data;

    VX_tcu_agu #(
        .INSTANCE_ID (`SFORMATF(("%s-agu", INSTANCE_ID))),
        .BLOCK_SIZE  (BLOCK_SIZE),
        .NUM_LANES   (NUM_LANES)
    ) agu (
        .clk                (clk),
        .reset              (reset),
        .per_block_ld_valid (agu_ld_valid),
        .per_block_ld_data  (agu_ld_data),
        .per_block_ld_ready (agu_ld_ready),
        .client_if          (tcu_mem_if),
        .meta_wr_en         (agu_meta_wr_en),
        .meta_wr_wid        (agu_meta_wr_wid),
        .meta_wr_idx        (agu_meta_wr_idx),
        .meta_wr_data       (agu_meta_wr_data),
        .result_valid       (agu_result_valid),
        .result_data        (agu_result_data),
        .result_ready       (agu_result_ready)
    );
`endif

    // -----------------------------------------------------------------------
    // TCU core instances
    // -----------------------------------------------------------------------

    for (genvar block_idx = 0; block_idx < BLOCK_SIZE; ++block_idx) begin : g_blocks
        VX_tcu_core #(
            .INSTANCE_ID (`SFORMATF(("%s-fused%0d", INSTANCE_ID, block_idx)))
        ) tcu_core (
            `SCOPE_IO_BIND (block_idx)
            .clk        (clk),
            .reset      (reset),
        `ifdef VX_CFG_TCU_WGMMA_ENABLE
            .tbuf_rs1_data (tbuf_rs1_data[block_idx]),
            .tbuf_rs2_data (tbuf_rs2_data[block_idx]),
            .tbuf_ready    (tbuf_ready_eff[block_idx]),
        `endif
        `ifdef TCU_META_ENABLE
            .ext_meta_wr_en   (agu_meta_wr_en),
            .ext_meta_wr_wid  (agu_meta_wr_wid),
            .ext_meta_wr_idx  (agu_meta_wr_idx),
            .ext_meta_wr_data (agu_meta_wr_data),
        `endif
            .execute_if (core_execute_if[block_idx]),
            .result_if  (core_result_if[block_idx])
        `ifdef PERF_ENABLE
            , .perf_credit_full (perf_credit_full_w[block_idx])
        `endif
        );
    end

    // -----------------------------------------------------------------------
    // Lane gather
    // -----------------------------------------------------------------------

    VX_lane_gather #(
        .BLOCK_SIZE (BLOCK_SIZE),
        .NUM_LANES  (NUM_LANES),
        .OUT_BUF    (3)
    ) lane_gather (
        .clk       (clk),
        .reset     (reset),
        .result_if (per_block_result_if),
        .commit_if (commit_if)
    );

`ifdef PERF_ENABLE
    // Block-summed TCU core occupancy: one uop per cycle per block at peak.
    wire [BLOCK_SIZE-1:0] perf_exe_fire, perf_exe_stall, perf_res_fire, perf_busy;
    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_perf_occ
        assign perf_exe_fire[bi]  = core_execute_if[bi].valid && core_execute_if[bi].ready;
        assign perf_exe_stall[bi] = core_execute_if[bi].valid && ~core_execute_if[bi].ready;
        assign perf_res_fire[bi]  = core_result_if[bi].valid && core_result_if[bi].ready;
        reg [7:0] inflight;
        always @(posedge clk) begin
            if (reset) begin
                inflight <= '0;
            end else begin
                inflight <= inflight + 8'(perf_exe_fire[bi])
                          - 8'(perf_res_fire[bi] && (inflight != 0 || perf_exe_fire[bi]));
            end
        end
        assign perf_busy[bi] = (inflight != 0) || perf_exe_fire[bi];
    end
    wire [`CLOG2(BLOCK_SIZE+1)-1:0] perf_fire_cnt, perf_stall_cnt, perf_busy_cnt;
    `POP_COUNT(perf_fire_cnt, perf_exe_fire);
    `POP_COUNT(perf_stall_cnt, perf_exe_stall);
    `POP_COUNT(perf_busy_cnt, perf_busy);
    reg [PERF_CTR_BITS-1:0] perf_uops_r, perf_stalls_r, perf_busy_r;
    always @(posedge clk) begin
        if (reset) begin
            perf_uops_r   <= '0;
            perf_stalls_r <= '0;
            perf_busy_r   <= '0;
        end else begin
            perf_uops_r   <= perf_uops_r   + PERF_CTR_BITS'(perf_fire_cnt);
            perf_stalls_r <= perf_stalls_r + PERF_CTR_BITS'(perf_stall_cnt);
            perf_busy_r   <= perf_busy_r   + PERF_CTR_BITS'(perf_busy_cnt);
        end
    end
    assign tcu_perf.uops        = perf_uops_r;
    assign tcu_perf.exec_stalls = perf_stalls_r;
    assign tcu_perf.busy        = perf_busy_r;

`ifdef VX_CFG_TCU_WGMMA_ENABLE
    // WGMMA stall attribution (block-cycles, block-summed). The four stall causes are mutually exclusive with
    // priority lockstep > tile buffer > credit > commit, so they sum exactly to wgmma_stalls:
    //   lockstep: tbuf_ready_eff low because another block holds a different CTA
    //   tbuf    : tbuf_ready_eff low because the A/B tile buffers have not delivered (incl. block key mismatch)
    //   credit  : operands ready but the core's header queue / landing-queue credits are full
    //   commit  : operands ready, credits free, result back-pressure (fedp_done && !result ready, setup pending)
    // starve / idle: no uop presented while the block is mid-WGMMA-expansion / between instructions.
    wire [BLOCK_SIZE-1:0] wgp_lock, wgp_tbuf, wgp_cred, wgp_comm, wgp_starve, wgp_idle;
    for (genvar bi = 0; bi < BLOCK_SIZE; ++bi) begin : g_perf_wg
        wire is_wg = (core_execute_if[bi].data.op_type == INST_TCU_WGMMA)
                `ifdef VX_CFG_TCU_SPARSE_ENABLE
                   || (core_execute_if[bi].data.op_type == INST_TCU_WGMMA_SP)
                `endif
                   ;
        wire stall = core_execute_if[bi].valid && ~core_execute_if[bi].ready && is_wg;
        assign wgp_lock[bi]   = stall && ~tbuf_ready_eff[bi] && perf_cta_conflict_w[bi];
        assign wgp_tbuf[bi]   = stall && ~tbuf_ready_eff[bi] && ~perf_cta_conflict_w[bi];
        assign wgp_cred[bi]   = stall && tbuf_ready_eff[bi] && perf_credit_full_w[bi];
        assign wgp_comm[bi]   = stall && tbuf_ready_eff[bi] && ~perf_credit_full_w[bi];
        assign wgp_starve[bi] = ~core_execute_if[bi].valid && perf_in_expansion_w[bi];
        assign wgp_idle[bi]   = ~core_execute_if[bi].valid && ~perf_in_expansion_w[bi];
    end
    wire [`CLOG2(BLOCK_SIZE+1)-1:0] wgp_lock_c, wgp_tbuf_c, wgp_cred_c, wgp_comm_c, wgp_starve_c, wgp_idle_c;
    `POP_COUNT(wgp_lock_c,   wgp_lock);
    `POP_COUNT(wgp_tbuf_c,   wgp_tbuf);
    `POP_COUNT(wgp_cred_c,   wgp_cred);
    `POP_COUNT(wgp_comm_c,   wgp_comm);
    `POP_COUNT(wgp_starve_c, wgp_starve);
    `POP_COUNT(wgp_idle_c,   wgp_idle);
    reg [PERF_CTR_BITS-1:0] wgp_lock_r, wgp_tbuf_r, wgp_cred_r, wgp_comm_r, wgp_starve_r, wgp_idle_r;
    always @(posedge clk) begin
        if (reset) begin
            wgp_lock_r   <= '0;
            wgp_tbuf_r   <= '0;
            wgp_cred_r   <= '0;
            wgp_comm_r   <= '0;
            wgp_starve_r <= '0;
            wgp_idle_r   <= '0;
        end else begin
            wgp_lock_r   <= wgp_lock_r   + PERF_CTR_BITS'(wgp_lock_c);
            wgp_tbuf_r   <= wgp_tbuf_r   + PERF_CTR_BITS'(wgp_tbuf_c);
            wgp_cred_r   <= wgp_cred_r   + PERF_CTR_BITS'(wgp_cred_c);
            wgp_comm_r   <= wgp_comm_r   + PERF_CTR_BITS'(wgp_comm_c);
            wgp_starve_r <= wgp_starve_r + PERF_CTR_BITS'(wgp_starve_c);
            wgp_idle_r   <= wgp_idle_r   + PERF_CTR_BITS'(wgp_idle_c);
        end
    end
    assign tcu_perf.wg_lockstep_stalls = wgp_lock_r;
    assign tcu_perf.wg_tbuf_stalls     = wgp_tbuf_r;
    assign tcu_perf.wg_credit_stalls   = wgp_cred_r;
    assign tcu_perf.wg_commit_stalls   = wgp_comm_r;
    assign tcu_perf.wg_starve          = wgp_starve_r;
    assign tcu_perf.wg_idle            = wgp_idle_r;
`else
    `UNUSED_VAR (perf_credit_full_w)
`endif
`endif

`undef TCU_WG_LMEM_IF

`endif // VX_CFG_TCU_SEQ_ONLY

endmodule
