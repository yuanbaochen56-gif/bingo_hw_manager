// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Simulation-only invariants of bingo_hw_manager_top: replay, remap, parking
// and the retirement of checkout heads. Each violation is an $error with the
// [BINGO_ASSERT] tag; nothing here drives the design.
//
// The top instantiates this checker with implicit port connections (.*), so
// every port is named after the top signal it observes. It is not attached
// with a bind statement: Questa only elaborates a bind outside a module when
// the file is compiled as one compilation unit with the design (-mfcu), which
// the Bender scripts do not do, and a bind inside the top is repeated for
// every instance of the top (several chips in one testbench).
`ifndef SYNTHESIS
module bingo_hw_manager_top_assert #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CheckoutQueueDepth = 8,
    parameter bit          ForeignStuckDrain = 1'b1,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned OutstandingWidth = 1,
    parameter type         task_desc_t = logic,      // bingo_hw_manager_task_desc_t of the top
    parameter type         done_info_t = logic,      // bingo_hw_manager_done_info_full_t
    parameter type         remote_tag_t = logic,
    parameter type         remote_export_t = logic
) (
    input logic                                       clk_i,
    input logic                                       rst_ni,
    // Watchdog and replay
    input logic       [NumCores-1:0][NumClusters-1:0] core_fenced,
    input logic       [NumCores-1:0][NumClusters-1:0] core_retired,
    input logic       [NumCores-1:0][NumClusters-1:0] replay_stuck_slot,
    input logic       [NumCores-1:0][NumClusters-1:0] replay_blocked_o,
    input logic       [NumCores-1:0][NumClusters-1:0] replay_move,
    input logic       [NumCores-1:0][NumClusters-1:0] replay_push,
    input logic       [NumCores-1:0][NumClusters-1:0] replay_pop,
    input logic                                       replay_push_ready_q,
    input task_desc_t                                 replay_data,
    input logic [CoreIdWidth-1:0]                     replay_src_core,
    input logic [ClusterIdWidth-1:0]                  replay_src_cluster,
    // bingo_hw_manager_replay_ctrl: the slot had an executing entry moved
    input logic       [NumCores-1:0][NumClusters-1:0] exec_moved_q,
    // Their values one cycle earlier (the top's simulation log)
    input logic       [NumCores-1:0][NumClusters-1:0] core_fenced_log_q,
    input logic       [NumCores-1:0][NumClusters-1:0] core_retired_log_q,
    input logic       [NumCores-1:0][NumClusters-1:0] replay_stuck_log_q,
    // Level 3 exports
    input logic                                       export_push,
    input remote_export_t                             export_in,
    // Routing and parking
    input task_desc_t [NumCores-1:0]                  waiting_dep_check_task_desc,
    input logic       [NumCores-1:0][NumClusters-1:0] remap_route_fire,
    input logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0] remap_route_src_core,
    input logic       [NumCores-1:0][NumClusters-1:0] park_block,
    input logic [OutstandingWidth-1:0]                remap_outstanding_q [NumCores][NumClusters],
    input logic [OutstandingWidth-1:0]                moved_in_q [NumCores][NumClusters][NumCores][NumClusters],
    // Ready, checkout and done queues
    input logic       [NumCores-1:0][NumClusters-1:0] ready_queue_push,
    input logic       [NumCores-1:0][NumClusters-1:0] ready_queue_pop,
    input logic       [NumCores-1:0][NumClusters-1:0] checkout_queue_push,
    input logic       [NumCores-1:0][NumClusters-1:0] checkout_queue_pop,
    input task_desc_t [NumCores-1:0][NumClusters-1:0] checkout_queue_data_in,
    input task_desc_t [NumCores-1:0][NumClusters-1:0] checkout_queue_data_out,
    input remote_tag_t [NumCores-1:0][NumClusters-1:0] checkout_remote_tag_in,
    input logic       [NumCores-1:0][NumClusters-1:0] checkout_head_exec,
    input logic       [NumCores-1:0][NumClusters-1:0] stuck_drain,
    input logic       [NumCores-1:0][NumClusters-1:0] stuck_head_skip,
    input logic       [NumCores-1:0][NumClusters-1:0] done_q_push,
    input logic       [NumCores-1:0][NumClusters-1:0] done_q_pop,
    input logic       [NumCores-1:0][NumClusters-1:0] done_q_empty,
    input done_info_t [NumCores-1:0][NumClusters-1:0] done_q_info,
    input logic       [NumCores-1:0][NumClusters-1:0] remote_done_push
);
    import bingo_hw_manager_pkg::*;

    // Replay invariants
    always @(posedge clk_i) begin : replay_assertions
        if (rst_ni) begin
            if (|(replay_blocked_o & ~replay_stuck_slot)) begin
                $error("[BINGO_ASSERT] blocked slot is not stuck");
            end
            if (export_push && export_in.desc.task_type == TaskNoReplay) begin
                $error("[BINGO_ASSERT] exported no-replay task %0d", export_in.desc.task_id);
            end
            if (replay_push_ready_q && replay_data.task_type == TaskNoReplay &&
                !exec_moved_q[replay_src_core][replay_src_cluster]) begin
                $error("[BINGO_ASSERT] replayed possibly started no-replay task %0d",
                       replay_data.task_id);
            end
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    // A parked slot's own new task must not be routed while it is still
                    // in HOLD (the checkout is draining).
                    if (remap_route_fire[c][cl]) begin
                        automatic int unsigned psrc = remap_route_src_core[c][cl];
                        automatic int unsigned pscl =
                            waiting_dep_check_task_desc[psrc].assigned_cluster_id;
                        if (park_block[psrc][pscl]) begin
                            $error("[BINGO_ASSERT] route of task %0d while logical core %0d cluster %0d is in HOLD",
                                   waiting_dep_check_task_desc[psrc].task_id, psrc, pscl);
                        end
                    end
                    // A1: a fenced core neither takes a task nor retires one
                    if (core_fenced[c][cl] && (ready_queue_pop[c][cl] ||
                                               (done_q_push[c][cl] && !remote_done_push[c][cl]))) begin
                        $error("[BINGO_ASSERT] fenced core %0d cluster %0d: ready pop %0b / done push %0b",
                               c, cl, ready_queue_pop[c][cl], done_q_push[c][cl]);
                    end
                    // A2: a fenced core receives no executing task; before it is
                    // retired or stuck it receives nothing at all
                    if (core_fenced[c][cl] && ready_queue_push[c][cl]) begin
                        $error("[BINGO_ASSERT] ready push into fenced core %0d cluster %0d", c, cl);
                    end
                    // (level 3: exported entries stay on their fenced proxy slot)
                    if (core_fenced[c][cl] && checkout_queue_push[c][cl] &&
                        !checkout_remote_tag_in[c][cl].exported &&
                        (!(core_retired[c][cl] || replay_stuck_slot[c][cl]) ||
                         (checkout_queue_data_in[c][cl].task_type != TaskDummy))) begin
                        $error("[BINGO_ASSERT] checkout push of task %0d (type %0d) into fenced core %0d cluster %0d (retired %0b)",
                               checkout_queue_data_in[c][cl].task_id, checkout_queue_data_in[c][cl].task_type,
                               c, cl, core_retired[c][cl]);
                    end
                    // A3: outstanding counter never wraps
                    if (remap_outstanding_q[c][cl] > NumCores * CheckoutQueueDepth) begin
                        $error("[BINGO_ASSERT] remap_outstanding[%0d][%0d] out of range: %0d",
                               c, cl, remap_outstanding_q[c][cl]);
                    end
                    // A3b: the per-slot split adds up to the logical core's count
                    if (ForeignStuckDrain) begin
                        automatic int unsigned moved_sum = 0;
                        for (int unsigned p = 0; p < NumCores; p++)
                            for (int unsigned pcl = 0; pcl < NumClusters; pcl++)
                                moved_sum += moved_in_q[p][pcl][c][cl];
                        if (moved_sum != remap_outstanding_q[c][cl]) begin
                            $error("[BINGO_ASSERT] moved_in of logical core %0d cluster %0d adds up to %0d, remap_outstanding is %0d",
                                   c, cl, moved_sum, remap_outstanding_q[c][cl]);
                        end
                    end
                    // A4: a slot being moved is not retired through its done queue
                    if (replay_move[c][cl] && done_q_pop[c][cl]) begin
                        $error("[BINGO_ASSERT] done pop on core %0d cluster %0d during replay MOVE", c, cl);
                    end
                    // A5: a replay push never collides with a routed push
                    if (replay_push[c][cl] && remap_route_fire[c][cl]) begin
                        $error("[BINGO_ASSERT] replay and routed push into core %0d cluster %0d", c, cl);
                    end
                    // A7: an executing task retires together with its own done
                    // (a stuck slot's head that the CERF skips does not execute)
                    if (checkout_queue_pop[c][cl] && !replay_pop[c][cl] &&
                        (checkout_queue_data_out[c][cl].task_type != TaskDummy) && !stuck_head_skip[c][cl] &&
                        (!done_q_pop[c][cl] || done_q_empty[c][cl] ||
                         (done_q_info[c][cl].task_id != checkout_queue_data_out[c][cl].task_id))) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d retired task %0d with done pop %0b (done task %0d)",
                               c, cl, checkout_queue_data_out[c][cl].task_id, done_q_pop[c][cl],
                               done_q_info[c][cl].task_id);
                    end
                    if (done_q_pop[c][cl] &&
                        !(checkout_queue_pop[c][cl] && !replay_pop[c][cl])) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d popped done of task %0d without retiring a task",
                               c, cl, done_q_info[c][cl].task_id);
                    end
                    // A8: a stuck slot keeps its entries, except heads that run on
                    // no core (dummy-set or CERF-skipped) of its own logical core
                    if (replay_stuck_slot[c][cl] && checkout_queue_pop[c][cl] &&
                        !(stuck_drain[c][cl] && !checkout_head_exec[c][cl])) begin
                        $error("[BINGO_ASSERT] stuck core %0d cluster %0d popped task %0d",
                               c, cl, checkout_queue_data_out[c][cl].task_id);
                    end
                    // A6: fenced / retired / stuck are sticky
                    if ((core_fenced_log_q[c][cl] && !core_fenced[c][cl]) ||
                        (core_retired_log_q[c][cl] && !core_retired[c][cl]) ||
                        (replay_stuck_log_q[c][cl] && !replay_stuck_slot[c][cl])) begin
                        $error("[BINGO_ASSERT] fenced/retired/stuck of core %0d cluster %0d fell", c, cl);
                    end
                end
            end
        end
    end
endmodule
`endif
