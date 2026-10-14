// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Simulation-only invariants of bingo_hw_manager_top: replay, remap, parking
// and the retirement of checkout heads. Each violation is an $error with the
// [BINGO_ASSERT] tag; nothing here drives the design.
//
// It reads the structs of the top (slot state, queues, route, replay step)
// as the submodules do. It is not attached with a bind statement: Questa only
// elaborates a bind outside a module when the file is compiled as one
// compilation unit with the design (-mfcu), which the Bender scripts do not
// do, and a bind inside the top is repeated for every instance of the top
// (several chips in one testbench).
`ifndef SYNTHESIS
module bingo_hw_manager_top_assert #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CheckoutQueueDepth = 8,
    parameter bit          ForeignStuckDrain = 1'b1,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned OutstandingWidth = 1,
    parameter type         remote_export_t = logic,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         slot_state_t = logic,
    parameter type         replay_step_t = logic,
    parameter type         replay_op_t = logic,
    parameter type         waiting_t = logic,
    parameter type         route_t = logic,
    parameter type         ready_t = logic,
    parameter type         checkout_t = logic,
    parameter type         done_t = logic,
    parameter type         done_remote_t = logic
) (
    input logic                                 clk_i,
    input logic                                 rst_ni,
    // Watchdog and replay: slot state and blocked slots, and their values one
    // cycle earlier (the top's simulation log)
    input slot_state_t                          slot_i,
    input logic [NumCores-1:0][NumClusters-1:0] blocked_i,
    input slot_state_t                          slot_log_i,
    // A MOVE step of the replay controller and what it does to the queues;
    // the slots that had an executing entry moved (bingo_hw_manager_replay_ctrl)
    input replay_step_t                         step_i,
    input replay_op_t                           replay_op_i,
    input logic [NumCores-1:0][NumClusters-1:0] exec_moved_i,
    // Level 3 exports
    input logic                                 export_push_i,
    input remote_export_t                       export_i,
    // Routing and parking
    input waiting_t                             waiting_i,
    input route_t                               route_i,
    input logic [NumCores-1:0][NumClusters-1:0] park_block_i,
    input logic [OutstandingWidth-1:0]          outstanding_i [NumCores][NumClusters],
    input logic [OutstandingWidth-1:0]          moved_in_i [NumCores][NumClusters][NumCores][NumClusters],
    // Ready, checkout and done queues
    input ready_t                               ready_i,
    input checkout_t                            checkout_i,
    input logic [NumCores-1:0][NumClusters-1:0] stuck_drain_i,
    input logic [NumCores-1:0][NumClusters-1:0] stuck_head_skip_i,
    input done_t                                done_i,
    input logic [NumCores-1:0][NumClusters-1:0] done_pop_i,
    input done_remote_t                         remote_i
);
    import bingo_hw_manager_pkg::*;

    // Replay invariants
    always @(posedge clk_i) begin : replay_assertions
        if (rst_ni) begin
            if (|(blocked_i & ~slot_i.stuck)) begin
                $error("[BINGO_ASSERT] blocked slot is not stuck");
            end
            if (export_push_i && export_i.desc.task_type == TaskNoReplay) begin
                $error("[BINGO_ASSERT] exported no-replay task %0d", export_i.desc.task_id);
            end
            if (step_i.push_ready && replay_op_i.data.task_type == TaskNoReplay &&
                !exec_moved_i[step_i.src_core][step_i.src_cluster]) begin
                $error("[BINGO_ASSERT] replayed possibly started no-replay task %0d",
                       replay_op_i.data.task_id);
            end
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    // A parked slot's own new task must not be routed while it is still
                    // in HOLD (the checkout is draining).
                    if (route_i.fire[c][cl]) begin
                        automatic int unsigned psrc = route_i.src_core[c][cl];
                        automatic int unsigned pscl =
                            waiting_i.desc[psrc].assigned_cluster_id;
                        if (park_block_i[psrc][pscl]) begin
                            $error("[BINGO_ASSERT] route of task %0d while logical core %0d cluster %0d is in HOLD",
                                   waiting_i.desc[psrc].task_id, psrc, pscl);
                        end
                    end
                    // A1: a fenced core neither takes a task nor retires one
                    if (slot_i.fenced[c][cl] && (ready_i.pop[c][cl] ||
                                               (done_i.push[c][cl] && !remote_i.push[c][cl]))) begin
                        $error("[BINGO_ASSERT] fenced core %0d cluster %0d: ready pop %0b / done push %0b",
                               c, cl, ready_i.pop[c][cl], done_i.push[c][cl]);
                    end
                    // A2: a fenced core receives no executing task; before it is
                    // retired or stuck it receives nothing at all
                    if (slot_i.fenced[c][cl] && ready_i.push[c][cl]) begin
                        $error("[BINGO_ASSERT] ready push into fenced core %0d cluster %0d", c, cl);
                    end
                    // (level 3: exported entries stay on their fenced proxy slot)
                    if (slot_i.fenced[c][cl] && checkout_i.push[c][cl] &&
                        !checkout_i.push_tag[c][cl].exported &&
                        (!(slot_i.retired[c][cl] || slot_i.stuck[c][cl]) ||
                         (checkout_i.push_desc[c][cl].task_type != TaskDummy))) begin
                        $error("[BINGO_ASSERT] checkout push of task %0d (type %0d) into fenced core %0d cluster %0d (retired %0b)",
                               checkout_i.push_desc[c][cl].task_id, checkout_i.push_desc[c][cl].task_type,
                               c, cl, slot_i.retired[c][cl]);
                    end
                    // A3: outstanding counter never wraps
                    if (outstanding_i[c][cl] > NumCores * CheckoutQueueDepth) begin
                        $error("[BINGO_ASSERT] remap_outstanding[%0d][%0d] out of range: %0d",
                               c, cl, outstanding_i[c][cl]);
                    end
                    // A3b: the per-slot split adds up to the logical core's count
                    if (ForeignStuckDrain) begin
                        automatic int unsigned moved_sum = 0;
                        for (int unsigned p = 0; p < NumCores; p++)
                            for (int unsigned pcl = 0; pcl < NumClusters; pcl++)
                                moved_sum += moved_in_i[p][pcl][c][cl];
                        if (moved_sum != outstanding_i[c][cl]) begin
                            $error("[BINGO_ASSERT] moved_in of logical core %0d cluster %0d adds up to %0d, remap_outstanding is %0d",
                                   c, cl, moved_sum, outstanding_i[c][cl]);
                        end
                    end
                    // A4: a slot being moved is not retired through its done queue
                    if (step_i.move[c][cl] && done_pop_i[c][cl]) begin
                        $error("[BINGO_ASSERT] done pop on core %0d cluster %0d during replay MOVE", c, cl);
                    end
                    // A5: a replay push never collides with a routed push
                    if (replay_op_i.push[c][cl] && route_i.fire[c][cl]) begin
                        $error("[BINGO_ASSERT] replay and routed push into core %0d cluster %0d", c, cl);
                    end
                    // A7: an executing task retires together with its own done
                    // (a stuck slot's head that the CERF skips does not execute)
                    if (checkout_i.pop[c][cl] && !replay_op_i.pop[c][cl] &&
                        (checkout_i.head[c][cl].task_type != TaskDummy) && !stuck_head_skip_i[c][cl] &&
                        (!done_pop_i[c][cl] || done_i.empty[c][cl] ||
                         (done_i.head[c][cl].task_id != checkout_i.head[c][cl].task_id))) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d retired task %0d with done pop %0b (done task %0d)",
                               c, cl, checkout_i.head[c][cl].task_id, done_pop_i[c][cl],
                               done_i.head[c][cl].task_id);
                    end
                    if (done_pop_i[c][cl] &&
                        !(checkout_i.pop[c][cl] && !replay_op_i.pop[c][cl])) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d popped done of task %0d without retiring a task",
                               c, cl, done_i.head[c][cl].task_id);
                    end
                    // A8: a stuck slot keeps its entries, except heads that run on
                    // no core (dummy-set or CERF-skipped) of its own logical core
                    if (slot_i.stuck[c][cl] && checkout_i.pop[c][cl] &&
                        !(stuck_drain_i[c][cl] && !checkout_i.head_exec[c][cl])) begin
                        $error("[BINGO_ASSERT] stuck core %0d cluster %0d popped task %0d",
                               c, cl, checkout_i.head[c][cl].task_id);
                    end
                    // A6: fenced / retired / stuck are sticky
                    if ((slot_log_i.fenced[c][cl] && !slot_i.fenced[c][cl]) ||
                        (slot_log_i.retired[c][cl] && !slot_i.retired[c][cl]) ||
                        (slot_log_i.stuck[c][cl] && !slot_i.stuck[c][cl])) begin
                        $error("[BINGO_ASSERT] fenced/retired/stuck of core %0d cluster %0d fell", c, cl);
                    end
                end
            end
        end
    end
endmodule
`endif
