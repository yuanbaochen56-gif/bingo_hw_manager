// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Data path of task replay: what the replay controller
// (bingo_hw_manager_replay_ctrl) sees of each checkout head, and what one of
// its MOVE steps does to the queues of the slots.
//
// Each checkout head is presented by its logical slot and kind: it runs on no
// core (a dummy, or an exit that is absorbed when it moves), it must not be
// replayed (task_type 11), or it was exported by this slot (level 3).
// An exit task is absorbed into a dummy when it would leave its own logical
// slot: its slot has a substitute elsewhere, or no substitute but level 3
// may export its type (and it was not imported). Its own core then still
// sees its exit, and the substitute is not stopped by another core's exit.
//
// A MOVE step pops the head of the source slot and pushes it into the
// destination (into its ready queue as well when it runs on a core); a
// bounced imported entry is only popped. The moved entry is the source head,
// an absorbed exit rewritten as a dummy.
module bingo_hw_manager_replay_path #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    parameter bit          RemoteEn = 1'b0,
    parameter type         task_desc_t = logic,   // bingo_hw_manager_task_desc_t of the top
    parameter type         remote_tag_t = logic
) (
    // Checkout heads
    input  task_desc_t  [NumCores-1:0][NumClusters-1:0] head_i,
    input  remote_tag_t [NumCores-1:0][NumClusters-1:0] head_tag_i,
    input  logic        [NumCores-1:0][NumClusters-1:0] head_imported_i,
    // Substitute of each logical slot (bingo_hw_manager_ctrl), and the core
    // types level 3 may export
    input  logic [NumCores-1:0][NumClusters-1:0]                     smt_found_i,
    input  logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_i,
    input  logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_i,
    input  logic [2**CoreTypeIdWidth-1:0]                            remote_type_en_i,
    // The heads as the replay controller sees them
    output logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    head_logical_core_o,
    output logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] head_logical_cluster_o,
    output logic [NumCores-1:0][NumClusters-1:0]                     head_no_exec_o,
    output logic [NumCores-1:0][NumClusters-1:0]                     head_no_replay_o,
    output logic [NumCores-1:0][NumClusters-1:0]                     head_exported_o,
    // Slot state, for the pending replay
    input  logic [NumCores-1:0][NumClusters-1:0]                     fenced_i,
    input  logic [NumCores-1:0][NumClusters-1:0]                     retired_i,
    input  logic [NumCores-1:0][NumClusters-1:0]                     stuck_i,
    // A MOVE step of the replay controller
    input  logic [NumCores-1:0][NumClusters-1:0]                     move_i,
    input  logic                                                     move_fire_i,
    input  logic                                                     push_ready_i,
    input  logic                                                     bounce_i,
    input  logic [CoreIdWidth-1:0]                                   src_core_i,
    input  logic [ClusterIdWidth-1:0]                                src_cluster_i,
    input  logic [CoreIdWidth-1:0]                                   dst_core_i,
    input  logic [ClusterIdWidth-1:0]                                dst_cluster_i,
    // ... the entry it moves, and what it does to the queues
    output task_desc_t                                               data_o,
    output logic                                                     exit_absorb_o,
    output logic [NumCores-1:0][NumClusters-1:0]                     pop_o,
    output logic [NumCores-1:0][NumClusters-1:0]                     push_o,
    output logic [NumCores-1:0][NumClusters-1:0]                     push_ready_o,
    // A slot is fenced, neither retired nor stuck; a MOVE may push into a cluster
    output logic                                                     pending_o,
    output logic [NumClusters-1:0]                                   move_cluster_o
);
    import bingo_hw_manager_pkg::*;

    logic [NumCores-1:0][NumClusters-1:0] head_exit_absorb;
    always_comb begin : compose_replay_inputs
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                automatic int logical_core = int'(head_i[c][cl].assigned_core_id);
                automatic int logical_cluster = int'(head_i[c][cl].assigned_cluster_id);
                head_exported_o[c][cl]        = head_tag_i[c][cl].exported;
                head_logical_core_o[c][cl]    = head_i[c][cl].assigned_core_id;
                head_logical_cluster_o[c][cl] = head_i[c][cl].assigned_cluster_id;
                head_exit_absorb[c][cl] = 1'b0;
                if (logical_core < NumCores && logical_cluster < NumClusters) begin
                    head_exit_absorb[c][cl] =
                        head_i[c][cl].is_exit &&
                        head_i[c][cl].task_type != TaskDummy &&
                        ((smt_found_i[logical_core][logical_cluster] &&
                          ((smt_core_i[logical_core][logical_cluster] != logical_core) ||
                           (smt_cluster_i[logical_core][logical_cluster] != logical_cluster))) ||
                         (RemoteEn && !smt_found_i[logical_core][logical_cluster] &&
                          !head_imported_i[c][cl] &&
                          CoreTypeId[logical_core][logical_cluster] != '0 &&
                          remote_type_en_i[CoreTypeId[logical_core][logical_cluster]]));
                end
                head_no_exec_o[c][cl] = (head_i[c][cl].task_type == TaskDummy) ||
                                        head_exit_absorb[c][cl];
                head_no_replay_o[c][cl] = (head_i[c][cl].task_type == TaskNoReplay);
            end
        end
    end

    assign exit_absorb_o = head_exit_absorb[src_core_i][src_cluster_i];
    always_comb begin
        data_o = head_i[src_core_i][src_cluster_i];
        if (exit_absorb_o) data_o.task_type = TaskDummy;
    end

    always_comb begin : compose_replay_signals
        pop_o          = '0;
        push_o         = '0;
        push_ready_o   = '0;
        pending_o      = 1'b0;
        move_cluster_o = '0;
        if (move_fire_i) begin
            pop_o[src_core_i][src_cluster_i] = 1'b1;
            // Level 3: a bounced (rejected) imported entry is only dropped
            if (!bounce_i) begin
                push_o[dst_core_i][dst_cluster_i]       = 1'b1;
                push_ready_o[dst_core_i][dst_cluster_i] = push_ready_i;
            end
        end
        // Nothing is routed into the slot being moved (fenced, not retired), only
        // into its current destination, which may be in another cluster.
        if (|move_i) begin
            move_cluster_o[dst_cluster_i] = 1'b1;
        end
        for (int unsigned cl = 0; cl < NumClusters; cl++) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                // A stuck slot is never migrated, so a retired core's new tasks
                // need not wait for it (its entries could only go to a core of
                // its own type, and none is live).
                if (fenced_i[c][cl] && !retired_i[c][cl] && !stuck_i[c][cl]) begin
                    pending_o = 1'b1;
                end
            end
        end
    end
endmodule
