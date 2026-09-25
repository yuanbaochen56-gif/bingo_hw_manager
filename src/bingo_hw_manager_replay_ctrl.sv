// Copyright 2025 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Replay controller: moves the outstanding tasks of a fenced (confirmed dead)
// core to live cores, so that they are executed again.
//
// A core's checkout FIFO holds every task dispatched to it and not yet retired,
// in dispatch order: the head is the task the core was running when it died,
// the rest were queued behind it. The dep checks of these tasks already passed,
// so they are pushed straight into the ready + checkout FIFOs of a substitute.
// Their dep_set still uses the logical core id of the descriptor, so the
// dependents see the same producer as before.
//
// One slot is migrated at a time:
//   IDLE   pick the lowest fenced, not yet retired slot D
//   DRAIN  flush D's ready FIFO; wait until D's done FIFO is empty (a done that
//          arrived before the fence retires its task normally, no replay)
//   MOVE   pop D's checkout head and push it to substitute S, chosen per entry
//          from the entry's logical core; stall while S is full or none exists
//   FINISH mark D retired
// While a slot of a cluster is fenced and not retired, the top blocks normal
// routing into that cluster, so the migrated (older) tasks of a logical core
// always enter a substitute before its newer ones.
module bingo_hw_manager_replay_ctrl #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // AllowMask[logical][physical] = 1: `physical` may run the tasks of `logical`
    // (same as bingo_hw_manager_core_remap).
    parameter logic [NumCores-1:0][NumCores-1:0] AllowMask = '1
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  logic [NumCores-1:0][NumClusters-1:0] fenced_i,
    input  logic [NumCores-1:0][NumClusters-1:0] done_q_empty_i,
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_empty_i,
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_full_i,
    input  logic [NumCores-1:0][NumClusters-1:0] ready_full_i,
    // Fields of each checkout head
    input  logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0] checkout_logical_core_i,
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_no_exec_i, // task_type 01: checkout only

    output logic [NumCores-1:0][NumClusters-1:0] retired_o,
    output logic [NumCores-1:0][NumClusters-1:0] ready_flush_o,
    // Slot currently drained by MOVE: its normal checkout output must be held
    output logic [NumCores-1:0][NumClusters-1:0] move_o,
    // One MOVE step: pop the head of (src_core_o, src_cluster_o) and push it to
    // (dst_core_o, src_cluster_o); push_ready_o also pushes its task id to the
    // ready FIFO of the destination.
    output logic                      move_fire_o,
    output logic                      push_ready_o,
    output logic [CoreIdWidth-1:0]    src_core_o,
    output logic [ClusterIdWidth-1:0] src_cluster_o,
    output logic [CoreIdWidth-1:0]    dst_core_o,
    // A fenced slot holds an entry that no live core may run
    output logic                      stuck_o
);

    typedef enum logic [1:0] {
        IDLE,
        DRAIN,
        MOVE,
        FINISH
    } replay_state_t;

    replay_state_t state_q, state_d;
    logic [CoreIdWidth-1:0]    src_core_q, src_core_d;
    logic [ClusterIdWidth-1:0] src_cluster_q, src_cluster_d;
    logic [NumCores-1:0][NumClusters-1:0] retired_q, retired_d;

    // Lowest fenced, not yet retired slot
    logic                      pending_found;
    logic [CoreIdWidth-1:0]    pending_core;
    logic [ClusterIdWidth-1:0] pending_cluster;

    always_comb begin
        pending_found   = 1'b0;
        pending_core    = '0;
        pending_cluster = '0;
        for (int cl = NumClusters - 1; cl >= 0; cl--) begin
            for (int c = NumCores - 1; c >= 0; c--) begin
                if (fenced_i[c][cl] && !retired_q[c][cl]) begin
                    pending_found   = 1'b1;
                    pending_core    = CoreIdWidth'(c);
                    pending_cluster = ClusterIdWidth'(cl);
                end
            end
        end
    end

    // Substitute for the head of the slot being moved: the logical core itself
    // if it is live, otherwise the lowest live core allowed by AllowMask.
    logic [CoreIdWidth-1:0] head_logical;
    logic                   dst_found;
    logic [CoreIdWidth-1:0] dst_core;

    assign head_logical = checkout_logical_core_i[src_core_q][src_cluster_q];

    always_comb begin
        dst_found = 1'b0;
        dst_core  = '0;
        for (int c = NumCores - 1; c >= 0; c--) begin
            if (((c == int'(head_logical)) || AllowMask[head_logical][c]) &&
                !fenced_i[c][src_cluster_q]) begin
                dst_found = 1'b1;
                dst_core  = CoreIdWidth'(c);
            end
        end
    end

    logic head_no_exec;
    logic dst_space;

    assign head_no_exec = checkout_no_exec_i[src_core_q][src_cluster_q];
    assign dst_space    = !checkout_full_i[dst_core][src_cluster_q] &&
                          (head_no_exec || !ready_full_i[dst_core][src_cluster_q]);

    always_comb begin
        state_d       = state_q;
        src_core_d    = src_core_q;
        src_cluster_d = src_cluster_q;
        retired_d     = retired_q;

        ready_flush_o = '0;
        move_o        = '0;
        move_fire_o   = 1'b0;
        push_ready_o  = 1'b0;
        stuck_o       = 1'b0;

        case (state_q)
            IDLE: begin
                if (pending_found) begin
                    src_core_d    = pending_core;
                    src_cluster_d = pending_cluster;
                    state_d       = DRAIN;
                end
            end
            DRAIN: begin
                ready_flush_o[src_core_q][src_cluster_q] = 1'b1;
                if (done_q_empty_i[src_core_q][src_cluster_q]) begin
                    state_d = MOVE;
                end
            end
            MOVE: begin
                move_o[src_core_q][src_cluster_q] = 1'b1;
                if (checkout_empty_i[src_core_q][src_cluster_q]) begin
                    state_d = FINISH;
                end else if (!dst_found) begin
                    stuck_o = 1'b1;
                end else if (dst_space) begin
                    move_fire_o  = 1'b1;
                    push_ready_o = !head_no_exec;
                end
            end
            FINISH: begin
                retired_d[src_core_q][src_cluster_q] = 1'b1;
                state_d = IDLE;
            end
            default: state_d = IDLE;
        endcase
    end

    assign retired_o     = retired_q;
    assign src_core_o    = src_core_q;
    assign src_cluster_o = src_cluster_q;
    assign dst_core_o    = dst_core;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q       <= IDLE;
            src_core_q    <= '0;
            src_cluster_q <= '0;
            retired_q     <= '0;
        end else begin
            state_q       <= state_d;
            src_core_q    <= src_core_d;
            src_cluster_q <= src_cluster_d;
            retired_q     <= retired_d;
        end
    end

endmodule
