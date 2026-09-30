// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Control plane of the HW manager: the per-slot state that the placement,
// replay and power policies share, and the decisions that move work between
// cores. The mechanisms (queues, replay engine, PM bus master, CERF register
// file, remote link) stay in their own modules and only read what this module
// decides.
//
// Slot mapping table (SMT): for every logical slot L = (core, cluster), the live
// slot that runs L's tasks once L is fenced (its substitute), or none
// (smt_found_o = 0). bingo_hw_manager_core_remap (new tasks) and
// bingo_hw_manager_replay_ctrl (outstanding tasks) both read it, so all tasks of
// one dead core go to the same substitute, in order.
//
// The table is a register. It is filled once after reset and then only changes
// when a slot is fenced: an entry is recomputed if its logical slot or its
// current substitute was just fenced (bingo_hw_manager_substitute_sel over the
// new fenced set); every other entry keeps its value. With the default choice
// (the lowest live slot of the same type, logical cluster first) removing a slot
// that is not the chosen one never changes a choice, so the table always equals
// the combinational choice of bingo_hw_manager_substitute_sel (checked in
// simulation). Keeping the choice in a register is what later lets a policy use
// information that changes over time (load, power level) without moving a dead
// core's tasks to a different substitute half-way.
//
// In the cycle a slot is fenced (and the cycle after reset) the table is being
// written: smt_update_o asks its readers to wait one cycle.
module bingo_hw_manager_ctrl #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // Type of each (core, cluster) slot (see bingo_hw_manager_top)
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    // Substitute levels (see bingo_hw_manager_top SubstituteLevelMask)
    parameter logic [2:0] SubstituteLevelMask = 3'b001
) (
    input  logic clk_i,
    input  logic rst_ni,

    // Watchdog: confirmed dead slots (sticky)
    input  logic [NumCores-1:0][NumClusters-1:0] fenced_i,

    // Slot mapping table, indexed by logical slot
    output logic [NumCores-1:0][NumClusters-1:0]                     smt_found_o,
    output logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_o,
    output logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_o,
    // The table is written this cycle: its readers hold
    output logic                                                     smt_update_o
);

    // ------------------------------------------------------------------
    // Candidate choice per logical slot over the current fenced set
    // ------------------------------------------------------------------
    logic [NumCores-1:0][NumClusters-1:0]                     choice_found;
    logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    choice_core;
    logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] choice_cluster;

    for (genvar c = 0; c < NumCores; c++) begin : gen_choice_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_choice_cluster
            bingo_hw_manager_substitute_sel #(
                .NumCores(NumCores),
                .NumClusters(NumClusters),
                .CoreIdWidth(CoreIdWidth),
                .ClusterIdWidth(ClusterIdWidth),
                .CoreTypeIdWidth(CoreTypeIdWidth),
                .CoreTypeId(CoreTypeId),
                .LevelMask(SubstituteLevelMask)
            ) i_choice (
                .logical_core_i(CoreIdWidth'(c)),
                .logical_cluster_i(ClusterIdWidth'(cl)),
                .fenced_i(fenced_i),
                .found_o(choice_found[c][cl]),
                .core_o(choice_core[c][cl]),
                .cluster_o(choice_cluster[c][cl])
            );
        end
    end

    // ------------------------------------------------------------------
    // Table update: only entries touched by a new fence
    // ------------------------------------------------------------------
    logic                                                     init_q;
    logic [NumCores-1:0][NumClusters-1:0]                     fenced_seen_q;
    logic [NumCores-1:0][NumClusters-1:0]                     fence_new;
    logic [NumCores-1:0][NumClusters-1:0]                     recompute;
    logic [NumCores-1:0][NumClusters-1:0]                     smt_found_q;
    logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_q;
    logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_q;

    assign fence_new = fenced_i & ~fenced_seen_q;

    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                recompute[c][cl] = init_q || fence_new[c][cl] ||
                    (smt_found_q[c][cl] && fence_new[smt_core_q[c][cl]][smt_cluster_q[c][cl]]);
            end
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            init_q        <= 1'b1;
            fenced_seen_q <= '0;
            smt_found_q   <= '0;
            smt_core_q    <= '0;
            smt_cluster_q <= '0;
        end else begin
            init_q        <= 1'b0;
            fenced_seen_q <= fenced_i;
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (recompute[c][cl]) begin
                        smt_found_q[c][cl]   <= choice_found[c][cl];
                        smt_core_q[c][cl]    <= choice_core[c][cl];
                        smt_cluster_q[c][cl] <= choice_cluster[c][cl];
                    end
                end
            end
        end
    end

    assign smt_found_o   = smt_found_q;
    assign smt_core_o    = smt_core_q;
    assign smt_cluster_o = smt_cluster_q;
    assign smt_update_o  = init_q || (|fence_new);

`ifndef SYNTHESIS
    // Outside an update cycle the table equals the combinational choice
    always @(posedge clk_i) begin : smt_equivalence
        if (rst_ni && !smt_update_o) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if ((smt_found_q[c][cl] != choice_found[c][cl]) ||
                        (choice_found[c][cl] && ((smt_core_q[c][cl] != choice_core[c][cl]) ||
                                                 (smt_cluster_q[c][cl] != choice_cluster[c][cl])))) begin
                        $error("[BINGO_ASSERT] SMT entry core %0d cluster %0d: found %0b core %0d cluster %0d, choice %0b core %0d cluster %0d",
                               c, cl, smt_found_q[c][cl], smt_core_q[c][cl], smt_cluster_q[c][cl],
                               choice_found[c][cl], choice_core[c][cl], choice_cluster[c][cl]);
                    end
                end
            end
        end
    end
`endif

endmodule
