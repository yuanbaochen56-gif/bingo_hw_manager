// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Substitute selector: the live slot that runs the tasks of logical core
// (logical_core_i, logical_cluster_i) once that core is fenced.
//
// Candidates are the logical slot itself and every slot of the chiplet with the
// same non-zero CoreTypeId that LevelMask allows, minus the fenced slots:
//   LevelMask[0] (level 1) slots of the logical cluster
//   LevelMask[1] (level 2) slots of the other clusters of the chiplet
// The logical cluster is searched first, then the other clusters in index
// order; within a cluster the lowest core wins. With LeastWeight the candidate
// with the lowest weight_i wins instead (e.g. its queue occupancy), still in the
// logical cluster first, the lowest index on a tie. bingo_hw_manager_ctrl keeps
// the choice in its slot mapping table, so all tasks of one dead core go to the
// same substitute even when the weights change.
module bingo_hw_manager_substitute_sel #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // Type of each (core, cluster) slot (see bingo_hw_manager_top)
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    // Substitute levels (see bingo_hw_manager_top SubstituteLevelMask); bit 2
    // (remote chiplet) is not handled here
    parameter logic [2:0] LevelMask = 3'b001,
    // Among the candidates of a level: lowest index (0) or lowest weight_i (1)
    parameter bit          LeastWeight = 1'b0,
    parameter int unsigned WeightWidth = 1
) (
    input  logic [CoreIdWidth-1:0]               logical_core_i,
    input  logic [ClusterIdWidth-1:0]            logical_cluster_i,
    input  logic [NumCores-1:0][NumClusters-1:0] fenced_i,
    input  logic [NumCores-1:0][NumClusters-1:0][WeightWidth-1:0] weight_i = '0,
    output logic                                 found_o,
    output logic [CoreIdWidth-1:0]               core_o,
    output logic [ClusterIdWidth-1:0]            cluster_o
);
    logic                                 logical_in_range;
    logic [CoreTypeIdWidth-1:0]           logical_type;
    logic [NumCores-1:0][NumClusters-1:0] candidate;

    assign logical_in_range = (int'(logical_core_i) < NumCores) &&
                              (int'(logical_cluster_i) < NumClusters);
    assign logical_type     = logical_in_range ? CoreTypeId[logical_core_i][logical_cluster_i] : '0;

    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                candidate[c][cl] = logical_in_range && !fenced_i[c][cl] &&
                    (((c == int'(logical_core_i)) && (cl == int'(logical_cluster_i))) ||
                     ((logical_type != '0) && (CoreTypeId[c][cl] == logical_type) &&
                      ((cl == int'(logical_cluster_i)) ? LevelMask[0] : LevelMask[1])));
            end
        end
    end

    // Level 1 (the logical cluster) first, then level 2 (the other clusters in
    // index order). Scanning upwards, a candidate replaces the current choice
    // only if nothing was found yet or (LeastWeight) its weight is lower, so
    // the lowest index wins among equals.
    logic [WeightWidth-1:0] best_weight;
    always_comb begin
        found_o     = 1'b0;
        core_o      = '0;
        cluster_o   = '0;
        best_weight = '0;
        for (int unsigned level = 1; level <= 2; level++) begin
            if (!found_o) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    for (int unsigned c = 0; c < NumCores; c++) begin
                        if (candidate[c][cl] && ((cl == int'(logical_cluster_i)) == (level == 1)) &&
                            (!found_o || (LeastWeight && (weight_i[c][cl] < best_weight)))) begin
                            found_o     = 1'b1;
                            core_o      = CoreIdWidth'(c);
                            cluster_o   = ClusterIdWidth'(cl);
                            best_weight = weight_i[c][cl];
                        end
                    end
                end
            end
        end
    end

endmodule
