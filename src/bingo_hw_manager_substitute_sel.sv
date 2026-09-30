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
// order; within a cluster the lowest core wins. The choice only depends on the
// set of fenced slots, and the
// remap selector (new tasks) and the replay controller (outstanding tasks) both
// use this module, so all tasks of one dead core go to the same substitute.
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
    parameter logic [2:0] LevelMask = 3'b001
) (
    input  logic [CoreIdWidth-1:0]               logical_core_i,
    input  logic [ClusterIdWidth-1:0]            logical_cluster_i,
    input  logic [NumCores-1:0][NumClusters-1:0] fenced_i,
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

    // Scan downwards so that the lowest index wins, and the logical cluster last
    // so that it wins over the other clusters.
    always_comb begin
        found_o   = 1'b0;
        core_o    = '0;
        cluster_o = '0;
        for (int cl = NumClusters - 1; cl >= 0; cl--) begin
            for (int c = NumCores - 1; c >= 0; c--) begin
                if (candidate[c][cl]) begin
                    found_o   = 1'b1;
                    core_o    = CoreIdWidth'(c);
                    cluster_o = ClusterIdWidth'(cl);
                end
            end
        end
        for (int c = NumCores - 1; c >= 0; c--) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                if ((cl == int'(logical_cluster_i)) && candidate[c][cl]) begin
                    found_o   = 1'b1;
                    core_o    = CoreIdWidth'(c);
                    cluster_o = ClusterIdWidth'(cl);
                end
            end
        end
    end

endmodule
