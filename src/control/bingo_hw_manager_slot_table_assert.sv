// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Simulation-only invariants of bingo_hw_manager_slot_table: the table against
// the combinational choice, the substitutes in it, and the end of parking on a
// fence. Each violation is an $error with the [BINGO_ASSERT] tag; nothing here
// drives the design. Instantiated by the slot table (see
// bingo_hw_manager_top_assert for why not with a bind statement).
`ifndef SYNTHESIS
module bingo_hw_manager_slot_table_assert #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    parameter int unsigned SubstitutePolicy = 0
) (
    input logic                                                     clk_i,
    input logic                                                     rst_ni,
    // Fenced slots, now and one cycle earlier; substitute choice from the host
    input logic [NumCores-1:0][NumClusters-1:0]                     fenced_i,
    input logic [NumCores-1:0][NumClusters-1:0]                     fenced_seen_i,
    input bingo_hw_manager_pkg::sub_cfg_t                           sub_cfg_i,
    // The table, the cycle it is written, and the entries written this cycle
    input logic [NumCores-1:0][NumClusters-1:0]                     smt_found_i,
    input logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_i,
    input logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_i,
    input logic                                                     smt_update_i,
    input logic [NumCores-1:0][NumClusters-1:0]                     recompute_i,
    input logic [NumCores-1:0][NumClusters-1:0]                     park_commit_i,
    input logic [NumCores-1:0][NumClusters-1:0]                     unpark_commit_i,
    // The combinational choice over the current fenced set
    input logic [NumCores-1:0][NumClusters-1:0]                     choice_found_i,
    input logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    choice_core_i,
    input logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] choice_cluster_i,
    // Parking state (parked: PARKED or UNPARK)
    input logic [NumCores-1:0][NumClusters-1:0]                     park_hold_i,
    input logic [NumCores-1:0][NumClusters-1:0]                     park_parked_i
);

    // Entries written before the last change of sub_cfg_i and not since: chosen
    // under the choice of before
    bingo_hw_manager_pkg::sub_cfg_t       sub_seen_q;
    logic [NumCores-1:0][NumClusters-1:0] sub_stale_q;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            sub_seen_q  <= sub_cfg_i;
            sub_stale_q <= '0;
        end else begin
            sub_seen_q <= sub_cfg_i;
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (recompute_i[c][cl] || park_commit_i[c][cl] || unpark_commit_i[c][cl]) sub_stale_q[c][cl] <= 1'b0;
                    else if (sub_cfg_i != sub_seen_q)                                        sub_stale_q[c][cl] <= 1'b1;
                end
            end
        end
    end

    // Outside an update cycle the table equals the combinational choice (only
    // for the lowest-index policy: a load-based choice depends on the load when
    // the core died; and not for an entry kept from before a change of sub_cfg_i)
    always @(posedge clk_i) begin : smt_equivalence
        if (rst_ni && !smt_update_i && (SubstitutePolicy == 0)) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (sub_stale_q[c][cl] || (sub_cfg_i != sub_seen_q)) continue;
                    // A parked entry was chosen among the slots that were neither in
                    // HOLD nor PARKED at the time, so it is only checked for validity
                    // (below and here: another slot, never itself, never parked).
                    if (park_parked_i[c][cl]) begin
                        if (!smt_found_i[c][cl] ||
                            ((smt_core_i[c][cl] == CoreIdWidth'(c)) && (smt_cluster_i[c][cl] == ClusterIdWidth'(cl))) ||
                            park_parked_i[smt_core_i[c][cl]][smt_cluster_i[c][cl]]) begin
                            $error("[BINGO_ASSERT] parked SMT entry core %0d cluster %0d: found %0b core %0d cluster %0d",
                                   c, cl, smt_found_i[c][cl], smt_core_i[c][cl], smt_cluster_i[c][cl]);
                        end
                    end else if (!fenced_i[c][cl] && smt_found_i[c][cl] &&
                                 (smt_core_i[c][cl] == CoreIdWidth'(c)) && (smt_cluster_i[c][cl] == ClusterIdWidth'(cl))) begin
                        // A park that fell back to the slot itself (its substitute died
                        // and none was left): the replay of its old tasks went there.
                        // A live slot's entry is not read until it is fenced or parked,
                        // and both write it again.
                    end else if ((smt_found_i[c][cl] != choice_found_i[c][cl]) ||
                        (choice_found_i[c][cl] && ((smt_core_i[c][cl] != choice_core_i[c][cl]) ||
                                                   (smt_cluster_i[c][cl] != choice_cluster_i[c][cl])))) begin
                        $error("[BINGO_ASSERT] SMT entry core %0d cluster %0d: found %0b core %0d cluster %0d, choice %0b core %0d cluster %0d",
                               c, cl, smt_found_i[c][cl], smt_core_i[c][cl], smt_cluster_i[c][cl],
                               choice_found_i[c][cl], choice_core_i[c][cl], choice_cluster_i[c][cl]);
                    end
                end
            end
        end
    end

    // Every policy: a substitute in the table is live and of the logical slot's type
    always @(posedge clk_i) begin : smt_validity
        if (rst_ni && !smt_update_i) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (smt_found_i[c][cl] &&
                        (fenced_i[smt_core_i[c][cl]][smt_cluster_i[c][cl]] ||
                         (((smt_core_i[c][cl] != c) || (smt_cluster_i[c][cl] != cl)) &&
                          ((CoreTypeId[c][cl] == '0) ||
                           (CoreTypeId[smt_core_i[c][cl]][smt_cluster_i[c][cl]] != CoreTypeId[c][cl]))))) begin
                        $error("[BINGO_ASSERT] SMT entry core %0d cluster %0d -> core %0d cluster %0d is fenced or of another type",
                               c, cl, smt_core_i[c][cl], smt_cluster_i[c][cl]);
                    end
                    // Parking ends on the edge that sees the fence, so a slot fenced
                    // since the previous edge must no longer be in HOLD or PARKED.
                    if ((park_hold_i[c][cl] || park_parked_i[c][cl]) && fenced_seen_i[c][cl]) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d park hold %0b parked %0b while fenced %0b",
                               c, cl, park_hold_i[c][cl], park_parked_i[c][cl], fenced_i[c][cl]);
                    end
                end
            end
        end
    end

endmodule
`endif
