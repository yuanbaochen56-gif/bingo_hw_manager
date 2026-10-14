// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level requests of the control plane (bingo_hw_manager_ctrl) to the power
// manager: which slots ask for the boost level, and which for the derate level.
//
// Recovery boost: a live slot that is the substitute of a fenced core in the
// slot mapping table does the work of two cores. While it is busy (not
// polling), pm_boost_o asks the PM to run its domain at the boost level
// (bingo_hw_manager_pm boost_power_level_i, a faster clock than the normal
// level; 0 = off).
// Capacity boost (boost_policy_i = 1) replaces that choice: a core type has
// lost capacity while one of its slots is fenced, stuck, rejected or parked at
// risk. Every busy live slot of such a type with a load of at least
// boost_load_min_i is a candidate (type 0 substitutes for nobody and is never
// one; neither is a slot in a derated domain). At most boost_credit_i domains
// per lost slot run boosted
// (0 = no limit), the lowest domain index first: the dead or parked cores no
// longer draw power, and the survivors of their type get that budget.
//
// Derate (risk_action_i[1]): an at-risk slot (bingo_hw_manager_risk) that is
// not parked asks the PM to run its domain no faster than the derate level
// (pm_derate_o); with the park action as well (risk_action_i[0]), only once
// parking has failed.
module bingo_hw_manager_level_policy #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // Type of each (core, cluster) slot (see bingo_hw_manager_top)
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    // Width of load_i
    parameter int unsigned LoadWidth = 4,
    // Parking works (the CSR ready queues); otherwise an at-risk slot is not parked
    parameter bit          ParkSupported = 1'b1
) (
    // Watchdog: confirmed dead slots (sticky)
    input  logic [NumCores-1:0][NumClusters-1:0]                     fenced_i,
    // Slots polling their ready queue (idle)
    input  logic [NumCores-1:0][NumClusters-1:0]                     waiting_i,
    // Load of every slot
    input  logic [NumCores-1:0][NumClusters-1:0][LoadWidth-1:0]      load_i,
    // Power domain of every slot (>= 32: no domain)
    input  logic [NumCores-1:0][NumClusters-1:0][5:0]                slot_domain_i,
    // Stuck (fenced, nowhere to replay) or rejected by a remote chiplet
    input  logic [NumCores-1:0][NumClusters-1:0]                     stuck_i,
    input  logic [NumCores-1:0][NumClusters-1:0]                     rejected_i,
    // Slot mapping table, indexed by logical slot, and the parking state
    input  logic [NumCores-1:0][NumClusters-1:0]                     smt_found_i,
    input  logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_i,
    input  logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_i,
    input  logic [NumCores-1:0][NumClusters-1:0]                     park_parked_i,
    input  logic [NumCores-1:0][NumClusters-1:0]                     park_fail_i,
    // At-risk slots and the actions (bit 0 park, bit 1 derate)
    input  logic [NumCores-1:0][NumClusters-1:0]                     risk_i,
    input  logic [1:0]                                               risk_action_i,
    // Boost choice: 0 = substitutes of fenced cores, 1 = capacity; boosted
    // domains per lost slot (0 = no limit); minimum load of a candidate
    input  logic                                                     boost_policy_i,
    input  logic [3:0]                                               boost_credit_i,
    input  logic [7:0]                                               boost_load_min_i,

    // Power manager: slots that run a dead core's tasks and are busy
    output logic [NumCores-1:0][NumClusters-1:0]                     pm_boost_o,
    // Slots whose domain runs at most at the derate level
    output logic [NumCores-1:0][NumClusters-1:0]                     pm_derate_o
);

    // ------------------------------------------------------------------
    // Recovery boost: substitutes of fenced cores, or capacity (see above)
    // ------------------------------------------------------------------
    logic [NumCores-1:0][NumClusters-1:0] boost_sub, boost_cand, lost_slot;
    logic [2**CoreTypeIdWidth-1:0]        type_lost;
    logic [31:0]                          domain_cand, domain_pick, domain_derate;
    logic [7:0]                           n_lost, picked;
    logic [11:0]                          budget;
    always_comb begin
        boost_sub = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (fenced_i[c][cl] && smt_found_i[c][cl] &&
                    ((smt_core_i[c][cl] != c) || (smt_cluster_i[c][cl] != cl))) begin
                    boost_sub[smt_core_i[c][cl]][smt_cluster_i[c][cl]] = 1'b1;
                end
            end
        end
        boost_sub = boost_sub & ~waiting_i & ~fenced_i;

        lost_slot = fenced_i | stuck_i | rejected_i | (risk_i & park_parked_i);
        type_lost = '0;
        n_lost    = '0;
        domain_derate = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (lost_slot[c][cl]) begin
                    n_lost = n_lost + 1;
                    if (CoreTypeId[c][cl] != '0) type_lost[CoreTypeId[c][cl]] = 1'b1;
                end
                if (pm_derate_o[c][cl] && (slot_domain_i[c][cl] < 6'd32))
                    domain_derate[slot_domain_i[c][cl][4:0]] = 1'b1;
            end
        end
        boost_cand  = '0;
        domain_cand = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (!waiting_i[c][cl] && !lost_slot[c][cl] && !park_parked_i[c][cl] &&
                    (CoreTypeId[c][cl] != '0) && type_lost[CoreTypeId[c][cl]] &&
                    (32'(load_i[c][cl]) >= 32'(boost_load_min_i)) && (slot_domain_i[c][cl] < 6'd32)) begin
                    boost_cand[c][cl] = 1'b1;
                    domain_cand[slot_domain_i[c][cl][4:0]] = 1'b1;
                end
            end
        end
        // Derate applies to the whole domain. Do not spend a boost credit on
        // a healthy candidate sharing that domain with an at-risk slot.
        domain_cand = domain_cand & ~domain_derate;
        budget      = 12'(boost_credit_i) * 12'(n_lost);
        picked      = '0;
        domain_pick = '0;
        for (int unsigned d = 0; d < 32; d++) begin
            if (domain_cand[d] && ((boost_credit_i == '0) || (12'(picked) < budget))) begin
                domain_pick[d] = 1'b1;
                picked = picked + 1;
            end
        end

        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                pm_boost_o[c][cl] = boost_policy_i ?
                    (boost_cand[c][cl] && domain_pick[slot_domain_i[c][cl][4:0]]) : boost_sub[c][cl];
            end
        end
    end

    // ------------------------------------------------------------------
    // Derate: at-risk slots that are not parked (with parking on: once it failed)
    // ------------------------------------------------------------------
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                pm_derate_o[c][cl] = risk_i[c][cl] && risk_action_i[1] && !fenced_i[c][cl] &&
                                     !park_parked_i[c][cl] &&
                                     !(risk_action_i[0] && ParkSupported && !park_fail_i[c][cl]);
            end
        end
    end

endmodule
