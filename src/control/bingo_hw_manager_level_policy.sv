// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level requests of the control plane (bingo_hw_manager_ctrl) to the power
// manager: which slots ask for the boost level, and which for the derate level.
//
// Recovery boost: a live slot that is the substitute of a fenced core in the
// slot mapping table does the work of two cores. While it is busy (not
// polling), pm_boost_o asks the PM to run its domain at the boost level
// (the PM's boost_level, a faster clock than the normal level; 0 = off).
// Capacity boost (cfg_i.boost_policy = 1) replaces that choice: a core type
// has lost capacity while one of its slots is fenced, stuck, rejected or
// parked at risk. Every busy live slot of such a type with a load of at least
// cfg_i.boost_load_min is a candidate (type 0 substitutes for nobody and is
// never one; neither is a slot in a derated domain). At most
// cfg_i.boost_credit domains per lost slot run boosted (0 = no limit), the
// lowest domain index first: the dead or parked cores no longer draw power,
// and the survivors of their type get that budget.
//
// Derate (cfg_i.risk_action[1]): an at-risk slot (bingo_hw_manager_risk) that
// is not parked asks the PM to run its domain no faster than the derate level
// (pm_derate_o); with the park action as well (cfg_i.risk_action[0]), only
// once parking has failed.
module bingo_hw_manager_level_policy
    import bingo_hw_manager_pkg::*;
#(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    parameter bit          ParkSupported = 1'b1,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         slot_state_t = logic,
    parameter type         ctrl_queue_t = logic,
    parameter type         ctrl_host_t = logic,
    parameter type         smt_t = logic,
    parameter type         park_t = logic
) (
    // Slot state (fenced, stuck, rejected); queues (waiting, load); power
    // domain of each slot (host)
    input  slot_state_t                          slot_i,
    input  ctrl_queue_t                          queue_i,
    input  ctrl_host_t                           host_i,
    // Slot mapping table, parking, at-risk slots
    input  smt_t                                 smt_i,
    input  park_t                                park_i,
    input  logic [NumCores-1:0][NumClusters-1:0] risk_i,
    // Host: risk action, boost policy, credit and minimum load
    input  ctrl_cfg_t                            cfg_i,

    output logic [NumCores-1:0][NumClusters-1:0] pm_boost_o,
    output logic [NumCores-1:0][NumClusters-1:0] pm_derate_o
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
                if (slot_i.fenced[c][cl] && smt_i.found[c][cl] &&
                    ((smt_i.core[c][cl] != c) || (smt_i.cluster[c][cl] != cl))) begin
                    boost_sub[smt_i.core[c][cl]][smt_i.cluster[c][cl]] = 1'b1;
                end
            end
        end
        boost_sub = boost_sub & ~queue_i.waiting & ~slot_i.fenced;

        lost_slot = slot_i.fenced | slot_i.stuck | slot_i.rejected | (risk_i & park_i.parked);
        type_lost = '0;
        n_lost    = '0;
        domain_derate = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (lost_slot[c][cl]) begin
                    n_lost = n_lost + 1;
                    if (CoreTypeId[c][cl] != '0) type_lost[CoreTypeId[c][cl]] = 1'b1;
                end
                if (pm_derate_o[c][cl] && (host_i.domain[c][cl] < 6'd32))
                    domain_derate[host_i.domain[c][cl][4:0]] = 1'b1;
            end
        end
        boost_cand  = '0;
        domain_cand = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (!queue_i.waiting[c][cl] && !lost_slot[c][cl] && !park_i.parked[c][cl] &&
                    (CoreTypeId[c][cl] != '0) && type_lost[CoreTypeId[c][cl]] &&
                    (32'(queue_i.load[c][cl]) >= 32'(cfg_i.boost_load_min)) && (host_i.domain[c][cl] < 6'd32)) begin
                    boost_cand[c][cl] = 1'b1;
                    domain_cand[host_i.domain[c][cl][4:0]] = 1'b1;
                end
            end
        end
        // Derate applies to the whole domain. Do not spend a boost credit on
        // a healthy candidate sharing that domain with an at-risk slot.
        domain_cand = domain_cand & ~domain_derate;
        budget      = 12'(cfg_i.boost_credit) * 12'(n_lost);
        picked      = '0;
        domain_pick = '0;
        for (int unsigned d = 0; d < 32; d++) begin
            if (domain_cand[d] && ((cfg_i.boost_credit == '0) || (12'(picked) < budget))) begin
                domain_pick[d] = 1'b1;
                picked = picked + 1;
            end
        end

        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                pm_boost_o[c][cl] = cfg_i.boost_policy ?
                    (boost_cand[c][cl] && domain_pick[host_i.domain[c][cl][4:0]]) : boost_sub[c][cl];
            end
        end
    end

    // ------------------------------------------------------------------
    // Derate: at-risk slots that are not parked (with parking on: once it failed)
    // ------------------------------------------------------------------
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                pm_derate_o[c][cl] = risk_i[c][cl] && cfg_i.risk_action[1] && !slot_i.fenced[c][cl] &&
                                     !park_i.parked[c][cl] &&
                                     !(cfg_i.risk_action[0] && ParkSupported && !park_i.fail[c][cl]);
            end
        end
    end

endmodule
