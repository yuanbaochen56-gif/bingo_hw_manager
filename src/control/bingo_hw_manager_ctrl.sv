// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Control plane of the HW manager: the per-slot state that the placement,
// replay and power policies share, and the decisions that move work between
// cores. The mechanisms (queues, replay engine, PM bus master, CERF register
// file, remote link) stay in their own modules and only read what this module
// decides. The rules are in its submodules:
//   bingo_hw_manager_slot_table     slot mapping table (the substitute of every
//                                   logical slot) and core parking
//   bingo_hw_manager_risk           fault precursors: late-beat counts and
//                                   at-risk slots
//   bingo_hw_manager_power_view     which slots the PM may count as idle, which
//                                   count as load, and the recovery hold
//   bingo_hw_manager_wd_tick        watchdog ticks in cycles of the normal clock
//   bingo_hw_manager_level_policy   boost and derate requests to the PM
//   bingo_hw_manager_cerf_fallback  CERF degradation of a core type that lost
//                                   a slot (stuck or rejected)
// This module joins them and keeps the simulation prints of the control plane
// ([BINGO_RECOVERY_HOLD], [BINGO_LATE], [BINGO_RISK], [BINGO_CERF_FB]).
//
// Fault precursors and their actions: a slot whose late beats (late_i) add up
// faster than they decay is at risk until the host clears it (risk_clear_i) or
// the slot is fenced.
// risk_action_i[0]: an at-risk slot is parked as if the host had asked for it
// (same path, same park_fail_o); clearing the risk moves it back.
// risk_action_i[1]: an at-risk slot that is not parked asks the PM to run its
// domain no faster than the derate level (pm_derate_o); with both bits, only
// once parking has failed. Without parking support (ParkSupported = 0) an
// at-risk slot is never parked. An at-risk slot may still be picked as the
// substitute of a fenced one: the fence path is unchanged.
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
    parameter logic [2:0] SubstituteLevelMask = 3'b001,
    parameter logic [2**CoreTypeIdWidth-1:0] SubstituteL2TypeEn = '1,
    // Substitute choice (see bingo_hw_manager_slot_table) and width of load_i
    parameter int unsigned SubstitutePolicy = 0,
    parameter int unsigned LoadWidth = 4,
    // Parking works (the CSR ready queues); otherwise an at-risk slot is not parked
    parameter bit          ParkSupported = 1'b1
) (
    input  logic clk_i,
    input  logic rst_ni,

    // Watchdog: confirmed dead slots (sticky)
    input  logic [NumCores-1:0][NumClusters-1:0] fenced_i,
    // Slots polling their ready queue (idle)
    input  logic [NumCores-1:0][NumClusters-1:0] waiting_i,
    // Load of every slot (SubstitutePolicy 1)
    input  logic [NumCores-1:0][NumClusters-1:0][LoadWidth-1:0] load_i,
    // Power state (bingo_hw_manager_pm): enable, mode, levels, slot -> domain
    input  logic                                     pm_enable_i,
    input  logic                                     pm_dvfs_i,
    input  logic [7:0]                               normal_level_i,
    input  logic [7:0]                               dvfs_level_i,
    input  logic [31:0][7:0]                         domain_level_i,
    input  logic [NumCores-1:0][NumClusters-1:0][5:0] slot_domain_i,  // >= 32: no domain
    // Cycles a slot must be idle before the PM may count it as idle (0: at once)
    input  logic [31:0]                              idle_delay_i,
    // External access to a cluster's memory this cycle, and the cycles its
    // slots stay awake after the last one (0: off)
    input  logic [NumClusters-1:0]                   cluster_access_i,
    input  logic [31:0]                              access_hold_i,
    // Recovery hold in top-clock cycles, 32 bits (0: off, max 2^32-1).
    input  logic [31:0]                              recovery_hold_i = '0,
    input  logic [NumCores-1:0][NumClusters-1:0]     retired_i = '0,
    // Host park request, checkout / ready empty, and a push into the slot
    // this cycle (route, replay, or import). See bingo_hw_manager_slot_table.
    input  logic [NumCores-1:0][NumClusters-1:0]     park_req_i = '0,
    input  logic [NumCores-1:0][NumClusters-1:0]     checkout_empty_i = '1,
    input  logic [NumCores-1:0][NumClusters-1:0]     ready_empty_i = '1,
    input  logic [NumCores-1:0][NumClusters-1:0]     slot_push_i = '0,
    // Logical slots with tasks in another core's checkout queue (UNPARK waits)
    input  logic [NumCores-1:0][NumClusters-1:0]     moved_i = '0,
    // Stuck (fenced, nowhere to replay) or rejected by a remote chiplet
    input  logic [NumCores-1:0][NumClusters-1:0]     stuck_i = '0,
    input  logic [NumCores-1:0][NumClusters-1:0]     rejected_i = '0,
    // Per core type: enable, CERF group to clear, CERF group to set. 0 = off.
    input  logic [2**CoreTypeIdWidth-1:0]            cerf_fb_en_i = '0,
    // Fence fallback: core types that take no substitute (see
    // bingo_hw_manager_slot_table). 0 = off.
    input  logic [2**CoreTypeIdWidth-1:0]            fence_fb_i = '0,
    input  logic [2**CoreTypeIdWidth-1:0][4:0]       cerf_fb_clear_i = '0,
    input  logic [2**CoreTypeIdWidth-1:0][4:0]       cerf_fb_set_i = '0,
    // The CERF update committed this cycle (host did not write the mask)
    input  logic                                     cerf_fb_done_i = 1'b0,
    // Fault precursors (see above): late beats, count threshold (0 = off),
    // actions (bit 0 park, bit 1 derate), halving period (0 = never), host clear
    input  logic [NumCores-1:0][NumClusters-1:0]     late_i = '0,
    input  logic [3:0]                               risk_thresh_i = '0,
    input  logic [1:0]                               risk_action_i = '0,
    input  logic [31:0]                              risk_epoch_i = '0,
    input  logic [NumCores-1:0][NumClusters-1:0]     risk_clear_i = '0,
    // Boost choice (bingo_hw_manager_level_policy): 0 = substitutes of fenced cores, 1 = capacity;
    // boosted domains per lost slot (0 = no limit); minimum load of a candidate
    input  logic                                     boost_policy_i = 1'b0,
    input  logic [3:0]                               boost_credit_i = '0,
    input  logic [7:0]                               boost_load_min_i = '0,

    // Power manager: slots that do not keep their domain at the normal level
    output logic [NumCores-1:0][NumClusters-1:0] pm_idle_o,
    // Post-edge recovery activity for the event logger (not a PM input).
    output logic [NumCores-1:0][NumClusters-1:0] rec_hold_o,
    // Slots otherwise PM-idle, awake only because their cluster is accessed.
    output logic [NumCores-1:0][NumClusters-1:0] access_only_o,
    // Load monitor: slots whose pending count is cleared
    output logic [NumCores-1:0][NumClusters-1:0] load_clear_o,
    // Watchdog: the slot's timer advances this cycle
    output logic [NumCores-1:0][NumClusters-1:0] wd_tick_o,
    // Adaptive bounds: the slot's domain runs faster than the normal level
    output logic [NumCores-1:0][NumClusters-1:0] wd_fast_o,
    // Power manager: slots that run a dead core's tasks and are busy
    output logic [NumCores-1:0][NumClusters-1:0] pm_boost_o,

    // Slot mapping table, indexed by logical slot
    output logic [NumCores-1:0][NumClusters-1:0]                     smt_found_o,
    output logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_o,
    output logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_o,
    // The table is written this cycle: its readers hold
    output logic                                                     smt_update_o,
    // Parking (bingo_hw_manager_slot_table). fail stays set until that slot's request drops.
    output logic [NumCores-1:0][NumClusters-1:0]     park_hold_o,
    output logic [NumCores-1:0][NumClusters-1:0]     park_parked_o,
    // PARKED slots moving back (new tasks held until moved_i drops)
    output logic [NumCores-1:0][NumClusters-1:0]     park_unpark_o,
    output logic [NumCores-1:0][NumClusters-1:0]     park_fail_o,

    // One CERF degradation, held until cerf_fb_done_i
    output logic                                     cerf_fb_req_o,
    output logic [CoreTypeIdWidth-1:0]               cerf_fb_type_o,
    output logic [4:0]                               cerf_fb_clear_o,
    output logic [4:0]                               cerf_fb_set_o,
    // Types whose degradation has fired; sticky until that enable drops
    output logic [2**CoreTypeIdWidth-1:0]            cerf_fb_evt_o,
    // At-risk slots, and the ones whose domain runs at most at the derate level
    output logic [NumCores-1:0][NumClusters-1:0]     risk_o,
    output logic [NumCores-1:0][NumClusters-1:0]     pm_derate_o
);

    // ------------------------------------------------------------------
    // Fault precursors: late-beat count per slot, halved every epoch
    // ------------------------------------------------------------------
    // risk_cnt_q, risk_q and risk_epoch_end keep the names that the HeMAiA quad
    // controller and the testbenches read here (i_ctrl.risk_cnt_q, ...).
    logic [NumCores-1:0][NumClusters-1:0][3:0] risk_cnt_q;
    logic [NumCores-1:0][NumClusters-1:0]      risk_q;
    logic                                      risk_epoch_end;
    logic [NumCores-1:0][NumClusters-1:0]      park_req;

    bingo_hw_manager_risk #(
        .NumCores(NumCores),
        .NumClusters(NumClusters)
    ) i_risk (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .fenced_i(fenced_i),
        .late_i(late_i),
        .risk_thresh_i(risk_thresh_i),
        .risk_epoch_i(risk_epoch_i),
        .risk_clear_i(risk_clear_i),
        .risk_o(risk_q),
        .risk_cnt_o(risk_cnt_q),
        .risk_epoch_end_o(risk_epoch_end)
    );

    // Host request, or an at-risk slot with the park action
    assign park_req = park_req_i | (risk_q & {(NumCores * NumClusters){risk_action_i[0] && ParkSupported}});
    assign risk_o = risk_q;

    // ------------------------------------------------------------------
    // Slot mapping table and core parking
    // ------------------------------------------------------------------
    logic [NumCores-1:0][NumClusters-1:0] smt_recompute;

    bingo_hw_manager_slot_table #(
        .NumCores(NumCores),
        .NumClusters(NumClusters),
        .CoreIdWidth(CoreIdWidth),
        .ClusterIdWidth(ClusterIdWidth),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreTypeId(CoreTypeId),
        .SubstituteLevelMask(SubstituteLevelMask),
        .SubstituteL2TypeEn(SubstituteL2TypeEn),
        .SubstitutePolicy(SubstitutePolicy),
        .LoadWidth(LoadWidth)
    ) i_slot_table (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .fenced_i(fenced_i),
        .load_i(load_i),
        .fence_fb_i(fence_fb_i),
        .park_req_i(park_req),
        .checkout_empty_i(checkout_empty_i),
        .ready_empty_i(ready_empty_i),
        .slot_push_i(slot_push_i),
        .moved_i(moved_i),
        .smt_found_o(smt_found_o),
        .smt_core_o(smt_core_o),
        .smt_cluster_o(smt_cluster_o),
        .smt_update_o(smt_update_o),
        .smt_recompute_o(smt_recompute),
        .park_hold_o(park_hold_o),
        .park_parked_o(park_parked_o),
        .park_unpark_o(park_unpark_o),
        .park_fail_o(park_fail_o)
    );

    // ------------------------------------------------------------------
    // Power and load view, recovery hold
    // ------------------------------------------------------------------
    bingo_hw_manager_power_view #(
        .NumCores(NumCores),
        .NumClusters(NumClusters),
        .CoreIdWidth(CoreIdWidth),
        .ClusterIdWidth(ClusterIdWidth)
    ) i_power_view (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .fenced_i(fenced_i),
        .waiting_i(waiting_i),
        .retired_i(retired_i),
        .smt_found_i(smt_found_o),
        .smt_core_i(smt_core_o),
        .smt_cluster_i(smt_cluster_o),
        .smt_recompute_i(smt_recompute),
        .idle_delay_i(idle_delay_i),
        .cluster_access_i(cluster_access_i),
        .access_hold_i(access_hold_i),
        .recovery_hold_i(recovery_hold_i),
        .pm_idle_o(pm_idle_o),
        .rec_hold_o(rec_hold_o),
        .access_only_o(access_only_o),
        .load_clear_o(load_clear_o)
    );

    // ------------------------------------------------------------------
    // Frequency-aware watchdog ticks
    // ------------------------------------------------------------------
    bingo_hw_manager_wd_tick #(
        .NumCores(NumCores),
        .NumClusters(NumClusters)
    ) i_wd_tick (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .pm_enable_i(pm_enable_i),
        .pm_dvfs_i(pm_dvfs_i),
        .normal_level_i(normal_level_i),
        .dvfs_level_i(dvfs_level_i),
        .domain_level_i(domain_level_i),
        .slot_domain_i(slot_domain_i),
        .wd_tick_o(wd_tick_o),
        .wd_fast_o(wd_fast_o)
    );

    // ------------------------------------------------------------------
    // Level requests to the PM: boost and derate
    // ------------------------------------------------------------------
    bingo_hw_manager_level_policy #(
        .NumCores(NumCores),
        .NumClusters(NumClusters),
        .CoreIdWidth(CoreIdWidth),
        .ClusterIdWidth(ClusterIdWidth),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreTypeId(CoreTypeId),
        .LoadWidth(LoadWidth),
        .ParkSupported(ParkSupported)
    ) i_level_policy (
        .fenced_i(fenced_i),
        .waiting_i(waiting_i),
        .load_i(load_i),
        .slot_domain_i(slot_domain_i),
        .stuck_i(stuck_i),
        .rejected_i(rejected_i),
        .smt_found_i(smt_found_o),
        .smt_core_i(smt_core_o),
        .smt_cluster_i(smt_cluster_o),
        .park_parked_i(park_parked_o),
        .park_fail_i(park_fail_o),
        .risk_i(risk_q),
        .risk_action_i(risk_action_i),
        .boost_policy_i(boost_policy_i),
        .boost_credit_i(boost_credit_i),
        .boost_load_min_i(boost_load_min_i),
        .pm_boost_o(pm_boost_o),
        .pm_derate_o(pm_derate_o)
    );

    // ------------------------------------------------------------------
    // CERF degradation
    // ------------------------------------------------------------------
    bingo_hw_manager_cerf_fallback #(
        .NumCores(NumCores),
        .NumClusters(NumClusters),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreTypeId(CoreTypeId)
    ) i_cerf_fallback (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .stuck_i(stuck_i),
        .rejected_i(rejected_i),
        .cerf_fb_en_i(cerf_fb_en_i),
        .cerf_fb_clear_i(cerf_fb_clear_i),
        .cerf_fb_set_i(cerf_fb_set_i),
        .cerf_fb_done_i(cerf_fb_done_i),
        .cerf_fb_req_o(cerf_fb_req_o),
        .cerf_fb_type_o(cerf_fb_type_o),
        .cerf_fb_clear_o(cerf_fb_clear_o),
        .cerf_fb_set_o(cerf_fb_set_o),
        .cerf_fb_evt_o(cerf_fb_evt_o)
    );

`ifndef SYNTHESIS
    always @(posedge clk_i) begin
        if (rst_ni) begin
            for (int c = 0; c < NumCores; c++)
                for (int cl = 0; cl < NumClusters; cl++)
                    if ((i_power_view.rec_hold_q[c][cl] != 0) != (i_power_view.rec_hold_d[c][cl] != 0))
                        $display("[BINGO_RECOVERY_HOLD] %0t core=%0d cluster=%0d active=%0b",
                                 $time, c, cl, i_power_view.rec_hold_d[c][cl] != 0);
        end
    end
    logic [NumCores-1:0][NumClusters-1:0] risk_seen_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            risk_seen_q <= '0;
        end else begin
            risk_seen_q <= risk_q;
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (late_i[c][cl])
                        $display("[BINGO_LATE] %0t core=%0d cluster=%0d", $time, c, cl);
                    if (risk_q[c][cl] && !risk_seen_q[c][cl])
                        $display("[BINGO_RISK] %0t core=%0d cluster=%0d at risk", $time, c, cl);
                end
            end
        end
    end
`endif

`ifndef SYNTHESIS
    always @(posedge clk_i) begin
        if (rst_ni && cerf_fb_done_i && cerf_fb_req_o) begin
            $display("[BINGO_CERF_FB] %0t type %0d clear g%0d set g%0d",
                     $time, cerf_fb_type_o, cerf_fb_clear_o, cerf_fb_set_o);
        end
    end
`endif

endmodule
