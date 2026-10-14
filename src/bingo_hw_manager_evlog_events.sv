// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Event sources of the control event log (bingo_hw_manager_evlog).
// Observation only: nothing here feeds back into the control plane.
//
// Turns the per-slot state of the control plane into one pulse per (group,
// slot) on the edge where the state changes, at bingo_hw_manager_pkg's
// evlog_idx() (one pulse per core type at evlog_type_idx() for a CERF
// fallback), with the argument of the record; and the replay controller's
// moves into move records. The pulses come on the same clock edge as the
// top's simulation prints of the same events.
module bingo_hw_manager_evlog_events #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter type         task_desc_t = logic,  // bingo_hw_manager_task_desc_t of the top
    // Dependent parameters, DO NOT OVERRIDE!
    parameter int unsigned NumSlots = NumCores * NumClusters,
    parameter int unsigned NumTypes = 2**CoreTypeIdWidth,
    parameter int unsigned NumEvents = bingo_hw_manager_pkg::EvlogGroups * NumSlots + NumTypes
) (
    input  logic                                      clk_i,
    input  logic                                      rst_ni,
    // Slot state: an event when it rises (suspect: also when it falls before
    // the fence; recovery hold: on every change)
    input  logic       [NumCores-1:0][NumClusters-1:0] suspect_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] fenced_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] stuck_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] blocked_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] risk_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] parked_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] park_fail_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] retired_i,
    input  logic       [NumCores-1:0][NumClusters-1:0] rec_hold_i,
    // The threshold that would confirm a fence of the slot now (argument of
    // the fence record: kept from the edge the watchdog commits the fence)
    input  logic [NumCores-1:0][NumClusters-1:0][1:0]  fence_reason_i,
    // An exit absorbed into a dummy, and the entry it enters as
    input  logic       [NumCores-1:0][NumClusters-1:0] exit_absorb_i,
    input  task_desc_t [NumCores-1:0][NumClusters-1:0] exit_desc_i,
    // CERF fallback of a core type (bingo_hw_manager_cerf_fallback), committed by the CERF
    input  logic                                      cerf_fb_req_i,
    input  logic [CoreTypeIdWidth-1:0]                cerf_fb_type_i,
    input  logic                                      cerf_fb_done_i,
    input  logic [NumTypes-1:0]                       cerf_fb_en_i,
    input  logic [NumTypes-1:0]                       cerf_fb_evt_i,
    input  logic [NumTypes-1:0][4:0]                  cerf_fb_clear_i,
    input  logic [NumTypes-1:0][4:0]                  cerf_fb_set_i,
    // Replay moves (bingo_hw_manager_replay_ctrl)
    input  logic                                      move_fire_i,
    input  logic                                      bounce_i,
    input  logic                                      rotate_i,
    input  task_desc_t                                move_desc_i,
    input  logic [CoreIdWidth-1:0]                    move_src_core_i,
    input  logic [ClusterIdWidth-1:0]                 move_src_cluster_i,
    input  logic [CoreIdWidth-1:0]                    move_dst_core_i,
    input  logic [ClusterIdWidth-1:0]                 move_dst_cluster_i,
    // To bingo_hw_manager_evlog
    output logic [NumEvents-1:0]                      events_o,
    output logic [NumEvents-1:0][15:0]                args_o,
    output logic                                      move_o,
    output logic [7:0]                                move_code_o,
    output logic [7:0]                                move_slot_o,
    output logic [15:0]                               move_arg_o
);
    import bingo_hw_manager_pkg::*;

    // Position of a slot pulse / a type pulse in the event vector
    function automatic int ev(input evlog_group_e group, input int slot);
        return evlog_idx(group, slot, NumSlots, NumTypes);
    endfunction
    function automatic int ev_type(input int core_type);
        return evlog_type_idx(core_type, NumSlots);
    endfunction
    // Last value of the level behind each of the first nine groups
    logic [EvlogGroupsBeforeTypes-1:0][NumCores-1:0][NumClusters-1:0] seen_q;
    logic [NumCores-1:0][NumClusters-1:0] rec_seen_q;
    logic [NumCores-1:0][NumClusters-1:0][1:0] reason_q;

    // The watchdog commits fenced one edge before the existing top print.
    // Keep that edge's reason, without reading or changing its control.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            seen_q <= '0;
            reason_q <= '0;
            rec_seen_q <= '0;
        end else begin
            seen_q[EvgSuspect]  <= suspect_i;
            seen_q[EvgClear]    <= suspect_i;
            seen_q[EvgFence]    <= fenced_i;
            seen_q[EvgStuck]    <= stuck_i;
            seen_q[EvgBlocked]  <= blocked_i;
            seen_q[EvgRisk]     <= risk_i;
            seen_q[EvgParked]   <= parked_i;
            seen_q[EvgParkFail] <= park_fail_i;
            seen_q[EvgRetired]  <= retired_i;
            rec_seen_q <= rec_hold_i;
            reason_q   <= fence_reason_i;
        end
    end

    always_comb begin
        events_o = '0;
        args_o = '0;
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                automatic int s = c + cl * NumCores;
                events_o[ev(EvgSuspect, s)] = suspect_i[c][cl] &&
                    !seen_q[EvgSuspect][c][cl] && !fenced_i[c][cl];
                events_o[ev(EvgClear, s)] = !suspect_i[c][cl] &&
                    seen_q[EvgClear][c][cl] && !fenced_i[c][cl];
                events_o[ev(EvgFence, s)] = fenced_i[c][cl] && !seen_q[EvgFence][c][cl];
                args_o[ev(EvgFence, s)] = {14'b0, reason_q[c][cl]};
                events_o[ev(EvgStuck, s)] = stuck_i[c][cl] && !seen_q[EvgStuck][c][cl];
                events_o[ev(EvgBlocked, s)] = blocked_i[c][cl] && !seen_q[EvgBlocked][c][cl];
                events_o[ev(EvgRisk, s)] = risk_i[c][cl] && !seen_q[EvgRisk][c][cl];
                events_o[ev(EvgParked, s)] = parked_i[c][cl] && !seen_q[EvgParked][c][cl];
                events_o[ev(EvgParkFail, s)] = park_fail_i[c][cl] && !seen_q[EvgParkFail][c][cl];
                events_o[ev(EvgRetired, s)] = retired_i[c][cl] && !seen_q[EvgRetired][c][cl];
                events_o[ev(EvgRecHold, s)] = rec_hold_i[c][cl] != rec_seen_q[c][cl];
                args_o[ev(EvgRecHold, s)] = {15'b0, rec_hold_i[c][cl]};
                events_o[ev(EvgExitAbsorb, s)] = exit_absorb_i[c][cl];
                args_o[ev(EvgExitAbsorb, s)] = {
                    4'(exit_desc_i[c][cl].assigned_core_id +
                       exit_desc_i[c][cl].assigned_cluster_id * NumCores),
                    12'(exit_desc_i[c][cl].task_id)};
            end
        end
        // armed_q (cerf_fb_evt_i) rises in the NBA of this commit edge.
        // Observe its proven next-state rise, not a delayed sampled rise:
        // this is exactly the existing [BINGO_CERF_FB] print edge.
        for (int t = 0; t < NumTypes; t++) begin
            events_o[ev_type(t)] = cerf_fb_done_i && cerf_fb_req_i &&
                cerf_fb_type_i == CoreTypeIdWidth'(t) && cerf_fb_en_i[t] &&
                !cerf_fb_evt_i[t];
            args_o[ev_type(t)] = {6'b0, cerf_fb_set_i[t], cerf_fb_clear_i[t]};
        end
        move_code_o = bounce_i ? EvBounce :
                      ((move_desc_i.task_type == TaskDummy) ? EvMoveDummy : EvMove);
        move_slot_o = {4'(move_src_cluster_i), 4'(move_src_core_i)};
        move_arg_o = {((bounce_i) ? 4'h0 : ((rotate_i) ? 4'hf :
            4'(move_dst_core_i + move_dst_cluster_i * NumCores))),
            12'(move_desc_i.task_id)};
    end
    assign move_o = move_fire_i && !bounce_i || bounce_i;
endmodule
