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
module bingo_hw_manager_evlog_events
    import bingo_hw_manager_pkg::*;
#(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter type         task_desc_t = logic,  // bingo_hw_manager_task_desc_t of the top
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         slot_state_t = logic,
    parameter type         ctrl_view_t = logic,
    parameter type         park_t = logic,
    parameter type         checkout_t = logic,
    parameter type         type_cfg_t = logic,
    parameter type         cerf_fb_t = logic,
    parameter type         replay_step_t = logic,
    // Dependent parameters, DO NOT OVERRIDE!
    parameter int unsigned NumSlots = NumCores * NumClusters,
    parameter int unsigned NumTypes = 2**CoreTypeIdWidth,
    parameter int unsigned NumEvents = EvlogGroups * NumSlots + NumTypes
) (
    input  logic                                      clk_i,
    input  logic                                      rst_ni,
    // Slot state: an event when it rises (suspect: also when it falls before
    // the fence; recovery hold: on every change). Fenced, stuck and retired
    // (slot state), suspect (watchdog), blocked (replay), risk and recovery
    // hold (control plane view), parked and park failed (parking)
    input  slot_state_t                               slot_i,
    input  logic [NumCores-1:0][NumClusters-1:0]      suspect_i,
    input  logic [NumCores-1:0][NumClusters-1:0]      blocked_i,
    input  ctrl_view_t                                view_i,
    input  park_t                                     park_i,
    // The threshold that would confirm a fence of the slot now (argument of
    // the fence record: kept from the edge the watchdog commits the fence)
    input  logic [NumCores-1:0][NumClusters-1:0][2:0] fence_reason_i,
    // An exit absorbed into a dummy, and the entry it enters as (checkout queues)
    input  checkout_t                                 checkout_i,
    // CERF fallback of a core type (bingo_hw_manager_cerf_fallback), committed
    // by the CERF; the host table and the types that have fired
    input  cerf_fb_t                                  cerf_fb_i,
    input  logic                                      cerf_fb_done_i,
    input  type_cfg_t                                 type_cfg_i,
    input  logic [NumTypes-1:0]                       cerf_fb_evt_i,
    // Replay moves (bingo_hw_manager_replay_ctrl), and the entry moved
    input  replay_step_t                              step_i,
    input  task_desc_t                                move_desc_i,
    // To bingo_hw_manager_evlog
    output logic [NumEvents-1:0]                      events_o,
    output logic [NumEvents-1:0][15:0]                args_o,
    output evlog_move_t                               move_o
);
    // The move record (move_o)
    logic        move_valid;
    logic [7:0]  move_code, move_slot;
    logic [15:0] move_arg;
    assign move_o = '{valid: move_valid, code: move_code, slot: move_slot, arg: move_arg};

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
    logic [NumCores-1:0][NumClusters-1:0][2:0] reason_q;

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
            seen_q[EvgFence]    <= slot_i.fenced;
            seen_q[EvgStuck]    <= slot_i.stuck;
            seen_q[EvgBlocked]  <= blocked_i;
            seen_q[EvgRisk]     <= view_i.risk;
            seen_q[EvgParked]   <= park_i.parked;
            seen_q[EvgParkFail] <= park_i.fail;
            seen_q[EvgRetired]  <= slot_i.retired;
            rec_seen_q <= view_i.rec_hold;
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
                    !seen_q[EvgSuspect][c][cl] && !slot_i.fenced[c][cl];
                events_o[ev(EvgClear, s)] = !suspect_i[c][cl] &&
                    seen_q[EvgClear][c][cl] && !slot_i.fenced[c][cl];
                events_o[ev(EvgFence, s)] = slot_i.fenced[c][cl] && !seen_q[EvgFence][c][cl];
                args_o[ev(EvgFence, s)] = {13'b0, reason_q[c][cl]};
                events_o[ev(EvgStuck, s)] = slot_i.stuck[c][cl] && !seen_q[EvgStuck][c][cl];
                events_o[ev(EvgBlocked, s)] = blocked_i[c][cl] && !seen_q[EvgBlocked][c][cl];
                events_o[ev(EvgRisk, s)] = view_i.risk[c][cl] && !seen_q[EvgRisk][c][cl];
                events_o[ev(EvgParked, s)] = park_i.parked[c][cl] && !seen_q[EvgParked][c][cl];
                events_o[ev(EvgParkFail, s)] = park_i.fail[c][cl] && !seen_q[EvgParkFail][c][cl];
                events_o[ev(EvgRetired, s)] = slot_i.retired[c][cl] && !seen_q[EvgRetired][c][cl];
                events_o[ev(EvgRecHold, s)] = view_i.rec_hold[c][cl] != rec_seen_q[c][cl];
                args_o[ev(EvgRecHold, s)] = {15'b0, view_i.rec_hold[c][cl]};
                events_o[ev(EvgExitAbsorb, s)] = checkout_i.exit_absorb[c][cl];
                args_o[ev(EvgExitAbsorb, s)] = {
                    4'(checkout_i.push_desc[c][cl].assigned_core_id +
                       checkout_i.push_desc[c][cl].assigned_cluster_id * NumCores),
                    12'(checkout_i.push_desc[c][cl].task_id)};
            end
        end
        // armed_q (cerf_fb_evt_i) rises in the NBA of this commit edge.
        // Observe its proven next-state rise, not a delayed sampled rise:
        // this is exactly the existing [BINGO_CERF_FB] print edge.
        for (int t = 0; t < NumTypes; t++) begin
            events_o[ev_type(t)] = cerf_fb_done_i && cerf_fb_i.req &&
                cerf_fb_i.core_type == CoreTypeIdWidth'(t) && type_cfg_i.cerf_fb_en[t] &&
                !cerf_fb_evt_i[t];
            args_o[ev_type(t)] = {6'b0, type_cfg_i.cerf_fb_set[t], type_cfg_i.cerf_fb_clear[t]};
        end
        move_code = step_i.bounce ? EvBounce :
                    ((move_desc_i.task_type == TaskDummy) ? EvMoveDummy : EvMove);
        move_slot = {4'(step_i.src_cluster), 4'(step_i.src_core)};
        move_arg = {((step_i.bounce) ? 4'h0 : ((step_i.rotate) ? 4'hf :
            4'(step_i.dst_core + step_i.dst_cluster * NumCores))),
            12'(move_desc_i.task_id)};
    end
    assign move_valid = step_i.fire && !step_i.bounce || step_i.bounce;
endmodule
