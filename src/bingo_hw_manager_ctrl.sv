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
// when a slot is fenced, or when a live slot is parked (below): an entry is
// recomputed if its logical slot or its current substitute was just fenced
// (bingo_hw_manager_substitute_sel over the new fenced set); every other entry
// keeps its value. With the default choice
// (the lowest live slot of the same type, logical cluster first) removing a slot
// that is not the chosen one never changes a choice, so the table always equals
// the combinational choice of bingo_hw_manager_substitute_sel (checked in
// simulation). Keeping the choice in a register is what lets a policy use
// information that changes over time without moving a dead core's tasks to a
// different substitute half-way:
//   SubstitutePolicy 0  lowest live slot of the type (logical cluster first)
//   SubstitutePolicy 1  least loaded live slot of the type (load_i: e.g. the
//                       occupancy of its checkout queue), logical cluster first,
//                       lowest index on a tie. The load is sampled when the
//                       entry is recomputed, i.e. when the core dies.
//
// In the cycle a slot is fenced, the cycle a live slot is parked, and the cycle
// after reset, the table is being written: smt_update_o asks its readers to
// wait one cycle.
//
// Core parking (host park_req_i, one bit per logical slot; 0 = off, and the
// machine then matches a build without this port). A requested slot that is
// not fenced enters HOLD: bingo_hw_manager_core_remap admits no new task to
// its checkout. Tasks already in its checkout or ready queue run to completion.
// Once both queues are empty and nothing is pushed into the slot this cycle,
// the same substitute selector (same SubstitutePolicy, same SubstituteLevelMask,
// with this slot and every slot in HOLD or PARKED masked out of the candidates,
// so two slots never park onto each other and a draining slot gets no new
// work) picks a live core of its type.
// That result is written into the table and the slot becomes PARKED: later
// executing tasks follow the table, dummy-set and CERF-skipped tasks stay on
// the logical core. Level 3 is not a parking substitute. No live core of the
// type releases HOLD and raises park_fail_o; the bit stays until the request
// drops, and the request is not tried again while it stays set. A slot that
// runs the tasks of another logical slot (the table target of a fenced or
// parked slot) cannot drain on its own: a request then fails at once, and a
// slot that becomes such a target while in HOLD gives up the same way, so its
// own tasks are not held behind the other slot's. A fence of the
// slot itself drops HOLD or PARKED and writes the table only through the fence
// update above, never as a park write in that same cycle. Clearing park_req_i
// of a PARKED slot moves it back (UNPARK): its new tasks are held, on neither
// core, until none of its tasks is left on another core (moved_i, the per
// logical core count of entries in other cores' checkout queues), so a task on
// the slot itself cannot overtake an older one on the substitute; then the
// entry goes back to the plain choice and the slot is no longer parked. The
// substitute keeps running its own tasks meanwhile: its queue is in order, so
// they never delay the parked slot's. Setting the request again during UNPARK
// returns to PARKED. If the substitute of a parked slot is fenced, the
// entry is recomputed with the slot still masked out; if nothing is left, the
// slot leaves PARKED, park_fail_o is raised, and the table points back at the
// slot so a replay of entries already on the dead substitute has a live core.
//
// Power and load view of the slots (fault-aware): a fenced slot is dead, so it
// no longer keeps its power domain at the normal level (pm_idle_o: polling for a
// task, or fenced) and no longer counts as load (load_clear_o). A slot that is
// only dead_suspect may still be working and keeps both.
// Idle entry delay (idle_delay_i, host-configured): a slot only counts as idle
// for the PM once it has been idle for idle_delay_i cycles, so a domain only
// drops to the idle level after all its slots were idle that long. Short gaps
// between tasks then keep the normal level and avoid the wake-up cost; 0 =
// at once (the PM's own behaviour).
// External access wake (access_hold_i, host-configured): a cluster whose memory
// is accessed from outside (cluster_access_i, e.g. the host reading its L1)
// runs on its own clock, so while it is accessed, and for access_hold_i cycles
// after the last access, none of its slots counts as idle, not even a fenced
// one. The access path is not held at the idle level; 0 = off. This only ever
// makes a slot idle later: the idle entry delay keeps counting the core's own
// polling, so once the hold ends a long idle core lets its domain drop at once.
//
// Frequency-aware watchdog: the power levels are clock dividers. While a slot's
// domain runs at level L above the normal level N (slower), its core makes
// progress N/L times as fast, so its heartbeat gaps stretch by L/N in manager
// cycles. wd_tick_o advances the slot's watchdog timer only N/L of the cycles
// (a fractional accumulator), so the timeouts count cycles of the normal clock
// and can be set for the normal level instead of the slowest one. The level is
// the one the DFS path applied to the domain, or the one the host acknowledged
// in DVFS mode (pm_mode_i[0]); without idle PM, or at or below the normal
// level, the timer advances every cycle.
//
// Recovery boost: a live slot that is the substitute of a fenced core in the
// table does the work of two cores. While it is busy (not polling), pm_boost_o
// asks the PM to run its domain at the boost level (bingo_hw_manager_pm
// boost_power_level_i, a faster clock than the normal level; 0 = off).
//
// CERF degradation: a host table, one entry per core type, default off. A type
// is faulted while any of its slots is stuck or rejected. The lowest enabled
// faulted type that has not fired yet requests one CERF update (clear one
// group, set another). The register file applies it only in a cycle the host
// does not write the whole mask; the request stays up until cerf_fb_done_i.
// cerf_fb_evt_o marks types that have fired and drops when that enable drops,
// which arms the type again if it is still faulted. One fire per enable, so a
// later host write of the mask is left alone.
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
    // Substitute choice (see above) and width of load_i
    parameter int unsigned SubstitutePolicy = 0,
    parameter int unsigned LoadWidth = 4
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
    // Host park request, checkout / ready empty, and a push into the slot
    // this cycle (route, replay, or import). See the parking note above.
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
    input  logic [2**CoreTypeIdWidth-1:0][4:0]       cerf_fb_clear_i = '0,
    input  logic [2**CoreTypeIdWidth-1:0][4:0]       cerf_fb_set_i = '0,
    // The CERF update committed this cycle (host did not write the mask)
    input  logic                                     cerf_fb_done_i = 1'b0,

    // Power manager: slots that do not keep their domain at the normal level
    output logic [NumCores-1:0][NumClusters-1:0] pm_idle_o,
    // Load monitor: slots whose pending count is cleared
    output logic [NumCores-1:0][NumClusters-1:0] load_clear_o,
    // Watchdog: the slot's timer advances this cycle
    output logic [NumCores-1:0][NumClusters-1:0] wd_tick_o,
    // Power manager: slots that run a dead core's tasks and are busy
    output logic [NumCores-1:0][NumClusters-1:0] pm_boost_o,

    // Slot mapping table, indexed by logical slot
    output logic [NumCores-1:0][NumClusters-1:0]                     smt_found_o,
    output logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_o,
    output logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_o,
    // The table is written this cycle: its readers hold
    output logic                                                     smt_update_o,
    // Parking (see above). fail stays set until that slot's request drops.
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
    output logic [2**CoreTypeIdWidth-1:0]            cerf_fb_evt_o
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
                .LevelMask(SubstituteLevelMask),
                .LeastWeight(SubstitutePolicy == 1),
                .WeightWidth(LoadWidth)
            ) i_choice (
                .logical_core_i(CoreIdWidth'(c)),
                .logical_cluster_i(ClusterIdWidth'(cl)),
                .fenced_i(fenced_i),
                .weight_i(load_i),
                .found_o(choice_found[c][cl]),
                .core_o(choice_core[c][cl]),
                .cluster_o(choice_cluster[c][cl])
            );
        end
    end

    // ------------------------------------------------------------------
    // Parking substitute: the same selector, with this slot and the slots in
    // HOLD or PARKED masked out
    // ------------------------------------------------------------------
    logic [NumCores-1:0][NumClusters-1:0]                     park_hold_q;
    logic [NumCores-1:0][NumClusters-1:0]                     park_parked_q;
    logic [NumCores-1:0][NumClusters-1:0]                     park_fail_q;
    logic [NumCores-1:0][NumClusters-1:0]                     park_unpark_q;
    logic [NumCores-1:0][NumClusters-1:0]                     unpark_commit;
    logic [NumCores-1:0][NumClusters-1:0]                     park_choice_found;
    logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    park_choice_core;
    logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] park_choice_cluster;

    for (genvar c = 0; c < NumCores; c++) begin : gen_park_choice_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_park_choice_cluster
            logic [NumCores-1:0][NumClusters-1:0] fenced_excl;
            always_comb begin
                fenced_excl = fenced_i | park_hold_q | park_parked_q;
                fenced_excl[c][cl] = 1'b1;
            end
            bingo_hw_manager_substitute_sel #(
                .NumCores(NumCores),
                .NumClusters(NumClusters),
                .CoreIdWidth(CoreIdWidth),
                .ClusterIdWidth(ClusterIdWidth),
                .CoreTypeIdWidth(CoreTypeIdWidth),
                .CoreTypeId(CoreTypeId),
                .LevelMask(SubstituteLevelMask),
                .LeastWeight(SubstitutePolicy == 1),
                .WeightWidth(LoadWidth)
            ) i_park_choice (
                .logical_core_i(CoreIdWidth'(c)),
                .logical_cluster_i(ClusterIdWidth'(cl)),
                .fenced_i(fenced_excl),
                .weight_i(load_i),
                .found_o(park_choice_found[c][cl]),
                .core_o(park_choice_core[c][cl]),
                .cluster_o(park_choice_cluster[c][cl])
            );
        end
    end

    // ------------------------------------------------------------------
    // Table update: a new fence, or a park commit after the slot drained
    // ------------------------------------------------------------------
    logic                                                     init_q;
    logic [NumCores-1:0][NumClusters-1:0]                     fenced_seen_q;
    logic [NumCores-1:0][NumClusters-1:0]                     fence_new;
    logic [NumCores-1:0][NumClusters-1:0]                     recompute;
    logic [NumCores-1:0][NumClusters-1:0]                     smt_found_q;
    logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_q;
    logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_q;
    logic [NumCores-1:0][NumClusters-1:0]                     park_choice_ok;
    logic [NumCores-1:0][NumClusters-1:0]                     park_target;
    logic [NumCores-1:0][NumClusters-1:0]                     park_drained;
    logic [NumCores-1:0][NumClusters-1:0]                     park_commit;
    logic [NumCores-1:0][NumClusters-1:0]                     park_give_up;

    assign fence_new = fenced_i & ~fenced_seen_q;

    // Slots that are the table target of a fenced or parked logical slot: they
    // receive that slot's tasks, so they cannot drain on their own
    always_comb begin
        park_target = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (smt_found_q[c][cl] && (fenced_i[c][cl] || park_parked_q[c][cl]) &&
                    ((smt_core_q[c][cl] != CoreIdWidth'(c)) || (smt_cluster_q[c][cl] != ClusterIdWidth'(cl)))) begin
                    park_target[smt_core_q[c][cl]][smt_cluster_q[c][cl]] = 1'b1;
                end
            end
        end
    end

    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                recompute[c][cl] = init_q || fence_new[c][cl] ||
                    (smt_found_q[c][cl] && fence_new[smt_core_q[c][cl]][smt_cluster_q[c][cl]]);
                park_choice_ok[c][cl] = park_choice_found[c][cl] &&
                    !((park_choice_core[c][cl] == CoreIdWidth'(c)) &&
                      (park_choice_cluster[c][cl] == ClusterIdWidth'(cl)));
                park_drained[c][cl] = checkout_empty_i[c][cl] && ready_empty_i[c][cl] &&
                                      !slot_push_i[c][cl];
                // Not in the cycle this slot is fenced: that write belongs to the fence.
                park_commit[c][cl] = park_hold_q[c][cl] && !park_parked_q[c][cl] &&
                                     !fenced_i[c][cl] && !fence_new[c][cl] && !park_target[c][cl] &&
                                     park_drained[c][cl] && park_choice_ok[c][cl];
                unpark_commit[c][cl] = park_parked_q[c][cl] && park_unpark_q[c][cl] &&
                                       !fenced_i[c][cl] && !fence_new[c][cl] && !recompute[c][cl] &&
                                       !moved_i[c][cl];
                park_give_up[c][cl] = park_hold_q[c][cl] && !park_parked_q[c][cl] &&
                                      !fenced_i[c][cl] && !fence_new[c][cl] &&
                                      park_drained[c][cl] && !park_choice_ok[c][cl];
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
            park_hold_q   <= '0;
            park_parked_q <= '0;
            park_fail_q   <= '0;
            park_unpark_q <= '0;
        end else begin
            init_q        <= 1'b0;
            fenced_seen_q <= fenced_i;
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (recompute[c][cl]) begin
                        // A parked slot whose substitute just died stays masked out
                        // of its own search. The slot's own fence uses `choice`.
                        if (park_parked_q[c][cl] && !fence_new[c][cl] && !init_q &&
                            park_choice_ok[c][cl]) begin
                            smt_found_q[c][cl]   <= 1'b1;
                            smt_core_q[c][cl]    <= park_choice_core[c][cl];
                            smt_cluster_q[c][cl] <= park_choice_cluster[c][cl];
                        end else if (park_parked_q[c][cl] && !fence_new[c][cl] && !init_q) begin
                            smt_found_q[c][cl]   <= 1'b1;
                            smt_core_q[c][cl]    <= CoreIdWidth'(c);
                            smt_cluster_q[c][cl] <= ClusterIdWidth'(cl);
                        end else begin
                            smt_found_q[c][cl]   <= choice_found[c][cl];
                            smt_core_q[c][cl]    <= choice_core[c][cl];
                            smt_cluster_q[c][cl] <= choice_cluster[c][cl];
                        end
                    end else if (park_commit[c][cl]) begin
                        smt_found_q[c][cl]   <= 1'b1;
                        smt_core_q[c][cl]    <= park_choice_core[c][cl];
                        smt_cluster_q[c][cl] <= park_choice_cluster[c][cl];
                    end else if (unpark_commit[c][cl]) begin
                        // Back to the entry of a live slot (the plain choice)
                        smt_found_q[c][cl]   <= choice_found[c][cl];
                        smt_core_q[c][cl]    <= choice_core[c][cl];
                        smt_cluster_q[c][cl] <= choice_cluster[c][cl];
                    end

                    if (fenced_i[c][cl]) begin
                        park_hold_q[c][cl]   <= 1'b0;
                        park_parked_q[c][cl] <= 1'b0;
                        park_fail_q[c][cl]   <= 1'b0;
                        park_unpark_q[c][cl] <= 1'b0;
                    end else if (park_commit[c][cl]) begin
                        park_hold_q[c][cl]   <= 1'b0;
                        park_parked_q[c][cl] <= 1'b1;
                        park_fail_q[c][cl]   <= 1'b0;
                    end else if (park_give_up[c][cl] || (park_hold_q[c][cl] && park_target[c][cl])) begin
                        park_hold_q[c][cl] <= 1'b0;
                        park_fail_q[c][cl] <= 1'b1;
                    end else if (park_hold_q[c][cl] && !park_req_i[c][cl]) begin
                        park_hold_q[c][cl] <= 1'b0;
                    end else if (!park_hold_q[c][cl] && !park_parked_q[c][cl] &&
                                 !park_fail_q[c][cl] && park_req_i[c][cl]) begin
                        // A slot that runs another slot's tasks cannot drain: fail at once
                        if (park_target[c][cl]) park_fail_q[c][cl] <= 1'b1;
                        else                    park_hold_q[c][cl] <= 1'b1;
                    end else if (park_fail_q[c][cl] && !park_req_i[c][cl]) begin
                        park_fail_q[c][cl] <= 1'b0;
                    end else if (park_parked_q[c][cl] && recompute[c][cl] &&
                                 !fence_new[c][cl] && !init_q && !park_choice_ok[c][cl]) begin
                        park_parked_q[c][cl] <= 1'b0;
                        park_unpark_q[c][cl] <= 1'b0;
                        park_fail_q[c][cl]   <= 1'b1;
                    end else if (unpark_commit[c][cl]) begin
                        park_parked_q[c][cl] <= 1'b0;
                        park_unpark_q[c][cl] <= 1'b0;
                    end else if (park_parked_q[c][cl] && !park_unpark_q[c][cl] && !park_req_i[c][cl]) begin
                        park_unpark_q[c][cl] <= 1'b1;
                    end else if (park_unpark_q[c][cl] && park_req_i[c][cl]) begin
                        // Asked again before the move back finished: stay parked
                        park_unpark_q[c][cl] <= 1'b0;
                    end
                end
            end
        end
    end

    assign smt_found_o   = smt_found_q;
    assign smt_core_o    = smt_core_q;
    assign smt_cluster_o = smt_cluster_q;
    assign smt_update_o  = init_q || (|fence_new) || (|park_commit) || (|unpark_commit);
    assign park_hold_o   = park_hold_q;
    assign park_parked_o = park_parked_q;
    assign park_unpark_o = park_unpark_q;
    assign park_fail_o   = park_fail_q;

    // ------------------------------------------------------------------
    // Fault-aware power and load view
    // ------------------------------------------------------------------
    // Clusters accessed from outside within the last access_hold_i cycles
    logic [NumClusters-1:0]       cluster_awake;
    logic [NumClusters-1:0][31:0] access_cnt_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            access_cnt_q <= '0;
        end else begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (cluster_access_i[cl])        access_cnt_q[cl] <= access_hold_i;
                else if (access_cnt_q[cl] != '0) access_cnt_q[cl] <= access_cnt_q[cl] - 32'd1;
            end
        end
    end
    for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_cluster_awake
        assign cluster_awake[cl] = (access_hold_i != '0) && (cluster_access_i[cl] || (access_cnt_q[cl] != '0));
    end
    logic [NumCores-1:0][NumClusters-1:0]       slot_idle;
    logic [NumCores-1:0][NumClusters-1:0][31:0] idle_cnt_q;
    assign slot_idle = waiting_i | fenced_i;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            idle_cnt_q <= '0;
        end else begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (!slot_idle[c][cl])                    idle_cnt_q[c][cl] <= '0;
                    else if (idle_cnt_q[c][cl] != '1)         idle_cnt_q[c][cl] <= idle_cnt_q[c][cl] + 32'd1;
                end
            end
        end
    end
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                pm_idle_o[c][cl] = slot_idle[c][cl] && (idle_cnt_q[c][cl] >= idle_delay_i) && !cluster_awake[cl];
            end
        end
    end
    assign load_clear_o = fenced_i;

    // ------------------------------------------------------------------
    // Frequency-aware watchdog ticks
    // ------------------------------------------------------------------
    logic [NumCores-1:0][NumClusters-1:0][7:0] slot_level;
    logic [NumCores-1:0][NumClusters-1:0]      slot_slow;
    logic [NumCores-1:0][NumClusters-1:0][8:0] tick_acc_q, tick_acc_d;
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                slot_level[c][cl] = pm_dvfs_i ? dvfs_level_i :
                                    (slot_domain_i[c][cl] < 32) ? domain_level_i[slot_domain_i[c][cl][4:0]] : 8'd0;
                slot_slow[c][cl]  = pm_enable_i && (normal_level_i != '0) && (slot_level[c][cl] > normal_level_i);
                // Accumulate N per cycle, tick (and subtract L) once it reaches L
                if (!slot_slow[c][cl]) begin
                    wd_tick_o[c][cl]  = 1'b1;
                    tick_acc_d[c][cl] = '0;
                end else if ((tick_acc_q[c][cl] + 9'(normal_level_i)) >= 9'(slot_level[c][cl])) begin
                    wd_tick_o[c][cl]  = 1'b1;
                    tick_acc_d[c][cl] = tick_acc_q[c][cl] + 9'(normal_level_i) - 9'(slot_level[c][cl]);
                end else begin
                    wd_tick_o[c][cl]  = 1'b0;
                    tick_acc_d[c][cl] = tick_acc_q[c][cl] + 9'(normal_level_i);
                end
            end
        end
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) tick_acc_q <= '0;
        else         tick_acc_q <= tick_acc_d;
    end

    // ------------------------------------------------------------------
    // Recovery boost: substitutes of fenced cores, while busy
    // ------------------------------------------------------------------
    always_comb begin
        pm_boost_o = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (fenced_i[c][cl] && smt_found_q[c][cl] &&
                    ((smt_core_q[c][cl] != c) || (smt_cluster_q[c][cl] != cl))) begin
                    pm_boost_o[smt_core_q[c][cl]][smt_cluster_q[c][cl]] = 1'b1;
                end
            end
        end
        pm_boost_o = pm_boost_o & ~waiting_i & ~fenced_i;
    end

    // ------------------------------------------------------------------
    // CERF degradation: one type per cycle, lowest index first
    // ------------------------------------------------------------------
    localparam int unsigned NumTypes = 2 ** CoreTypeIdWidth;

    logic [NumTypes-1:0]            type_fault;
    logic [NumTypes-1:0]            pending;
    logic [NumTypes-1:0]            armed_q, armed_d;
    logic                           fb_req;
    logic [CoreTypeIdWidth-1:0]     fb_sel;
    logic [4:0]                     fb_clear, fb_set;

    always_comb begin
        type_fault = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (stuck_i[c][cl] || rejected_i[c][cl]) begin
                    type_fault[CoreTypeId[c][cl]] = 1'b1;
                end
            end
        end
    end
    assign pending = cerf_fb_en_i & type_fault & ~armed_q;

    always_comb begin
        fb_req   = 1'b0;
        fb_sel   = '0;
        fb_clear = '0;
        fb_set   = '0;
        for (int unsigned t = 0; t < NumTypes; t++) begin
            if (pending[t] && !fb_req) begin
                fb_req   = 1'b1;
                fb_sel   = CoreTypeIdWidth'(t);
                fb_clear = cerf_fb_clear_i[t];
                fb_set   = cerf_fb_set_i[t];
            end
        end
    end
    assign cerf_fb_req_o   = fb_req;
    assign cerf_fb_type_o  = fb_sel;
    assign cerf_fb_clear_o = fb_clear;
    assign cerf_fb_set_o   = fb_set;
    assign cerf_fb_evt_o   = armed_q;

    always_comb begin
        armed_d = armed_q & cerf_fb_en_i;
        if (cerf_fb_done_i && fb_req && cerf_fb_en_i[fb_sel]) begin
            armed_d[fb_sel] = 1'b1;
        end
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) armed_q <= '0;
        else         armed_q <= armed_d;
    end

`ifndef SYNTHESIS
    always @(posedge clk_i) begin
        if (rst_ni && cerf_fb_done_i && fb_req) begin
            $display("[BINGO_CERF_FB] %0t type %0d clear g%0d set g%0d",
                     $time, fb_sel, fb_clear, fb_set);
        end
    end
`endif

`ifndef SYNTHESIS
    // Outside an update cycle the table equals the combinational choice (only
    // for the lowest-index policy: a load-based choice depends on the load when
    // the core died)
    always @(posedge clk_i) begin : smt_equivalence
        if (rst_ni && !smt_update_o && (SubstitutePolicy == 0)) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    // A parked entry was chosen among the slots that were neither in
                    // HOLD nor PARKED at the time, so it is only checked for validity
                    // (below and here: another slot, never itself, never parked).
                    if (park_parked_q[c][cl]) begin
                        if (!smt_found_q[c][cl] ||
                            ((smt_core_q[c][cl] == CoreIdWidth'(c)) && (smt_cluster_q[c][cl] == ClusterIdWidth'(cl))) ||
                            park_parked_q[smt_core_q[c][cl]][smt_cluster_q[c][cl]]) begin
                            $error("[BINGO_ASSERT] parked SMT entry core %0d cluster %0d: found %0b core %0d cluster %0d",
                                   c, cl, smt_found_q[c][cl], smt_core_q[c][cl], smt_cluster_q[c][cl]);
                        end
                    end else if (!fenced_i[c][cl] && smt_found_q[c][cl] &&
                                 (smt_core_q[c][cl] == CoreIdWidth'(c)) && (smt_cluster_q[c][cl] == ClusterIdWidth'(cl))) begin
                        // A park that fell back to the slot itself (its substitute died
                        // and none was left): the replay of its old tasks went there.
                        // A live slot's entry is not read until it is fenced or parked,
                        // and both write it again.
                    end else if ((smt_found_q[c][cl] != choice_found[c][cl]) ||
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
    // Every policy: a substitute in the table is live and of the logical slot's type
    always @(posedge clk_i) begin : smt_validity
        if (rst_ni && !smt_update_o) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (smt_found_q[c][cl] &&
                        (fenced_i[smt_core_q[c][cl]][smt_cluster_q[c][cl]] ||
                         (((smt_core_q[c][cl] != c) || (smt_cluster_q[c][cl] != cl)) &&
                          ((CoreTypeId[c][cl] == '0) ||
                           (CoreTypeId[smt_core_q[c][cl]][smt_cluster_q[c][cl]] != CoreTypeId[c][cl]))))) begin
                        $error("[BINGO_ASSERT] SMT entry core %0d cluster %0d -> core %0d cluster %0d is fenced or of another type",
                               c, cl, smt_core_q[c][cl], smt_cluster_q[c][cl]);
                    end
                    // Cleared on the edge that sees the fence, so a slot fenced since
                    // the previous edge must no longer be in HOLD or PARKED.
                    if ((park_hold_q[c][cl] && park_parked_q[c][cl]) ||
                        (park_unpark_q[c][cl] && !park_parked_q[c][cl]) ||
                        ((park_hold_q[c][cl] || park_parked_q[c][cl]) && fenced_seen_q[c][cl])) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d park hold %0b parked %0b while fenced %0b",
                               c, cl, park_hold_q[c][cl], park_parked_q[c][cl], fenced_i[c][cl]);
                    end
                end
            end
        end
    end
`endif

endmodule
