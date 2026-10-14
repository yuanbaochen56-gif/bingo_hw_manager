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
// Frequency-aware watchdog (bingo_hw_manager_wd_tick): a slot's watchdog timer
// counts cycles of the normal clock while its domain runs slower (wd_tick_o),
// and wd_fast_o marks a domain that runs faster than the normal level.
//
// Recovery boost: a live slot that is the substitute of a fenced core in the
// table does the work of two cores. While it is busy (not polling), pm_boost_o
// asks the PM to run its domain at the boost level (bingo_hw_manager_pm
// boost_power_level_i, a faster clock than the normal level; 0 = off).
// Capacity boost (boost_policy_i = 1) replaces that choice: a core type has
// lost capacity while one of its slots is fenced, stuck, rejected or parked at
// risk. Every busy live slot of such a type with a load of at least
// boost_load_min_i is a candidate (type 0 substitutes for nobody and is never
// one; neither is a slot in a derated domain). At most boost_credit_i domains
// per lost slot run boosted
// (0 = no limit), the lowest domain index first: the dead or parked cores no
// longer draw power, and the survivors of their type get that budget.
//
// Fence fallback (fence_fb_i, host-configured per core type, 0 = off): a type
// whose bit is set takes no substitute, as if it had no other slot. The table
// entry of a fenced slot of that type is written as none, and a park of such a
// slot fails as with no live core of the type. The fenced slot then becomes
// stuck, so its outstanding and later tasks are left to the CERF degradation
// below (e.g. a host backup that is cheaper than a replay on another cluster).
// The top keeps such a type from level 3 as well. The entries of live slots do
// not depend on the bit. The table samples it when an entry is written, like
// the load: set it before the first fence or park.
//
// CERF degradation (bingo_hw_manager_cerf_fallback): while a core type has a
// stuck or rejected slot, one host-configured CERF update per enabled type.
//
// Fault precursors (risk, bingo_hw_manager_risk): a slot whose late beats
// (late_i) add up faster than they decay is at risk until the host clears it
// (risk_clear_i) or the slot is fenced.
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
    // Substitute choice (see above) and width of load_i
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
    // Fence fallback: core types that take no substitute (see above). 0 = off.
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
    // Boost choice (see above): 0 = substitutes of fenced cores, 1 = capacity;
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
    output logic [2**CoreTypeIdWidth-1:0]            cerf_fb_evt_o,
    // At-risk slots, and the ones whose domain runs at most at the derate level
    output logic [NumCores-1:0][NumClusters-1:0]     risk_o,
    output logic [NumCores-1:0][NumClusters-1:0]     pm_derate_o
);

    // ------------------------------------------------------------------
    // Candidate choice per logical slot over the current fenced set
    // ------------------------------------------------------------------
    logic [NumCores-1:0][NumClusters-1:0]                     choice_any;
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
                .L2TypeEn(SubstituteL2TypeEn),
                .LeastWeight(SubstitutePolicy == 1),
                .WeightWidth(LoadWidth)
            ) i_choice (
                .logical_core_i(CoreIdWidth'(c)),
                .logical_cluster_i(ClusterIdWidth'(cl)),
                .fenced_i(fenced_i),
                .weight_i(load_i),
                .found_o(choice_any[c][cl]),
                .core_o(choice_core[c][cl]),
                .cluster_o(choice_cluster[c][cl])
            );
            // Fence fallback: a fenced slot of such a type has no substitute
            assign choice_found[c][cl] = choice_any[c][cl] &&
                                         !(fenced_i[c][cl] && fence_fb_i[CoreTypeId[c][cl]]);
        end
    end

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
                .L2TypeEn(SubstituteL2TypeEn),
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
                // Fence fallback: such a type has no parking substitute either
                park_choice_ok[c][cl] = park_choice_found[c][cl] &&
                    !fence_fb_i[CoreTypeId[c][cl]] &&
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
                    end else if (park_hold_q[c][cl] && !park_req[c][cl]) begin
                        park_hold_q[c][cl] <= 1'b0;
                    end else if (!park_hold_q[c][cl] && !park_parked_q[c][cl] &&
                                 !park_fail_q[c][cl] && park_req[c][cl]) begin
                        // A slot that runs another slot's tasks cannot drain: fail at once
                        if (park_target[c][cl]) park_fail_q[c][cl] <= 1'b1;
                        else                    park_hold_q[c][cl] <= 1'b1;
                    end else if (park_fail_q[c][cl] && !park_req[c][cl]) begin
                        park_fail_q[c][cl] <= 1'b0;
                    end else if (park_parked_q[c][cl] && recompute[c][cl] &&
                                 !fence_new[c][cl] && !init_q && !park_choice_ok[c][cl]) begin
                        park_parked_q[c][cl] <= 1'b0;
                        park_unpark_q[c][cl] <= 1'b0;
                        park_fail_q[c][cl]   <= 1'b1;
                    end else if (unpark_commit[c][cl]) begin
                        park_parked_q[c][cl] <= 1'b0;
                        park_unpark_q[c][cl] <= 1'b0;
                    end else if (park_parked_q[c][cl] && !park_unpark_q[c][cl] && !park_req[c][cl]) begin
                        park_unpark_q[c][cl] <= 1'b1;
                    end else if (park_unpark_q[c][cl] && park_req[c][cl]) begin
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

    // A fence writes the SMT first. On the following edge, load its current
    // live substitute, unless the logical slot has already retired. Include
    // affected dead entries when their previous substitute is fenced again.
    logic [NumCores-1:0][NumClusters-1:0] rec_pending_q, rec_load;
    logic [NumCores-1:0][NumClusters-1:0][31:0] rec_hold_q, rec_hold_d;
    always_comb begin
        rec_load = '0;
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                if (rec_pending_q[c][cl] && !retired_i[c][cl] && smt_found_q[c][cl] &&
                    !fenced_i[smt_core_q[c][cl]][smt_cluster_q[c][cl]])
                    rec_load[smt_core_q[c][cl]][smt_cluster_q[c][cl]] = 1'b1;
            end
        end
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                rec_hold_d[c][cl] = rec_hold_q[c][cl];
                if (fenced_i[c][cl]) rec_hold_d[c][cl] = '0;
                else if (rec_load[c][cl] && recovery_hold_i != 0)
                    rec_hold_d[c][cl] = recovery_hold_i;
                else if (rec_hold_q[c][cl] != 0)
                    rec_hold_d[c][cl] = rec_hold_q[c][cl] - 32'd1;
                // The logger samples this on the very edge the count changes,
                // so pending-event ages refer to load/zero, not one edge later.
                rec_hold_o[c][cl] = rec_hold_d[c][cl] != 0;
            end
        end
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rec_pending_q <= '0;
            rec_hold_q <= '0;
        end else begin
            rec_pending_q <= recompute & fenced_i & ~retired_i;
            rec_hold_q <= rec_hold_d;
        end
    end

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
                access_only_o[c][cl] = slot_idle[c][cl] && (idle_cnt_q[c][cl] >= idle_delay_i) &&
                                       cluster_awake[cl] && rec_hold_q[c][cl] == 0;
                pm_idle_o[c][cl] &= rec_hold_q[c][cl] == 0;
            end
        end
    end
    assign load_clear_o = fenced_i;

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
                if (fenced_i[c][cl] && smt_found_q[c][cl] &&
                    ((smt_core_q[c][cl] != c) || (smt_cluster_q[c][cl] != cl))) begin
                    boost_sub[smt_core_q[c][cl]][smt_cluster_q[c][cl]] = 1'b1;
                end
            end
        end
        boost_sub = boost_sub & ~waiting_i & ~fenced_i;

        lost_slot = fenced_i | stuck_i | rejected_i | (risk_q & park_parked_q);
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
                if (!waiting_i[c][cl] && !lost_slot[c][cl] && !park_parked_q[c][cl] &&
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
                pm_derate_o[c][cl] = risk_q[c][cl] && risk_action_i[1] && !fenced_i[c][cl] &&
                                     !park_parked_q[c][cl] &&
                                     !(risk_action_i[0] && ParkSupported && !park_fail_q[c][cl]);
            end
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk_i) begin
        if (rst_ni) begin
            for (int c = 0; c < NumCores; c++)
                for (int cl = 0; cl < NumClusters; cl++)
                    if ((rec_hold_q[c][cl] != 0) != (rec_hold_d[c][cl] != 0))
                        $display("[BINGO_RECOVERY_HOLD] %0t core=%0d cluster=%0d active=%0b",
                                 $time, c, cl, rec_hold_d[c][cl] != 0);
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
        if (rst_ni && cerf_fb_done_i && cerf_fb_req_o) begin
            $display("[BINGO_CERF_FB] %0t type %0d clear g%0d set g%0d",
                     $time, cerf_fb_type_o, cerf_fb_clear_o, cerf_fb_set_o);
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
