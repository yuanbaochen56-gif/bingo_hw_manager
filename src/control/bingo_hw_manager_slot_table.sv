// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Slot mapping table and core parking of the control plane
// (bingo_hw_manager_ctrl).
//
// Slot mapping table (SMT): for every logical slot L = (core, cluster), the live
// slot that runs L's tasks once L is fenced (its substitute), or none
// (smt_o.found = 0). bingo_hw_manager_core_remap (new tasks) and
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
//   SubstitutePolicy 1  least loaded live slot of the type (queue_i.load: the
//                       occupancy of its checkout queue), logical cluster first,
//                       lowest index on a tie. The load is sampled when the
//                       entry is recomputed, i.e. when the core dies.
//
// In the cycle a slot is fenced, the cycle a live slot is parked, and the cycle
// after reset, the table is being written: smt_update_o asks its readers to
// wait one cycle. smt_recompute_o marks the entries recomputed in that cycle
// (after reset, or because their slot or their substitute was just fenced).
//
// Core parking (park_req_i: the host request, or an at-risk slot with the park
// action, see bingo_hw_manager_ctrl; one bit per logical slot; 0 = off, and the
// machine then matches a build without parking). A requested slot that is
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
// type releases HOLD and raises park_o.fail; the bit stays until the request
// drops, and the request is not tried again while it stays set. A slot that
// runs the tasks of another logical slot (the table target of a fenced or
// parked slot) cannot drain on its own: a request then fails at once, and a
// slot that becomes such a target while in HOLD gives up the same way, so its
// own tasks are not held behind the other slot's. A fence of the
// slot itself drops HOLD or PARKED and writes the table only through the fence
// update above, never as a park write in that same cycle. Clearing park_req_i
// of a PARKED slot moves it back (UNPARK): its new tasks are held, on neither
// core, until none of its tasks is left on another core (queue_i.moved, the per
// logical core count of entries in other cores' checkout queues), so a task on
// the slot itself cannot overtake an older one on the substitute; then the
// entry goes back to the plain choice and the slot is no longer parked. The
// substitute keeps running its own tasks meanwhile: its queue is in order, so
// they never delay the parked slot's. Setting the request again during UNPARK
// returns to PARKED. If the substitute of a parked slot is fenced, the
// entry is recomputed with the slot still masked out; if nothing is left, the
// slot leaves PARKED, park_o.fail is raised, and the table points back at the
// slot so a replay of entries already on the dead substitute has a live core.
//
// Fence fallback (type_cfg_i.fence_fb, host-configured per core type, 0 = off):
// a type whose bit is set takes no substitute, as if it had no other slot. The
// table entry of a fenced slot of that type is written as none, and a park of
// such a slot fails as with no live core of the type. The fenced slot then becomes
// stuck, so its outstanding and later tasks are left to the CERF degradation
// (bingo_hw_manager_cerf_fallback; e.g. a host backup that is cheaper than a
// replay on another cluster).
// The top keeps such a type from level 3 as well. The entries of live slots do
// not depend on the bit. The table samples it when an entry is written, like
// the load: set it before the first fence or park.
module bingo_hw_manager_slot_table #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    parameter logic [2:0] SubstituteLevelMask = 3'b001,
    parameter logic [2**CoreTypeIdWidth-1:0] SubstituteL2TypeEn = '1,
    parameter int unsigned SubstitutePolicy = 0,
    parameter int unsigned LoadWidth = 4,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         slot_state_t = logic,
    parameter type         ctrl_queue_t = logic,
    parameter type         type_cfg_t = logic,
    parameter type         smt_t = logic,
    parameter type         park_t = logic
) (
    input  logic clk_i,
    input  logic rst_ni,

    // Slot state (fenced); queues (checkout and ready empty, push, moved, load)
    input  slot_state_t                          slot_i,
    input  ctrl_queue_t                          queue_i,
    // Fence fallback per core type, park request per slot
    input  type_cfg_t                            type_cfg_i,
    input  logic [NumCores-1:0][NumClusters-1:0] park_req_i,

    // The table, the cycle it is written, and the entries recomputed this cycle
    output smt_t                                 smt_o,
    output logic                                 smt_update_o,
    output logic [NumCores-1:0][NumClusters-1:0] smt_recompute_o,
    output park_t                                park_o
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
                .fenced_i(slot_i.fenced),
                .weight_i(queue_i.load),
                .found_o(choice_any[c][cl]),
                .core_o(choice_core[c][cl]),
                .cluster_o(choice_cluster[c][cl])
            );
            // Fence fallback: a fenced slot of such a type has no substitute
            assign choice_found[c][cl] = choice_any[c][cl] &&
                                         !(slot_i.fenced[c][cl] && type_cfg_i.fence_fb[CoreTypeId[c][cl]]);
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
                fenced_excl = slot_i.fenced | park_hold_q | park_parked_q;
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
                .weight_i(queue_i.load),
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

    assign fence_new = slot_i.fenced & ~fenced_seen_q;

    // Slots that are the table target of a fenced or parked logical slot: they
    // receive that slot's tasks, so they cannot drain on their own
    always_comb begin
        park_target = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (smt_found_q[c][cl] && (slot_i.fenced[c][cl] || park_parked_q[c][cl]) &&
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
                    !type_cfg_i.fence_fb[CoreTypeId[c][cl]] &&
                    !((park_choice_core[c][cl] == CoreIdWidth'(c)) &&
                      (park_choice_cluster[c][cl] == ClusterIdWidth'(cl)));
                park_drained[c][cl] = queue_i.checkout_empty[c][cl] && queue_i.ready_empty[c][cl] &&
                                      !queue_i.push[c][cl];
                // Not in the cycle this slot is fenced: that write belongs to the fence.
                park_commit[c][cl] = park_hold_q[c][cl] && !park_parked_q[c][cl] &&
                                     !slot_i.fenced[c][cl] && !fence_new[c][cl] && !park_target[c][cl] &&
                                     park_drained[c][cl] && park_choice_ok[c][cl];
                unpark_commit[c][cl] = park_parked_q[c][cl] && park_unpark_q[c][cl] &&
                                       !slot_i.fenced[c][cl] && !fence_new[c][cl] && !recompute[c][cl] &&
                                       !queue_i.moved[c][cl];
                park_give_up[c][cl] = park_hold_q[c][cl] && !park_parked_q[c][cl] &&
                                      !slot_i.fenced[c][cl] && !fence_new[c][cl] &&
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
            fenced_seen_q <= slot_i.fenced;
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

                    if (slot_i.fenced[c][cl]) begin
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

    assign smt_o.found   = smt_found_q;
    assign smt_o.core    = smt_core_q;
    assign smt_o.cluster = smt_cluster_q;
    assign smt_update_o  = init_q || (|fence_new) || (|park_commit) || (|unpark_commit);
    assign park_o.hold   = park_hold_q;
    assign park_o.parked = park_parked_q;
    assign park_o.unpark = park_unpark_q;
    assign park_o.fail   = park_fail_q;
    assign smt_recompute_o = recompute;

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
                    end else if (!slot_i.fenced[c][cl] && smt_found_q[c][cl] &&
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
                        (slot_i.fenced[smt_core_q[c][cl]][smt_cluster_q[c][cl]] ||
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
                               c, cl, park_hold_q[c][cl], park_parked_q[c][cl], slot_i.fenced[c][cl]);
                    end
                end
            end
        end
    end
`endif

endmodule
