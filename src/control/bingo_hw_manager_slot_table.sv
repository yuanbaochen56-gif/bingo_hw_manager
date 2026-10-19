// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Slot mapping table (SMT) and core parking (bingo_hw_manager_ctrl).
//
// SMT: for each logical slot L = (core, cluster), the live substitute that
// runs L's tasks once L is fenced or parked, or none (smt_o.found = 0). Both
// bingo_hw_manager_core_remap (new tasks) and bingo_hw_manager_replay_ctrl
// (outstanding tasks) read it so all tasks of L go to one substitute in order.
//
// Entries are registered so a policy can depend on state at the moment of the
// update without switching a core's substitute later:
//   SubstitutePolicy 0  lowest live slot of the type (logical cluster first)
//   SubstitutePolicy 1  least loaded live slot (queue_i.load, logical cluster
//                       first, lowest index on a tie), sampled on write
// Spare clusters (sub_cfg_i.spare) are searched before the logical cluster;
// cross-cluster candidates require a shared partition (sub_cfg_i.partition).
// If type_cfg_i.fence_fb is set for a core type, its fenced slots take no
// substitute (leaving tasks to CERF degradation) and cannot be parked. Both
// sub_cfg_i and fence_fb take effect when an entry is written; set them before
// the first fence or park.
//
// An entry is recomputed after reset or when its slot or current substitute is
// newly fenced (smt_recompute_o), and is also written on park/unpark commits.
// With policy 0, removing a slot that is not the chosen one never changes a
// choice, so the table then equals the combinational choice (checked in
// simulation). Whenever the table is written, smt_update_o tells readers to
// wait one cycle.
//
// Core parking (park_req_i): a requested live slot that is not another slot's
// substitute enters HOLD, drains its checkout and ready queues, and picks a
// live same-type substitute with itself and all HOLD/PARKED slots masked out
// (never level 3). It then enters PARKED and writes the substitute into the
// table, or raises park_o.fail until the request drops. Clearing park_req_i
// enters UNPARK, holding new tasks until queue_i.moved == 0 so no new task
// overtakes an older one on the substitute, then restores the plain table
// entry. The states of one slot are kept in bingo_hw_manager_park_fsm.
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
    // Fence fallback per core type, substitute choice, park request per slot
    input  type_cfg_t                            type_cfg_i,
    input  bingo_hw_manager_pkg::sub_cfg_t       sub_cfg_i,
    input  logic [NumCores-1:0][NumClusters-1:0] park_req_i,

    // The table, the cycle it is written, and the entries recomputed this cycle
    output smt_t                                 smt_o,
    output logic                                 smt_update_o,
    output logic [NumCores-1:0][NumClusters-1:0] smt_recompute_o,
    output park_t                                park_o
);

    // ------------------------------------------------------------------
    // Substitute choice from the host
    // ------------------------------------------------------------------
    localparam int unsigned PartBits = bingo_hw_manager_pkg::SubPartBits;
    localparam int unsigned PartClusters = (NumClusters < 32 / PartBits) ? NumClusters : 32 / PartBits;
    logic [NumClusters-1:0] spare;
    assign spare = sub_cfg_i.spare[NumClusters-1:0];
    // The partitions of every cluster (the ones beyond the field: all), and
    // part_ok[l][cl]: a slot of cluster cl may stand in for a slot of cluster l
    // at level 2 (the selector never asks it for l itself: the own cluster is
    // level 1, which partitions do not restrict)
    logic [NumClusters-1:0][PartBits-1:0]    part_of;
    logic [NumClusters-1:0][NumClusters-1:0] part_ok;
    for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_part_of
        if (cl < PartClusters) begin : gen_in_field
            assign part_of[cl] = sub_cfg_i.partition[PartBits * cl +: PartBits];
        end else begin : gen_beyond
            assign part_of[cl] = '1;
        end
    end
    always_comb begin
        for (int unsigned l = 0; l < NumClusters; l++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                part_ok[l][cl] = (part_of[l] & part_of[cl]) != '0;
            end
        end
    end

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
                .spare_i(spare),
                .part_ok_i(part_ok[cl]),
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
    logic [NumCores-1:0][NumClusters-1:0]                     park_hold;
    logic [NumCores-1:0][NumClusters-1:0]                     park_parked;
    logic [NumCores-1:0][NumClusters-1:0]                     park_unpark;
    logic [NumCores-1:0][NumClusters-1:0]                     park_fail;
    logic [NumCores-1:0][NumClusters-1:0]                     park_choice_found;
    logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    park_choice_core;
    logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] park_choice_cluster;

    for (genvar c = 0; c < NumCores; c++) begin : gen_park_choice_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_park_choice_cluster
            logic [NumCores-1:0][NumClusters-1:0] fenced_excl;
            always_comb begin
                fenced_excl = slot_i.fenced | park_hold | park_parked;
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
                .spare_i(spare),
                .part_ok_i(part_ok[cl]),
                .fenced_i(fenced_excl),
                .weight_i(queue_i.load),
                .found_o(park_choice_found[c][cl]),
                .core_o(park_choice_core[c][cl]),
                .cluster_o(park_choice_cluster[c][cl])
            );
        end
    end

    // ------------------------------------------------------------------
    // Table state, and the entries recomputed this cycle
    // ------------------------------------------------------------------
    logic                                                     init_q;
    logic [NumCores-1:0][NumClusters-1:0]                     fenced_seen_q;
    logic [NumCores-1:0][NumClusters-1:0]                     smt_found_q;
    logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_q;
    logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_q;
    logic [NumCores-1:0][NumClusters-1:0]                     fence_new;
    logic [NumCores-1:0][NumClusters-1:0]                     recompute;
    logic [NumCores-1:0][NumClusters-1:0]                     sub_fenced;

    assign fence_new = slot_i.fenced & ~fenced_seen_q;

    // An entry is recomputed after reset, when its slot is newly fenced, or when
    // its substitute is (sub_fenced: for that last reason only)
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                recompute[c][cl] = init_q || fence_new[c][cl] ||
                    (smt_found_q[c][cl] && fence_new[smt_core_q[c][cl]][smt_cluster_q[c][cl]]);
                sub_fenced[c][cl] = recompute[c][cl] && !fence_new[c][cl] && !init_q;
            end
        end
    end

    // ------------------------------------------------------------------
    // Parking state of every logical slot
    // ------------------------------------------------------------------
    logic [NumCores-1:0][NumClusters-1:0] park_target;
    logic [NumCores-1:0][NumClusters-1:0] park_choice_ok;
    logic [NumCores-1:0][NumClusters-1:0] park_commit;
    logic [NumCores-1:0][NumClusters-1:0] unpark_commit;

    // Slots that are the table target of a fenced or parked logical slot: they
    // receive that slot's tasks, so they cannot drain on their own
    always_comb begin
        park_target = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (smt_found_q[c][cl] && (slot_i.fenced[c][cl] || park_parked[c][cl]) &&
                    ((smt_core_q[c][cl] != CoreIdWidth'(c)) || (smt_cluster_q[c][cl] != ClusterIdWidth'(cl)))) begin
                    park_target[smt_core_q[c][cl]][smt_cluster_q[c][cl]] = 1'b1;
                end
            end
        end
    end

    for (genvar c = 0; c < NumCores; c++) begin : gen_park_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_park_cluster
            // Fence fallback: such a type has no parking substitute either
            assign park_choice_ok[c][cl] = park_choice_found[c][cl] &&
                                           !type_cfg_i.fence_fb[CoreTypeId[c][cl]] &&
                                           !((park_choice_core[c][cl] == CoreIdWidth'(c)) &&
                                             (park_choice_cluster[c][cl] == ClusterIdWidth'(cl)));
            bingo_hw_manager_park_fsm i_park (
                .clk_i(clk_i),
                .rst_ni(rst_ni),
                .req_i(park_req_i[c][cl]),
                .fenced_i(slot_i.fenced[c][cl]),
                .target_i(park_target[c][cl]),
                .drained_i(queue_i.checkout_empty[c][cl] && queue_i.ready_empty[c][cl] &&
                           !queue_i.push[c][cl]),
                .choice_ok_i(park_choice_ok[c][cl]),
                .recompute_i(recompute[c][cl]),
                .sub_fenced_i(sub_fenced[c][cl]),
                .moved_i(queue_i.moved[c][cl]),
                .hold_o(park_hold[c][cl]),
                .parked_o(park_parked[c][cl]),
                .unpark_o(park_unpark[c][cl]),
                .fail_o(park_fail[c][cl]),
                .commit_o(park_commit[c][cl]),
                .unpark_commit_o(unpark_commit[c][cl])
            );
        end
    end

    // ------------------------------------------------------------------
    // Table update: a new fence, or a park or unpark commit
    // ------------------------------------------------------------------
    // What the table entry of a logical slot is written with this cycle
    typedef enum logic [1:0] {
        SmtKeep,        // no write
        SmtChoice,      // the plain choice (choice_*): after reset, a fence of the
                        // slot or of its substitute, or the end of an UNPARK
        SmtParkChoice,  // the parking choice (park_choice_*): a park commit, or a
                        // parked slot whose substitute was just fenced
        SmtSelf         // the slot itself. Only for a parked slot whose substitute
                        // was just fenced when no other is left: the slot leaves
                        // PARKED (fail), and the replay of its tasks already on the
                        // dead substitute has a live core to go to. Read nowhere
                        // else: a live slot's entry is unread until the slot is
                        // fenced or parked, and both write it again.
    } smt_write_e;
    smt_write_e [NumCores-1:0][NumClusters-1:0] smt_write;

    // A recompute (reset, a fence of the slot or of its substitute) wins over a
    // park or unpark commit
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (recompute[c][cl]) begin
                    if (park_parked[c][cl] && sub_fenced[c][cl]) begin
                        smt_write[c][cl] = park_choice_ok[c][cl] ? SmtParkChoice : SmtSelf;
                    end else begin
                        smt_write[c][cl] = SmtChoice;
                    end
                end else if (park_commit[c][cl]) begin
                    smt_write[c][cl] = SmtParkChoice;
                end else if (unpark_commit[c][cl]) begin
                    smt_write[c][cl] = SmtChoice;
                end else begin
                    smt_write[c][cl] = SmtKeep;
                end
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
        end else begin
            init_q        <= 1'b0;
            fenced_seen_q <= slot_i.fenced;
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    case (smt_write[c][cl])
                        SmtChoice: begin
                            smt_found_q[c][cl]   <= choice_found[c][cl];
                            smt_core_q[c][cl]    <= choice_core[c][cl];
                            smt_cluster_q[c][cl] <= choice_cluster[c][cl];
                        end
                        SmtParkChoice: begin
                            smt_found_q[c][cl]   <= 1'b1;
                            smt_core_q[c][cl]    <= park_choice_core[c][cl];
                            smt_cluster_q[c][cl] <= park_choice_cluster[c][cl];
                        end
                        SmtSelf: begin
                            smt_found_q[c][cl]   <= 1'b1;
                            smt_core_q[c][cl]    <= CoreIdWidth'(c);
                            smt_cluster_q[c][cl] <= ClusterIdWidth'(cl);
                        end
                        default: ;  // SmtKeep
                    endcase
                end
            end
        end
    end

    assign smt_o.found     = smt_found_q;
    assign smt_o.core      = smt_core_q;
    assign smt_o.cluster   = smt_cluster_q;
    assign smt_update_o    = init_q || (|fence_new) || (|park_commit) || (|unpark_commit);
    assign smt_recompute_o = recompute;
    assign park_o.hold     = park_hold;
    assign park_o.parked   = park_parked;
    assign park_o.unpark   = park_unpark;
    assign park_o.fail     = park_fail;

`ifndef SYNTHESIS
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
                    if (recompute[c][cl] || park_commit[c][cl] || unpark_commit[c][cl]) sub_stale_q[c][cl] <= 1'b0;
                    else if (sub_cfg_i != sub_seen_q)                                  sub_stale_q[c][cl] <= 1'b1;
                end
            end
        end
    end

    // Outside an update cycle the table equals the combinational choice (only
    // for the lowest-index policy: a load-based choice depends on the load when
    // the core died; and not for an entry kept from before a change of sub_cfg_i)
    always @(posedge clk_i) begin : smt_equivalence
        if (rst_ni && !smt_update_o && (SubstitutePolicy == 0)) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (sub_stale_q[c][cl] || (sub_cfg_i != sub_seen_q)) continue;
                    // A parked entry was chosen among the slots that were neither in
                    // HOLD nor PARKED at the time, so it is only checked for validity
                    // (below and here: another slot, never itself, never parked).
                    if (park_parked[c][cl]) begin
                        if (!smt_found_q[c][cl] ||
                            ((smt_core_q[c][cl] == CoreIdWidth'(c)) && (smt_cluster_q[c][cl] == ClusterIdWidth'(cl))) ||
                            park_parked[smt_core_q[c][cl]][smt_cluster_q[c][cl]]) begin
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
                    // Parking ends on the edge that sees the fence, so a slot fenced
                    // since the previous edge must no longer be in HOLD or PARKED.
                    if ((park_hold[c][cl] || park_parked[c][cl]) && fenced_seen_q[c][cl]) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d park hold %0b parked %0b while fenced %0b",
                               c, cl, park_hold[c][cl], park_parked[c][cl], slot_i.fenced[c][cl]);
                    end
                end
            end
        end
    end
`endif

endmodule
