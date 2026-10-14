// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level 3, executor side (bingo_hw_manager_top SubstituteLevelMask[2]): tasks
// that another chiplet exported to this one, and their dones and rejects back
// to it.
//
// An imported task goes to the lowest slot of its CoreTypeId (its home), or
// to the live stand-in of that slot (level 1/2 substitute, never re-exported,
// ImportSubstituteLevelMask), into its ready and checkout queues, as a task of
// the home slot, tagged with its origin. Its dependencies were checked on the
// origin chiplet and it sets none here. Imports wait while a local replay is
// pending or moving, and never collide with a routed or replayed push into
// the same slot.
// A task that no live core here may run, now or later (fenced is sticky), is
// taken and rejected back to its origin instead of blocking the link; so is
// an imported entry that a replay bounces (its core died and no other live
// core is left). The reject (reject_o) and the dones of retiring imported
// tasks go back to the origin through bingo_hw_manager_remote_reply, which
// holds one reject at a time (reject_pending_i).
// Without RemoteEn every output is 0.
module bingo_hw_manager_remote_import #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter bit          RemoteEn = 1'b0,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    parameter logic [2:0]  ImportSubstituteLevelMask = 3'b001,
    parameter logic [2**CoreTypeIdWidth-1:0] SubstituteL2TypeEn = '1,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter type         task_desc_t = logic,  // bingo_hw_manager_task_desc_t of the top
    parameter type         remote_tag_t = logic,
    parameter type         remote_done_t = logic,
    parameter type         chip_id_t = logic,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         remote_dispatch_t = logic, // with a task_desc_t
    parameter type         slot_state_t = logic,
    parameter type         replay_step_t = logic,
    parameter type         route_t = logic,
    parameter type         checkout_t = logic,
    parameter type         ready_t = logic,
    parameter type         import_t = logic
) (
    input  logic                                 clk_i,
    input  logic                                 rst_ni,
    input  chip_id_t                             chip_id_i,
    // From the transport; desc is the exported descriptor, as a task of this chiplet
    input  logic                                 dispatch_valid_i,
    output logic                                 dispatch_ready_o,
    input  remote_dispatch_t                     dispatch_i,
    // Where an import may go now: live slots (slot state), no replay pending
    // or moving (the replay step), no routed or replayed push into the slot,
    // room in its checkout and ready queues
    input  slot_state_t                          slot_i,
    input  logic                                 replay_pending_i,
    input  replay_step_t                         step_i,
    input  logic [NumCores-1:0][NumClusters-1:0] replay_push_i,
    input  route_t                               route_i,
    input  checkout_t                            checkout_i,
    input  ready_t                               ready_i,
    // The import taken this cycle (pushed into one slot, or rejected), its home
    // slot and the slot it runs on
    output import_t                              import_o,
    // A bounced imported entry (step_i.bounce: replay found no live core for it)
    input  remote_tag_t                          bounce_tag_i,
    input  task_desc_t                           bounce_desc_i,
    // A reject for the origin: of the import taken this cycle, or of a bounced
    // entry; and a reject still waits (none is taken meanwhile)
    output logic                                 reject_valid_o,
    output remote_done_t                         reject_o,
    input  logic                                 reject_pending_i
);
    // The import taken this cycle (import_o)
    logic                                 import_fire, import_reject;
    logic [NumCores-1:0][NumClusters-1:0] import_push;
    task_desc_t                           import_desc;
    remote_tag_t                          import_tag;
    logic [CoreIdWidth-1:0]               home_core, import_core;
    logic [ClusterIdWidth-1:0]            home_cluster, import_cluster;
    assign import_o = '{fire:         import_fire,
                        reject:       import_reject,
                        push:         import_push,
                        desc:         import_desc,
                        tag:          import_tag,
                        home_core:    home_core,
                        home_cluster: home_cluster,
                        core:         import_core,
                        cluster:      import_cluster};


    if (RemoteEn) begin : gen_import
        logic home_found, sub_found;
        // Import: home slot = lowest slot of the requested type
        always_comb begin : find_import_home
            home_found     = 1'b0;
            home_core    = '0;
            home_cluster = '0;
            for (int cl = NumClusters - 1; cl >= 0; cl--) begin
                for (int c = NumCores - 1; c >= 0; c--) begin
                    if ((dispatch_i.core_type != '0) &&
                        (CoreTypeId[c][cl] == dispatch_i.core_type)) begin
                        home_found     = 1'b1;
                        home_core    = CoreIdWidth'(c);
                        home_cluster = ClusterIdWidth'(cl);
                    end
                end
            end
        end
        // Its live stand-in on this chiplet (never re-exported)
        bingo_hw_manager_substitute_sel #(
            .NumCores(NumCores),
            .NumClusters(NumClusters),
            .CoreIdWidth(CoreIdWidth),
            .ClusterIdWidth(ClusterIdWidth),
            .CoreTypeIdWidth(CoreTypeIdWidth),
            .CoreTypeId(CoreTypeId),
            .LevelMask(ImportSubstituteLevelMask & 3'b011),
            .L2TypeEn(SubstituteL2TypeEn)
        ) i_import_sel (
            .logical_core_i(home_core),
            .logical_cluster_i(home_cluster),
            .fenced_i(slot_i.fenced),
            .found_o(sub_found),
            .core_o(import_core),
            .cluster_o(import_cluster)
        );
        // A task of the home slot, checked on its origin chiplet: no dependencies here
        always_comb begin : compose_import_desc
            import_desc                     = dispatch_i.desc;
            import_desc.assigned_chiplet_id = chip_id_i;
            import_desc.assigned_cluster_id = home_cluster;
            import_desc.assigned_core_id    = home_core;
            import_desc.dep_check_info      = '0;
            import_desc.dep_set_info        = '0;
            import_desc.cond_exec_en        = 1'b0;
            import_desc.cond_exec_group_id  = '0;
            import_desc.cond_exec_invert    = 1'b0;
        end
        assign import_tag = '{exported:    1'b0,
                              imported:    1'b1,
                              origin_chip: dispatch_i.origin_chip,
                              proxy_slot:  dispatch_i.proxy_slot};
        assign import_fire = dispatch_valid_i && home_found && sub_found &&
                             !replay_pending_i && !(|step_i.move) &&
                             !route_i.valid[import_core][import_cluster] &&
                             !replay_push_i[import_core][import_cluster] &&
                             !checkout_i.full[import_core][import_cluster] &&
                             !ready_i.full[import_core][import_cluster];
        always_comb begin
            import_push = '0;
            import_push[import_core][import_cluster] = import_fire;
        end
        // No live core here may run it, now or later (fenced is sticky): take it
        // and reject it back to its origin instead of blocking the link. Waits
        // for a local replay, which may first bounce older imports of the type.
        assign import_reject = dispatch_valid_i && !(home_found && sub_found) &&
                               !replay_pending_i && !(|step_i.move) && !reject_pending_i;
        assign dispatch_ready_o = import_fire || import_reject;

        // A reject for the origin: the import taken this cycle, else a bounced
        // imported entry
        assign reject_valid_o = import_reject || step_i.bounce;
        always_comb begin
            if (import_reject) begin
                reject_o = '{chip:       dispatch_i.origin_chip,
                             proxy_slot: dispatch_i.proxy_slot,
                             task_id:    import_desc.task_id,
                             reject:     1'b1};
            end else begin
                reject_o = '{chip:       bounce_tag_i.origin_chip,
                             proxy_slot: bounce_tag_i.proxy_slot,
                             task_id:    bounce_desc_i.task_id,
                             reject:     1'b1};
            end
        end
    end else begin : gen_no_import
        assign dispatch_ready_o    = 1'b0;
        assign import_fire         = 1'b0;
        assign import_reject       = 1'b0;
        assign import_push         = '0;
        assign import_desc         = '0;
        assign import_tag          = '0;
        assign home_core           = '0;
        assign home_cluster        = '0;
        assign import_core         = '0;
        assign import_cluster      = '0;
        assign reject_valid_o      = 1'b0;
        assign reject_o            = '0;
    end
endmodule
