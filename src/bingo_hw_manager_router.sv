// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Placement of new tasks: which physical slot each waiting queue head enters,
// and in what form.
//
// Each logical core has a waiting queue; its head, once its dependencies are
// checked, is offered to the physical cluster chosen for it (offer_valid_i,
// the top's demux). The slot is the logical one, unless the control plane
// moved the logical slot's tasks to a substitute (retired or parked, see
// bingo_hw_manager_core_remap), or level 3 exports them (remote): then the
// task stays on its proxy slot and is only copied to another chiplet.
// A dummy-set or CERF-skipped task never executes on a core, so it stays on
// its logical core and must drain through that core's checkout queue, which
// orders it after the core's earlier tasks; it is held back until none of
// those is left in another slot. A fenced logical core (or one with tasks in
// another slot) is also held while a replay is pending: its older tasks may
// sit in the slot being replayed and must reach their new core first.
//
// Per destination slot, one offer is taken per cycle (the lowest logical
// core), never while a replay MOVE may push into its cluster. The routed task
// enters the slot's checkout queue and, if it runs on a core, its ready queue
// (ready_push_o). It runs on no core when it is a dummy set, the CERF skips it
// (skip_i), it is exported (route_export_o, once granted) or it is an
// exit absorbed into a dummy (route_exit_absorb_o): an exit task that does not
// run on its own logical slot, so that slot's core still sees its own exit.
module bingo_hw_manager_router #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    parameter logic [2:0]  SubstituteLevelMask = 3'b001,
    parameter bit          RemoteEn = 1'b0,
    parameter int unsigned OutstandingWidth = 1,
    parameter type         task_desc_t = logic   // bingo_hw_manager_task_desc_t of the top
) (
    // Waiting queue heads, one per logical core
    input  task_desc_t [NumCores-1:0]                   waiting_desc_i,
    input  logic       [NumCores-1:0]                   waiting_empty_i,
    input  logic       [NumCores-1:0]                   skip_i,          // the CERF skips the head
    // The checked head of each logical core, offered to one physical cluster
    input  logic [NumCores-1:0][NumClusters-1:0]        offer_valid_i,
    output logic [NumCores-1:0][NumClusters-1:0]        offer_ready_o,
    // Slot state (watchdog, replay controller, control plane, level 3)
    input  logic [NumCores-1:0][NumClusters-1:0]        fenced_i,
    input  logic [NumCores-1:0][NumClusters-1:0]        retired_i,
    input  logic [NumCores-1:0][NumClusters-1:0]        stuck_i,
    input  logic [NumCores-1:0][NumClusters-1:0]        rejected_i,
    input  logic [2**CoreTypeIdWidth-1:0]               remote_type_en_i,
    input  logic [NumCores-1:0][NumClusters-1:0]                     smt_found_i,
    input  logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_i,
    input  logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_i,
    input  logic                                        smt_update_i,
    input  logic [NumCores-1:0][NumClusters-1:0]        park_hold_i,
    input  logic [NumCores-1:0][NumClusters-1:0]        park_parked_i,
    input  logic [OutstandingWidth-1:0]                 outstanding_i [NumCores][NumClusters],
    input  logic                                        replay_pending_i,
    input  logic [NumClusters-1:0]                      replay_move_cluster_i,
    // Selection per logical core: valid, physical slot, exported
    output logic [NumCores-1:0]                         select_valid_o,
    output logic [NumCores-1:0][CoreIdWidth-1:0]        physical_core_o,
    output logic [NumCores-1:0][ClusterIdWidth-1:0]     physical_cluster_o,
    // Per destination slot: a task is routed to it, from the waiting queue of
    // that logical core (each consumer reads the waiting queue head itself)
    output logic       [NumCores-1:0][NumClusters-1:0]                  route_valid_o,
    output logic       [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0] route_src_core_o,
    output logic       [NumCores-1:0][NumClusters-1:0]                  route_exit_absorb_o,
    output logic       [NumCores-1:0][NumClusters-1:0]                  route_export_o,
    // Room in the slot's queues, and the grant of an export (bingo_hw_manager_remote_export)
    input  logic       [NumCores-1:0][NumClusters-1:0]                  checkout_full_i,
    input  logic       [NumCores-1:0][NumClusters-1:0]                  ready_full_i,
    input  logic       [NumCores-1:0][NumClusters-1:0]                  export_grant_i,
    // The routed task enters the checkout queue (route_fire_o) and, if it runs
    // on a core, the ready queue (ready_push_o, while the ready queue has room)
    output logic       [NumCores-1:0][NumClusters-1:0]                  route_fire_o,
    output logic       [NumCores-1:0][NumClusters-1:0]                  ready_push_o
);
    import bingo_hw_manager_pkg::*;

    // Per logical core (waiting queue head): 1 if the task executes on a core
    // and may therefore be remapped (not a dummy-set, not CERF-skipped).
    logic [NumCores-1:0] remap_remappable;
    logic [NumCores-1:0] remap_select_valid_raw;
    logic [NumCores-1:0] remap_remote;

    // A task only leaves its logical core once that core is retired (fenced and
    // its outstanding tasks replayed) or PARKED (live, drained, see
    // bingo_hw_manager_core_remap).
    // Dummy-set and CERF-skipped tasks never execute on a core and must drain
    // through their own core's checkout FIFO, which is what orders them after the
    // core's earlier tasks, so they are never remapped.
    for (genvar core = 0; core < NumCores; core++) begin : gen_core_remap
        logic                      is_dummy_set;
        logic [ClusterIdWidth-1:0] task_cluster;
        logic                      outstanding_clear;
        logic                      replay_hold;

        assign is_dummy_set = (waiting_desc_i[core].task_type == TaskDummy) &&
                              waiting_desc_i[core].dep_set_info.dep_set_en;
        assign remap_remappable[core] = !is_dummy_set && !skip_i[core];

        bingo_hw_manager_core_remap #(
            .NumCores(NumCores),
            .NumClusters(NumClusters),
            .CoreIdWidth(CoreIdWidth),
            .ClusterIdWidth(ClusterIdWidth),
            .CoreTypeIdWidth(CoreTypeIdWidth),
            .CoreTypeId(CoreTypeId),
            .SubstituteLevelMask(SubstituteLevelMask)
        ) i_core_remap (
            .req_valid_i(!waiting_empty_i[core]),
            .logical_core_i(CoreIdWidth'(core)),
            .logical_cluster_i(waiting_desc_i[core].assigned_cluster_id),
            .remappable_i(remap_remappable[core]),
            .no_replay_i(waiting_desc_i[core].task_type == TaskNoReplay),
            .core_fenced_i(fenced_i),
            .core_retired_i(retired_i),
            .core_stuck_i(stuck_i),
            .remote_type_en_i(remote_type_en_i),
            .remote_rejected_i(rejected_i),
            .smt_found_i(smt_found_i),
            .smt_core_i(smt_core_i),
            .smt_cluster_i(smt_cluster_i),
            .smt_update_i(smt_update_i),
            .park_hold_i(park_hold_i),
            .park_parked_i(park_parked_i),
            .select_valid_o(remap_select_valid_raw[core]),
            .physical_core_o(physical_core_o[core]),
            .physical_cluster_o(physical_cluster_o[core]),
            .remote_o(remap_remote[core])
        );

        // A dummy-set / skipped task stays on its logical core, but earlier tasks of
        // that core may have been remapped (or replayed) elsewhere. Hold it back
        // until all of those have left their checkout queues, so that its dep_set
        // still fires after every earlier task of its logical core.
        // Healthy cores never have outstanding remapped tasks: no behaviour change.
        //
        // A fenced logical core is also held while any slot still waits to be
        // replayed: its older tasks may sit in that slot (in any cluster, after a
        // cross-cluster remap) and must reach their new core first. The same
        // holds for a live logical core whose tasks sit on another core (parked,
        // or back from a parked state after its substitute died): they may be
        // on the slot being replayed.
        assign task_cluster = waiting_desc_i[core].assigned_cluster_id;
        always_comb begin
            outstanding_clear = 1'b1;
            replay_hold       = 1'b0;
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (ClusterIdWidth'(cl) == task_cluster) begin
                    if (outstanding_i[core][cl] != '0) begin
                        outstanding_clear = 1'b0;
                    end
                    if ((fenced_i[core][cl] || (outstanding_i[core][cl] != '0)) && replay_pending_i) begin
                        replay_hold = 1'b1;
                    end
                end
            end
        end
        assign select_valid_o[core] = remap_select_valid_raw[core] &&
                                      (remap_remappable[core] || outstanding_clear) &&
                                      !replay_hold;
    end

    // Per destination slot, the lowest logical core whose head is offered to it
    always_comb begin : compose_remap_route_signals
        route_valid_o    = '0;
        route_src_core_o = '0;

        for (int unsigned dst_core = 0; dst_core < NumCores; dst_core++) begin
            for (int unsigned dst_cluster = 0; dst_cluster < NumClusters; dst_cluster++) begin
                for (int unsigned src_core = 0; src_core < NumCores; src_core++) begin
                    // Replay pushes have priority over routed tasks in their cluster.
                    if (!route_valid_o[dst_core][dst_cluster] &&
                        !replay_move_cluster_i[dst_cluster] &&
                        offer_valid_i[src_core][dst_cluster] &&
                        select_valid_o[src_core] &&
                        (physical_core_o[src_core] == CoreIdWidth'(dst_core)) &&
                        (physical_cluster_o[src_core] == ClusterIdWidth'(dst_cluster))) begin

                        route_valid_o[dst_core][dst_cluster] = 1'b1;
                        route_src_core_o[dst_core][dst_cluster] = CoreIdWidth'(src_core);
                    end
                end
            end
        end
    end

    // The offer of a logical core is taken when its destination slot takes it
    for (genvar core = 0; core < NumCores; core++) begin : gen_offer_ready
        always_comb begin
            for (int cluster = 0; cluster < NumClusters; cluster = cluster + 1) begin
                offer_ready_o[core][cluster] = 1'b0;

                if (select_valid_o[core] &&
                    (physical_cluster_o[core] == ClusterIdWidth'(cluster)) &&
                    (route_src_core_o[physical_core_o[core]][physical_cluster_o[core]] ==
                        CoreIdWidth'(core))) begin

                    offer_ready_o[core][cluster] =
                        route_fire_o[physical_core_o[core]][physical_cluster_o[core]];
                end
            end
        end
    end

    // The index expressions below are written out at every use, as each read
    // of an array with a variable index is its own X source for the
    // equivalence check (NumCores need not be a power of two).
    for (genvar core = 0; core < NumCores; core = core + 1) begin: gen_route_core
        for (genvar cluster = 0; cluster < NumClusters; cluster = cluster + 1) begin: gen_route_cluster
            logic ready_inp_valid, ready_inp_ready, ready_drop;
            // Level 3: an exported task only enters the proxy's checkout queue
            // (with its export) and never its ready queue
            assign route_export_o[core][cluster] = RemoteEn && route_valid_o[core][cluster] &&
                                                   remap_remote[route_src_core_o[core][cluster]] &&
                                                   !route_exit_absorb_o[core][cluster];
            assign route_exit_absorb_o[core][cluster] =
                waiting_desc_i[route_src_core_o[core][cluster]].is_exit &&
                waiting_desc_i[route_src_core_o[core][cluster]].task_type != TaskDummy &&
                ((waiting_desc_i[route_src_core_o[core][cluster]].assigned_core_id != CoreIdWidth'(core)) ||
                 (waiting_desc_i[route_src_core_o[core][cluster]].assigned_cluster_id != ClusterIdWidth'(cluster)) ||
                 remap_remote[route_src_core_o[core][cluster]]);
            assign route_fire_o[core][cluster] =
                route_valid_o[core][cluster] &&
                !checkout_full_i[core][cluster] &&
                ready_inp_ready &&
                (!route_export_o[core][cluster] || export_grant_i[core][cluster]);
            // Drop from ready queue if:
            // 1. Dummy set task (task_type==01, dep_set_en==1)
            // 2. DARTS CERF: conditionally skipped task — skip execution but propagate deps
            // 3. an absorbed exit, or an export
            assign ready_inp_valid = route_valid_o[core][cluster] && !checkout_full_i[core][cluster];
            assign ready_drop =
                route_valid_o[core][cluster] &&
                (((waiting_desc_i[route_src_core_o[core][cluster]].task_type == TaskDummy) &&
                  (waiting_desc_i[route_src_core_o[core][cluster]].dep_set_info.dep_set_en == 1'b1)) ||
                 skip_i[route_src_core_o[core][cluster]] ||
                 route_exit_absorb_o[core][cluster] ||
                 route_export_o[core][cluster]);
            stream_filter i_stream_filter_for_ready_queue_dummy_set (
                .valid_i ( ready_inp_valid              ),
                .ready_o ( ready_inp_ready              ),
                .drop_i  ( ready_drop                   ),
                .valid_o ( ready_push_o[core][cluster]  ),
                .ready_i ( !ready_full_i[core][cluster] )
            );
        end
    end
endmodule
