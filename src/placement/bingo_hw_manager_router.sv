// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Placement of new tasks: into which physical slot the head of each waiting
// queue goes, and in which form.
//
// Each core index has a waiting queue, shared by the clusters (here called a
// logical core; see bingo_hw_manager/typedef.svh). Once its head has passed
// the dependency check, it is sent to the physical cluster chosen for it,
// which serves the logical slot (core index, the head's assigned_cluster_id).
// The target slot is
// the logical slot, unless the control plane has moved the tasks of this logical
// slot to a substitute (retired or parked, see bingo_hw_manager_core_remap), or
// level 3 exports them (remote): then the task stays on its proxy slot and is
// only copied to another chiplet.
// Dummy set tasks and tasks skipped by CERF do not execute on any core, so they
// stay on their own logical core. They must be drained from that core's checkout
// queue, which orders them behind the earlier tasks of that core; as long as one
// of those earlier tasks is still in another slot, they are held back. A fenced
// logical core (or one with a task in another slot) is also held back while a
// replay is pending: its earlier tasks may still be in the slot being replayed
// and must arrive at the new core first.
//
// Each target slot accepts only one offer per cycle (the one from the logical
// core with the lowest index); it accepts none while a replay MOVE may push into
// its cluster. A routed task enters the checkout queue of that slot; if it is to
// execute on the core, it also enters its ready queue (route_o.ready_push). The
// following tasks do not execute on any core: dummy set; tasks skipped by CERF
// (waiting_i.skip); exported tasks (route_o.remote, after the grant); and exits
// absorbed into a dummy (route_o.exit_absorb), i.e. exit tasks that do not run
// on their own logical slot. They are absorbed so that the core on the target
// slot does not execute someone else's exit and still waits for its own exit.
//
// Head-of-line blocking: a waiting queue holds the tasks of one core index of
// every cluster in push order, and only its head is routed. A head that may not
// be placed (its logical slot is in HOLD or UNPARK, or it is held as above)
// also holds every task behind it, including those of the same core index in
// other clusters: e.g. while (c, 0) is in HOLD, a later task of (c, 1) waits
// too. The compiler and the scenes keep such tasks out of the way (no task
// that must run behind a protected slot's task in the same waiting queue).
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
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         waiting_t = logic,
    parameter type         slot_state_t = logic,
    parameter type         smt_t = logic,
    parameter type         park_t = logic,
    parameter type         checkout_t = logic,
    parameter type         ready_t = logic,
    parameter type         select_t = logic,
    parameter type         route_t = logic,
    // The request and answer of bingo_hw_manager_core_remap
    parameter type         remap_req_t = logic,
    parameter type         remap_sel_t = logic
) (
    // Waiting queue heads, one per logical core: the head, empty, CERF skip
    input  waiting_t                             waiting_i,
    // The checked head of each logical core, offered to one physical cluster
    input  logic [NumCores-1:0][NumClusters-1:0] offer_valid_i,
    output logic [NumCores-1:0][NumClusters-1:0] offer_ready_o,
    // Slot state (watchdog, replay controller, level 3), the core types level 3
    // may export, the slot mapping table and the cycle it is written, parking
    input  slot_state_t                          slot_i,
    input  logic [2**CoreTypeIdWidth-1:0]        remote_type_en_i,
    input  smt_t                                 smt_i,
    input  logic                                 smt_update_i,
    input  park_t                                park_i,
    // Tasks of each logical slot in the checkout queues of other slots
    input  logic [OutstandingWidth-1:0]          outstanding_i [NumCores][NumClusters],
    // A replay is pending; a replay MOVE may push into the cluster
    input  logic                                 replay_pending_i,
    input  logic [NumClusters-1:0]               replay_move_cluster_i,
    // Room in the slot's checkout and ready queues (full), and the grant of an
    // export (bingo_hw_manager_remote_export)
    input  checkout_t                            checkout_i,
    input  ready_t                               ready_i,
    input  logic [NumCores-1:0][NumClusters-1:0] export_grant_i,
    // Selection per logical core: valid, physical slot
    output select_t                              select_o,
    // Per destination slot: a task is routed to it, from the waiting queue of
    // that logical core (each consumer reads the waiting queue head itself),
    // and enters its checkout queue (fire) and, if it runs on a core, its
    // ready queue (ready_push, while the ready queue has room)
    output route_t                               route_o
);
    import bingo_hw_manager_pkg::*;

    // The selection per logical core and what is routed to each slot (select_o, route_o)
    logic [NumCores-1:0]                                   select_valid;
    logic [NumCores-1:0][CoreIdWidth-1:0]                  physical_core;
    logic [NumCores-1:0][ClusterIdWidth-1:0]               physical_cluster;
    logic [NumCores-1:0][NumClusters-1:0]                  route_valid, route_exit_absorb, route_export;
    logic [NumCores-1:0][NumClusters-1:0]                  route_fire, ready_push;
    logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0] route_src_core;
    assign select_o = '{valid: select_valid, core: physical_core, cluster: physical_cluster};
    assign route_o  = '{valid:       route_valid,
                        src_core:    route_src_core,
                        exit_absorb: route_exit_absorb,
                        remote:      route_export,
                        fire:        route_fire,
                        ready_push:  ready_push};

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

        assign is_dummy_set = (waiting_i.desc[core].task_type == TaskDummy) &&
                              waiting_i.desc[core].dep_set_info.dep_set_en;
        assign remap_remappable[core] = !is_dummy_set && !waiting_i.skip[core];

        remap_req_t remap_req;
        remap_sel_t remap_sel;
        assign remap_req = '{valid:      !waiting_i.empty[core],
                             core:       CoreIdWidth'(core),
                             cluster:    waiting_i.desc[core].assigned_cluster_id,
                             remappable: remap_remappable[core],
                             no_replay:  waiting_i.desc[core].task_type == TaskNoReplay};
        bingo_hw_manager_core_remap #(
            .NumCores(NumCores),
            .NumClusters(NumClusters),
            .CoreIdWidth(CoreIdWidth),
            .ClusterIdWidth(ClusterIdWidth),
            .CoreTypeIdWidth(CoreTypeIdWidth),
            .CoreTypeId(CoreTypeId),
            .SubstituteLevelMask(SubstituteLevelMask),
            .remap_req_t(remap_req_t),
            .remap_sel_t(remap_sel_t),
            .slot_state_t(slot_state_t),
            .smt_t(smt_t),
            .park_t(park_t)
        ) i_core_remap (
            .req_i(remap_req),
            .slot_i(slot_i),
            .remote_type_en_i(remote_type_en_i),
            .smt_i(smt_i),
            .smt_update_i(smt_update_i),
            .park_i(park_i),
            .sel_o(remap_sel)
        );
        assign remap_select_valid_raw[core] = remap_sel.valid;
        assign physical_core[core]          = remap_sel.core;
        assign physical_cluster[core]       = remap_sel.cluster;
        assign remap_remote[core]           = remap_sel.remote;

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
        assign task_cluster = waiting_i.desc[core].assigned_cluster_id;
        always_comb begin
            outstanding_clear = 1'b1;
            replay_hold       = 1'b0;
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (ClusterIdWidth'(cl) == task_cluster) begin
                    if (outstanding_i[core][cl] != '0) begin
                        outstanding_clear = 1'b0;
                    end
                    if ((slot_i.fenced[core][cl] || (outstanding_i[core][cl] != '0)) && replay_pending_i) begin
                        replay_hold = 1'b1;
                    end
                end
            end
        end
        assign select_valid[core] = remap_select_valid_raw[core] &&
                                      (remap_remappable[core] || outstanding_clear) &&
                                      !replay_hold;
    end

    // Per destination slot, the lowest logical core whose head is offered to it
    always_comb begin : compose_remap_route_signals
        route_valid    = '0;
        route_src_core = '0;

        for (int unsigned dst_core = 0; dst_core < NumCores; dst_core++) begin
            for (int unsigned dst_cluster = 0; dst_cluster < NumClusters; dst_cluster++) begin
                for (int unsigned src_core = 0; src_core < NumCores; src_core++) begin
                    // Replay pushes have priority over routed tasks in their cluster.
                    if (!route_valid[dst_core][dst_cluster] &&
                        !replay_move_cluster_i[dst_cluster] &&
                        offer_valid_i[src_core][dst_cluster] &&
                        select_valid[src_core] &&
                        (physical_core[src_core] == CoreIdWidth'(dst_core)) &&
                        (physical_cluster[src_core] == ClusterIdWidth'(dst_cluster))) begin

                        route_valid[dst_core][dst_cluster] = 1'b1;
                        route_src_core[dst_core][dst_cluster] = CoreIdWidth'(src_core);
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

                if (select_valid[core] &&
                    (physical_cluster[core] == ClusterIdWidth'(cluster)) &&
                    (route_src_core[physical_core[core]][physical_cluster[core]] ==
                        CoreIdWidth'(core))) begin

                    offer_ready_o[core][cluster] =
                        route_fire[physical_core[core]][physical_cluster[core]];
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
            assign route_export[core][cluster] = RemoteEn && route_valid[core][cluster] &&
                                                   remap_remote[route_src_core[core][cluster]] &&
                                                   !route_exit_absorb[core][cluster];
            assign route_exit_absorb[core][cluster] =
                waiting_i.desc[route_src_core[core][cluster]].is_exit &&
                waiting_i.desc[route_src_core[core][cluster]].task_type != TaskDummy &&
                ((waiting_i.desc[route_src_core[core][cluster]].assigned_core_id != CoreIdWidth'(core)) ||
                 (waiting_i.desc[route_src_core[core][cluster]].assigned_cluster_id != ClusterIdWidth'(cluster)) ||
                 remap_remote[route_src_core[core][cluster]]);
            assign route_fire[core][cluster] =
                route_valid[core][cluster] &&
                !checkout_i.full[core][cluster] &&
                ready_inp_ready &&
                (!route_export[core][cluster] || export_grant_i[core][cluster]);
            // Drop from ready queue if:
            // 1. Dummy set task (task_type==01, dep_set_en==1)
            // 2. DARTS CERF: conditionally skipped task — skip execution but propagate deps
            // 3. an absorbed exit, or an export
            assign ready_inp_valid = route_valid[core][cluster] && !checkout_i.full[core][cluster];
            assign ready_drop =
                route_valid[core][cluster] &&
                (((waiting_i.desc[route_src_core[core][cluster]].task_type == TaskDummy) &&
                  (waiting_i.desc[route_src_core[core][cluster]].dep_set_info.dep_set_en == 1'b1)) ||
                 waiting_i.skip[route_src_core[core][cluster]] ||
                 route_exit_absorb[core][cluster] ||
                 route_export[core][cluster]);
            stream_filter i_stream_filter_for_ready_queue_dummy_set (
                .valid_i ( ready_inp_valid              ),
                .ready_o ( ready_inp_ready              ),
                .drop_i  ( ready_drop                   ),
                .valid_o ( ready_push[core][cluster]  ),
                .ready_i ( !ready_i.full[core][cluster] )
            );
        end
    end
endmodule
