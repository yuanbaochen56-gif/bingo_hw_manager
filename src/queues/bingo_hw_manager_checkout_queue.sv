// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Checkout queue of one physical slot (Core, Cluster) and the rule by which
// its head leaves.
//
// Every task placed on the slot waits here, in dispatch order, until it
// retires: the head the core runs (or ran) is the oldest entry. An entry
// keeps the logical core and cluster of its descriptor, so a slot may hold
// tasks of other logical slots (remapped, replayed or imported).
//
// Push (at most one source per cycle, in this priority):
//   - replay: an entry moved from a fenced slot, already checked and CERF-
//     marked at its first dispatch; a rotation (level 3) pushes it back to
//     its own slot, marked exported
//   - import (level 3): a task of another chiplet, run here for it
//   - route: a new task of the waiting queues; one the CERF skips at
//     dispatch, or an absorbed exit, enters as a dummy (it runs on no core)
//
// The head retires (see retire_kind below):
//   - a head that ran on a core leaves together with its own done, popped
//     from the slot's done queue in the same cycle; for an exported head that
//     is the remote done of the same task
//   - a head that runs on no core leaves at once, without a done
//   - nothing leaves a stuck or rejected slot, except heads that run on no
//     core (stuck_drain); a replay MOVE holds the output while it drains the
//     queue (replay_hold_i) and pops it itself (replay_pop_i)
// A retiring head takes its dep_set path: the local dependency matrix (unless
// its dep_set is disabled), or another chiplet; an imported head instead
// leaves through the remote done stream back to its origin.
module bingo_hw_manager_checkout_queue #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    // This slot
    parameter int unsigned Core = 0,
    parameter int unsigned Cluster = 0,
    // Tasks the slot may hold (bingo_hw_manager_top CheckoutQueueDepth)
    parameter int unsigned Depth = 8,
    parameter int unsigned TaskIdWidth = 12,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // Level 3 remote dispatch (side tags; one spare entry for replay rotations)
    parameter bit          RemoteEn = 1'b0,
    // See bingo_hw_manager_top ForeignStuckDrain
    parameter bit          ForeignStuckDrain = 1'b1,
    // Width of the remap outstanding counters (bingo_hw_manager_remap_count)
    parameter int unsigned OutstandingWidth = 1,
    parameter type         task_desc_t = logic,  // bingo_hw_manager_task_desc_t of the top
    parameter type         remote_tag_t = logic, // remote_tag_t of the top
    parameter type         chip_id_t = logic,
    // Dependent parameters, DO NOT OVERRIDE!
    parameter int unsigned FifoDepth = Depth + (RemoteEn ? 1 : 0),
    parameter int unsigned UsageWidth = (FifoDepth > 1) ? $clog2(FifoDepth) : 1
) (
    input  logic                    clk_i,
    input  logic                    rst_ni,
    input  chip_id_t                chip_id_i,
    // Routed task (a waiting queue head placed on this slot)
    input  logic                    route_push_i,
    input  task_desc_t              route_desc_i,
    input  logic                    route_skip_i,        // the CERF skips it at dispatch
    input  logic                    route_exit_absorb_i, // an exit absorbed here (bingo_hw_manager_router)
    input  logic                    route_export_i,      // level 3: exported, this slot is its proxy
    // Replayed entry (bingo_hw_manager_replay_path)
    input  logic                    replay_push_i,
    input  task_desc_t              replay_desc_i,
    input  remote_tag_t             replay_tag_i,        // side tag of the entry on its source slot
    input  logic                    replay_rotate_i,
    input  logic                    replay_exit_absorb_i,
    input  logic                    replay_pop_i,
    input  logic                    replay_hold_i,
    // Imported task (level 3)
    input  logic                    import_push_i,
    input  task_desc_t              import_desc_i,
    input  remote_tag_t             import_tag_i,
    // The entry pushed this cycle, and an exit that enters as a dummy
    output logic                    push_o,
    output task_desc_t              push_desc_o,
    output remote_tag_t             push_tag_o,
    output logic                    exit_absorb_o,
    output logic                    pop_o,
    // Occupancy; full keeps the spare entry for replay rotations
    output logic                    full_o,
    output logic                    full_raw_o,
    output logic                    empty_o,
    output logic [UsageWidth-1:0]   usage_o,
    // Head
    output task_desc_t              head_o,
    output remote_tag_t             head_tag_o,
    output logic                    head_imported_o,
    output logic                    head_exec_o,          // runs on a core: retires with its done
    // Slot state: stuck (no live core may run its head) or rejected (level 3)
    input  logic                    stuck_i,
    input  logic                    rejected_i,
    input  logic [31:0]             cerf_state_i,
    // Foreign heads of a stuck slot (ForeignStuckDrain): retired logical
    // slots, their remap outstanding counts, and how many of their tasks
    // sit in this slot
    input  logic [NumCores-1:0][NumClusters-1:0] retired_i,
    input  logic [OutstandingWidth-1:0]          outstanding_i [NumCores][NumClusters],
    input  logic [OutstandingWidth-1:0]          moved_in_i [NumCores][NumClusters],
    output logic                    stuck_drain_o,
    output logic                    stuck_head_skip_o,
    // Done queue of this slot
    input  logic                    done_empty_i,
    input  logic [TaskIdWidth-1:0]  done_task_id_i,
    output logic                    done_pop_o,
    // Level 3: the done at the head of the done queue is not the exported head's
    output logic                    remote_mismatch_o,
    // Retirement: the head may leave (retire_valid_o) and leaves (retire_fire_o)
    output logic                    retire_valid_o,
    output logic                    retire_fire_o,
    // ... into the local dependency matrix
    output logic                    local_set_valid_o,
    input  logic                    local_set_ready_i,
    // ... to another chiplet
    output logic                    chiplet_set_valid_o,
    input  logic                    chiplet_set_ready_i,
    // ... or, imported, back to its origin
    output logic                    remote_done_valid_o,
    input  logic                    remote_done_ready_i
);
    import bingo_hw_manager_pkg::*;

    fifo_v3 #(
        .FALL_THROUGH ( 1'b0          ),
        .DEPTH        ( FifoDepth     ),
        .dtype        ( task_desc_t   )
    ) i_checkout_queue (
        .clk_i       ( clk_i       ),
        .rst_ni      ( rst_ni      ),
        .testmode_i  ( 1'b0        ),
        .flush_i     ( 1'b0        ),
        .full_o      ( full_raw_o  ),
        .empty_o     ( empty_o     ),
        .usage_o     ( usage_o     ),
        .data_i      ( push_desc_o ),
        .push_i      ( push_o      ),
        .data_o      ( head_o      ),
        .pop_i       ( pop_o       )
    );
    // Level 3: the spare entry is only for replay rotations
    if (RemoteEn) begin : gen_checkout_remote
        assign full_o = full_raw_o || (usage_o == UsageWidth'(Depth));
        fifo_v3 #(
            .FALL_THROUGH ( 1'b0         ),
            .DEPTH        ( FifoDepth    ),
            .dtype        ( remote_tag_t )
        ) i_checkout_remote_tag (
            .clk_i       ( clk_i                            ),
            .rst_ni      ( rst_ni                           ),
            .testmode_i  ( 1'b0                             ),
            .flush_i     ( 1'b0                             ),
            .full_o      ( /* same as the checkout queue */ ),
            .empty_o     ( /* same as the checkout queue */ ),
            .usage_o     ( /* same as the checkout queue */ ),
            .data_i      ( push_tag_o                       ),
            .push_i      ( push_o                           ),
            .data_o      ( head_tag_o                       ),
            .pop_i       ( pop_o                            )
        );
        always_comb begin
            push_tag_o = '0;
            if (replay_push_i) begin
                push_tag_o = replay_tag_i;
                if (replay_rotate_i) push_tag_o.exported = 1'b1;
            end else if (import_push_i) begin
                push_tag_o = import_tag_i;
            end else begin
                push_tag_o.exported = route_export_i;
            end
        end
    end else begin : gen_checkout_no_remote
        assign full_o     = full_raw_o;
        assign push_tag_o = '0;
        assign head_tag_o = '0;
    end
    assign head_imported_o = RemoteEn && head_tag_o.imported;

    // DARTS CERF: if task is conditionally skipped, mark as dummy (TaskDummy)
    // so checkout logic fires dep_set without done_queue match
    always_comb begin
        if (replay_push_i) begin
            // Replayed entry: already checked (and CERF-marked) at its first dispatch
            push_desc_o = replay_desc_i;
        end else if (import_push_i) begin
            // Imported entry: checked on its origin chiplet
            push_desc_o = import_desc_i;
        end else begin
            push_desc_o = route_desc_i;
            if (route_skip_i || route_exit_absorb_i) begin
                push_desc_o.task_type = TaskDummy;
            end
        end
    end
    assign push_o = route_push_i | replay_push_i | import_push_i;
    assign exit_absorb_o = (route_push_i && route_exit_absorb_i) ||
                           (replay_push_i && replay_exit_absorb_i);
    // Pop on the handshake only: the downstream arbiters may raise ready without
    // valid, which would retire an executing head before its done arrived.
    // While a replay MOVE drains this queue, only the replay controller pops it
    // (valid is held low then). A stuck slot keeps its remaining entries: a
    // dummy-set among them must not fire before its lost source task
    // (only heads that run on no core leave it, see stuck_drain).
    // An imported head retires into the remote done stream instead.
    assign pop_o = retire_fire_o || replay_pop_i;

    // A stuck or rejected slot runs nothing more. Its head may still retire
    // the way a skipped task does (dep_set, no done) when it never runs on
    // a core anyway (dummy-set, or skipped at dispatch) or when the CERF
    // now skips it (degradation, see bingo_hw_manager_cerf_fallback). For the
    // slot's own logical core with no task in another slot: then every
    // older task of that core has left this FIFO. With ForeignStuckDrain,
    // also for a logical core X that died before this slot: X is retired
    // (its own queue holds none of its tasks) and every task of X in
    // another slot is in this one, so this FIFO keeps X's order (a type
    // with a substitute whose two cores both died). Anything else waits.
    logic stuck_head_own, stuck_head_foreign;
    assign stuck_head_own =
        (head_o.assigned_core_id == CoreIdWidth'(Core)) &&
        (head_o.assigned_cluster_id == ClusterIdWidth'(Cluster)) &&
        (outstanding_i[Core][Cluster] == '0);
    if (ForeignStuckDrain) begin : gen_foreign_drain
        logic [CoreIdWidth-1:0]    x_core;
        logic [ClusterIdWidth-1:0] x_cluster;
        assign x_core    = head_o.assigned_core_id;
        assign x_cluster = head_o.assigned_cluster_id;
        assign stuck_head_foreign =
            ((x_core != CoreIdWidth'(Core)) || (x_cluster != ClusterIdWidth'(Cluster))) &&
            (int'(x_core) < NumCores) && (int'(x_cluster) < NumClusters) &&
            retired_i[x_core][x_cluster] &&
            (moved_in_i[x_core][x_cluster] == outstanding_i[x_core][x_cluster]);
    end else begin : gen_no_foreign_drain
        assign stuck_head_foreign = 1'b0;
    end
    assign stuck_drain_o = (stuck_i || rejected_i) && !empty_o && !head_imported_o &&
                           (stuck_head_own || stuck_head_foreign);
    assign stuck_head_skip_o = stuck_drain_o && head_o.cond_exec_en &&
        (head_o.cond_exec_invert ? cerf_state_i[head_o.cond_exec_group_id] :
                                   !cerf_state_i[head_o.cond_exec_group_id]);
    // An executing task (normal / gating) only leaves the checkout queue with its
    // own done, for the local and the chiplet dep_set path alike.
    assign head_exec_o = ((head_o.task_type == TaskNormal) || (head_o.task_type == TaskGating) ||
                          (head_o.task_type == TaskNoReplay)) && !stuck_head_skip_o;
    // Level 3: an exported head only retires on the remote done of the
    // same task (the dones of a proxy slot come back in export order)
    assign remote_mismatch_o = RemoteEn && head_tag_o.exported && !empty_o && head_exec_o &&
                               !done_empty_i && (done_task_id_i != head_o.task_id);

    // How the head may leave this cycle (a replay move is not a retirement).
    // Everything that retires a head reads this one rule.
    typedef enum logic [1:0] {
        RetireNone,     // stays
        RetireSkip,     // runs on no core (dummy set, or a head the CERF skips): leaves without a done
        RetireWithDone  // ran on a core: leaves together with its own done
    } retire_kind_e;
    retire_kind_e retire_kind;
    // The retire rule. A head that ran on a core waits for its done (for an
    // exported head: the remote done of the same task) and never leaves a
    // stuck or rejected slot. A head that runs on no core leaves at once; on
    // a stuck or rejected slot only when stuck_drain allows it.
    always_comb begin
        retire_kind = RetireNone;
        if (!empty_o && !replay_hold_i) begin
            if (head_exec_o) begin
                if (!(stuck_i || rejected_i) && !done_empty_i && !remote_mismatch_o) begin
                    retire_kind = RetireWithDone;
                end
            end else if (!(stuck_i || rejected_i) || stuck_drain_o) begin
                retire_kind = RetireSkip;
            end
        end
    end
    assign retire_valid_o = (retire_kind != RetireNone);

    // An imported head retires into the remote done stream, any other into its dep_set path
    logic       set_valid, set_ready;
    logic [1:0] set_oup_valid, set_oup_ready;
    logic       local_valid, local_ready, local_drop;
    assign set_valid           = retire_valid_o && !head_imported_o;
    assign remote_done_valid_o = retire_valid_o && head_imported_o;
    assign retire_fire_o = (set_valid && set_ready) || (remote_done_valid_o && remote_done_ready_i);
    // The done of a head that ran leaves with it. Popping on the dep-matrix
    // arbiter's ready instead left the done behind when the head took another
    // path, so the next task retired on it prematurely.
    assign done_pop_o = retire_fire_o && (retire_kind == RetireWithDone);

    // [0]: local dep set, [1]: chiplet dep set
    stream_demux #(
        .N_OUP ( 2 )
    ) i_stream_demux_checkout_queue_chiplet_dep_set (
        .inp_valid_i ( set_valid     ),
        .inp_ready_o ( set_ready     ),
        .oup_sel_i   ( head_o.dep_set_info.dep_set_chiplet_id != chip_id_i ),
        .oup_valid_o ( set_oup_valid ),
        .oup_ready_i ( set_oup_ready )
    );
    assign chiplet_set_valid_o = set_oup_valid[1];
    assign set_oup_ready       = {chiplet_set_ready_i, local_ready};

    // Only drop the signal when dep set is disabled and the per-(core,cluster) done queue is non-empty
    // (a head that ran has its done here already, see retire_kind; the done
    // queue only decides for a head that runs on no core)
    assign local_drop = (head_o.dep_set_info.dep_set_en == 1'b0) && !done_empty_i;
    stream_filter i_stream_filter_checkout_queue_dep_set_enable (
        .valid_i ( set_oup_valid[0]  ),
        .ready_o ( local_ready       ),
        .drop_i  ( local_drop        ),
        .valid_o ( local_set_valid_o ),
        .ready_i ( local_set_ready_i )
    );
endmodule
