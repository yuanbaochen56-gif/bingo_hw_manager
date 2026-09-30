// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Replay controller: moves the outstanding tasks of a fenced (confirmed dead)
// core to live cores, so that they are executed again.
//
// A core's checkout FIFO holds every task dispatched to it and not yet retired,
// in dispatch order: the head is the task the core was running when it died,
// the rest were queued behind it. The dep checks of these tasks already passed,
// so they are pushed straight into the ready + checkout FIFOs of a substitute.
// Their dep_set still uses the logical core id of the descriptor, so the
// dependents see the same producer as before.
//
// One slot is migrated at a time:
//   IDLE   pick the lowest fenced slot D that is neither retired nor stuck and
//          does not wait for another slot (see below)
//   DRAIN  flush D's ready FIFO; wait until D's done FIFO is empty (a done that
//          arrived before the fence retires its task normally, no replay)
//   MOVE   pop D's checkout head and push it to substitute S, chosen per entry
//          from the entry's logical core and cluster (bingo_hw_manager_substitute_sel,
//          so S may sit in another cluster with level 2); stall while S is full. If no live
//          core may run the entry, mark D stuck and go back to IDLE.
//   FINISH mark D retired
// A substitute S may be fenced while D is only partly moved (e.g. MOVE stalls
// because S is full, and S is dead as well). D's entries already on S are older
// than the ones still on D, so moving the rest of D to the next substitute S2
// before S itself is migrated would put D's newest tasks ahead of its oldest
// ones on S2. MOVE therefore aborts D (back to IDLE, not retired) as soon as a
// slot it pushed into is fenced; D waits until every such slot is retired (S's
// entries, including D's older ones, reached S2 first) and then resumes. If
// such a slot is stuck, D is marked stuck as well. The wait-for relation is
// acyclic: D only waits for slots fenced after they received D's entries.
// While D is partly moved, its checkout output stays held (hold_o), so a
// dummy-set left on D cannot fire before D's moved tasks.
// Stuck is sticky and final: fenced is sticky, so a missing substitute never
// appears later. A stuck slot keeps its remaining entries (the top holds its
// checkout output), and the other fenced slots are still migrated.
// While a slot is fenced and not retired, the top holds the new tasks of every
// fenced logical core, so the migrated (older) tasks of a logical core always
// enter a substitute before its newer ones.
//
// Level 3 (SubstituteLevelMask[2], remote chiplet): an entry that no live core
// of the chiplet may run is not stuck if its logical core has a non-zero type
// that the transport can export (remote_type_en_i) and the entry was not itself
// imported from another chiplet. MOVE then
// rotates it: pops D's head and pushes it back to D's tail (rotate_o, marked
// exported), and an executing entry is also copied to the export FIFO
// (export_o). D stays the proxy of these entries: the remote done of an
// exported entry is pushed into D's done FIFO and retires it at D's checkout
// head, with the normal dep_set, once D is retired. MOVE ends when the head is
// an exported entry (every older entry was moved or rotated). A dummy-set
// entry is rotated without an export, so it still fires after D's earlier
// tasks. D's done FIFO may already hold remote dones when an aborted
// migration resumes, so DRAIN does not wait for it once D rotated an entry.
// An entry imported from another chiplet that no live core here may run is
// bounced instead of stuck: MOVE pops it without a push (bounce_o) and the top
// sends a reject back to its origin, which marks the proxy slot stuck there.
module bingo_hw_manager_replay_ctrl #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // Type of each (core, cluster) slot (same as bingo_hw_manager_core_remap)
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    // Substitute levels (see bingo_hw_manager_top SubstituteLevelMask)
    parameter logic [2:0] SubstituteLevelMask = 3'b001
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  logic [NumCores-1:0][NumClusters-1:0] fenced_i,
    input  logic [NumCores-1:0][NumClusters-1:0] done_q_empty_i,
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_empty_i,
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_full_i,
    input  logic [NumCores-1:0][NumClusters-1:0] ready_full_i,
    // Fields of each checkout head
    input  logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0] checkout_logical_core_i,
    input  logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] checkout_logical_cluster_i,
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_no_exec_i, // task_type 01: checkout only
    // Level 3: the head was exported by this slot (rotated) / imported from
    // another chiplet
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_exported_i,
    input  logic [NumCores-1:0][NumClusters-1:0] checkout_imported_i,
    // Level 3: the export FIFO accepts an entry
    input  logic                                 export_ready_i,
    // Level 3: core types with an export target (index: CoreTypeId)
    input  logic [2**CoreTypeIdWidth-1:0]        remote_type_en_i,
    // Level 3: the reject register accepts a bounced entry
    input  logic                                 bounce_ready_i,

    output logic [NumCores-1:0][NumClusters-1:0] retired_o,
    output logic [NumCores-1:0][NumClusters-1:0] ready_flush_o,
    // Slot currently drained by MOVE: its normal checkout output must be held
    output logic [NumCores-1:0][NumClusters-1:0] move_o,
    // Fenced slot whose checkout output must be held: being moved, or partly
    // moved and waiting to resume
    output logic [NumCores-1:0][NumClusters-1:0] hold_o,
    // One MOVE step: pop the head of (src_core_o, src_cluster_o) and push it to
    // (dst_core_o, dst_cluster_o); push_ready_o also pushes its task id to the
    // ready FIFO of the destination.
    output logic                      move_fire_o,
    output logic                      push_ready_o,
    output logic [CoreIdWidth-1:0]    src_core_o,
    output logic [ClusterIdWidth-1:0] src_cluster_o,
    output logic [CoreIdWidth-1:0]    dst_core_o,
    output logic [ClusterIdWidth-1:0] dst_cluster_o,
    // Fenced slot whose next entry no live core may run (sticky); its checkout
    // output must stay held
    output logic [NumCores-1:0][NumClusters-1:0] stuck_o,
    // Level 3: this MOVE step rotates the head of the source (dst = src) and,
    // with export_o, copies it to the export FIFO
    output logic                      rotate_o,
    output logic                      export_o,
    // Level 3: this MOVE step drops the (imported) head of the source without
    // a push; the top rejects it back to its origin chiplet
    output logic                      bounce_o
);

    localparam bit RemoteEn = SubstituteLevelMask[2];

    typedef enum logic [1:0] {
        IDLE,
        DRAIN,
        MOVE,
        FINISH
    } replay_state_t;

    replay_state_t state_q, state_d;
    logic [CoreIdWidth-1:0]    src_core_q, src_core_d;
    logic [ClusterIdWidth-1:0] src_cluster_q, src_cluster_d;
    logic [NumCores-1:0][NumClusters-1:0] retired_q, retired_d;
    logic [NumCores-1:0][NumClusters-1:0] stuck_q, stuck_d;
    // used_q[s]: slots that received entries of slot s during its (possibly
    // aborted) migration; cleared when s is retired
    logic [NumCores-1:0][NumClusters-1:0][NumCores-1:0][NumClusters-1:0] used_q, used_d;
    // Slot rotated (exported) at least one entry
    logic [NumCores-1:0][NumClusters-1:0] rotated_q, rotated_d;
    // Per slot: a slot it pushed into is fenced and not retired (wait), or stuck
    logic [NumCores-1:0][NumClusters-1:0] wait_used;
    logic [NumCores-1:0][NumClusters-1:0] stuck_used;
    logic [NumCores-1:0][NumClusters-1:0] partly_moved;

    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                wait_used[c][cl]    = |(used_q[c][cl] & fenced_i & ~retired_q);
                stuck_used[c][cl]   = |(used_q[c][cl] & stuck_q);
                partly_moved[c][cl] = |used_q[c][cl];
            end
        end
    end

    // Lowest fenced slot that is neither retired nor stuck, nor waits for a
    // slot it partly moved into
    logic                      pending_found;
    logic [CoreIdWidth-1:0]    pending_core;
    logic [ClusterIdWidth-1:0] pending_cluster;

    always_comb begin
        pending_found   = 1'b0;
        pending_core    = '0;
        pending_cluster = '0;
        for (int cl = NumClusters - 1; cl >= 0; cl--) begin
            for (int c = NumCores - 1; c >= 0; c--) begin
                if (fenced_i[c][cl] && !retired_q[c][cl] && !stuck_q[c][cl] &&
                    !wait_used[c][cl]) begin
                    pending_found   = 1'b1;
                    pending_core    = CoreIdWidth'(c);
                    pending_cluster = ClusterIdWidth'(cl);
                end
            end
        end
    end

    // Substitute for the head of the slot being moved, from its logical slot
    // (the entry may already sit in the queue of an earlier substitute)
    logic [CoreIdWidth-1:0]    head_logical_core;
    logic [ClusterIdWidth-1:0] head_logical_cluster;
    logic                      dst_found;
    logic [CoreIdWidth-1:0]    dst_core;
    logic [ClusterIdWidth-1:0] dst_cluster;

    assign head_logical_core    = checkout_logical_core_i[src_core_q][src_cluster_q];
    assign head_logical_cluster = checkout_logical_cluster_i[src_core_q][src_cluster_q];

    bingo_hw_manager_substitute_sel #(
        .NumCores(NumCores),
        .NumClusters(NumClusters),
        .CoreIdWidth(CoreIdWidth),
        .ClusterIdWidth(ClusterIdWidth),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreTypeId(CoreTypeId),
        .LevelMask(SubstituteLevelMask)
    ) i_substitute_sel (
        .logical_core_i(head_logical_core),
        .logical_cluster_i(head_logical_cluster),
        .fenced_i(fenced_i),
        .found_o(dst_found),
        .core_o(dst_core),
        .cluster_o(dst_cluster)
    );

    logic head_no_exec;
    logic dst_space;
    logic head_exported;
    logic can_rotate;

    assign head_no_exec  = checkout_no_exec_i[src_core_q][src_cluster_q];
    assign head_exported = RemoteEn && checkout_exported_i[src_core_q][src_cluster_q];
    assign can_rotate    = RemoteEn && !checkout_imported_i[src_core_q][src_cluster_q] &&
                           (CoreTypeId[head_logical_core][head_logical_cluster] != '0) &&
                           remote_type_en_i[CoreTypeId[head_logical_core][head_logical_cluster]];
    assign dst_space    = !checkout_full_i[dst_core][dst_cluster] &&
                          (head_no_exec || !ready_full_i[dst_core][dst_cluster]);

    always_comb begin
        state_d       = state_q;
        src_core_d    = src_core_q;
        src_cluster_d = src_cluster_q;
        retired_d     = retired_q;
        stuck_d       = stuck_q;
        used_d        = used_q;
        rotated_d     = rotated_q;
        rotate_o      = 1'b0;
        export_o      = 1'b0;
        bounce_o      = 1'b0;

        // A partly moved slot waiting for a stuck slot can never resume
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (fenced_i[c][cl] && !retired_q[c][cl] && stuck_used[c][cl]) begin
                    stuck_d[c][cl] = 1'b1;
                end
            end
        end

        ready_flush_o = '0;
        move_o        = '0;
        move_fire_o   = 1'b0;
        push_ready_o  = 1'b0;

        case (state_q)
            IDLE: begin
                if (pending_found) begin
                    src_core_d    = pending_core;
                    src_cluster_d = pending_cluster;
                    state_d       = DRAIN;
                end
            end
            DRAIN: begin
                ready_flush_o[src_core_q][src_cluster_q] = 1'b1;
                if (done_q_empty_i[src_core_q][src_cluster_q] || rotated_q[src_core_q][src_cluster_q]) begin
                    state_d = MOVE;
                end
            end
            MOVE: begin
                move_o[src_core_q][src_cluster_q] = 1'b1;
                if (checkout_empty_i[src_core_q][src_cluster_q] || head_exported) begin
                    // Everything left (if anything) was rotated: D is its proxy
                    state_d = FINISH;
                end else if (wait_used[src_core_q][src_cluster_q]) begin
                    // A substitute that already holds entries of this slot died:
                    // migrate it first (see above)
                    state_d = IDLE;
                end else if (!dst_found && can_rotate) begin
                    // Level 3: keep it on D as a remote proxy entry. Pop and push
                    // in one cycle leave the usage unchanged, so D's fullness
                    // does not matter.
                    if (head_no_exec || export_ready_i) begin
                        move_fire_o = 1'b1;
                        rotate_o    = 1'b1;
                        export_o    = !head_no_exec;
                        rotated_d[src_core_q][src_cluster_q] = 1'b1;
                    end
                end else if (!dst_found && RemoteEn && checkout_imported_i[src_core_q][src_cluster_q]) begin
                    // Level 3: an imported entry nobody here may run goes back
                    // to its origin as a reject
                    if (bounce_ready_i) begin
                        move_fire_o = 1'b1;
                        bounce_o    = 1'b1;
                    end
                end else if (!dst_found) begin
                    stuck_d[src_core_q][src_cluster_q] = 1'b1;
                    state_d = IDLE;
                end else if (dst_space) begin
                    move_fire_o  = 1'b1;
                    push_ready_o = !head_no_exec;
                    used_d[src_core_q][src_cluster_q][dst_core][dst_cluster] = 1'b1;
                end
            end
            FINISH: begin
                retired_d[src_core_q][src_cluster_q] = 1'b1;
                used_d[src_core_q][src_cluster_q]    = '0;
                state_d = IDLE;
            end
            default: state_d = IDLE;
        endcase
    end

    assign retired_o     = retired_q;
    assign stuck_o       = stuck_q;
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                hold_o[c][cl] = move_o[c][cl] ||
                                (fenced_i[c][cl] && !retired_q[c][cl] && partly_moved[c][cl]);
            end
        end
    end
    assign src_core_o    = src_core_q;
    assign src_cluster_o = src_cluster_q;
    assign dst_core_o    = rotate_o ? src_core_q : dst_core;
    assign dst_cluster_o = rotate_o ? src_cluster_q : dst_cluster;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q       <= IDLE;
            src_core_q    <= '0;
            src_cluster_q <= '0;
            retired_q     <= '0;
            stuck_q       <= '0;
            used_q        <= '0;
            rotated_q     <= '0;
        end else begin
            state_q       <= state_d;
            src_core_q    <= src_core_d;
            src_cluster_q <= src_cluster_d;
            retired_q     <= retired_d;
            stuck_q       <= stuck_d;
            used_q        <= used_d;
            rotated_q     <= rotated_d;
        end
    end

endmodule
