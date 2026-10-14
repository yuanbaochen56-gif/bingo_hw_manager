// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level 3, origin side (bingo_hw_manager_top SubstituteLevelMask[2]): export
// queues towards other chiplets and the state of the proxy slots.
//
// A task whose logical slot D is fenced and has no live substitute on this
// chiplet (any enabled level) stays in D's checkout queue, marked exported,
// and a copy leaves through remote_dispatch_*_o:
//   - outstanding tasks: rotated by the replay controller (replay_export_i)
//   - new tasks of a retired D: routed to D's checkout queue only
//     (route_export_i); routed exports of different slots are granted one
//     per cycle (route_grant_o), after a replay export, among the slots
//     whose checkout queue and export queue have room
// Replay rotations and routed exports never coincide (the new tasks of
// fenced logical cores are held while a replay is pending).
// There is one export queue per peer of the transport; a proxy only exports
// tasks of its own type, so all of them go through one queue, in order. Only
// a peer that takes an export at once (export_peer_ready_i) competes.
//
// D is the proxy: the remote done of an exported task enters D's done queue
// (bingo_hw_manager_done_queues) and retires the head with the normal
// dep_set. A proxy waits while its head is an exported executing task whose
// remote done has not arrived (the dones come back in export order, so this
// is the time the remote chiplet takes for that one task). After
// timeout_cycles_i waiting cycles in a row (0: never) the done is taken as
// lost and the slot gives up as on a reject: rejected_o (sticky) stops it.
// Resending is not safe (the task may only be slow and would run twice).
// A done that does not belong to the exported head, or a remote done for a
// slot that is no proxy, sets mismatch_o (sticky).
module bingo_hw_manager_remote_export #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter bit          RemoteEn = 1'b0,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned NumPeers = 1,
    parameter int unsigned PeerIdWidth = 1,
    parameter int unsigned FifoDepth = 9,             // RemoteExportFifoDepth of the top
    parameter type         task_desc_full_t = logic,  // bingo_hw_manager_task_desc_full_t of the top
    parameter type         remote_slot_t = logic,
    parameter type         remote_tag_t = logic,
    parameter type         remote_export_t = logic,
    parameter type         chip_id_t = logic,
    parameter type         data_t = logic             // host AXI-Lite data word
) (
    input  logic                                          clk_i,
    input  logic                                          rst_ni,
    input  chip_id_t                                      chip_id_i,
    // Replay rotation of an executing entry of the slot being moved
    input  logic                                          replay_export_i,
    input  task_desc_full_t                               replay_desc_i,
    input  logic [CoreIdWidth-1:0]                        replay_src_core_i,
    input  logic [ClusterIdWidth-1:0]                     replay_src_cluster_i,
    // Routed tasks that a proxy slot is to export
    input  logic            [NumCores-1:0][NumClusters-1:0] route_export_i,
    input  task_desc_full_t [NumCores-1:0][NumClusters-1:0] route_desc_i,
    input  logic            [NumCores-1:0][NumClusters-1:0] route_fire_i,
    input  logic            [NumCores-1:0][NumClusters-1:0] checkout_full_i,
    output logic            [NumCores-1:0][NumClusters-1:0] route_grant_o,
    // Per core type: its export queue has room
    output logic [2**CoreTypeIdWidth-1:0]                 type_ready_o,
    // The export pushed this cycle
    output logic                                          export_push_o,
    output remote_export_t                                export_o,
    // Peer (export queue) of each core type, and the peers that take an export now
    input  logic [2**CoreTypeIdWidth-1:0][PeerIdWidth-1:0] type_peer_i,
    input  logic [NumPeers-1:0]                           peer_ready_i,
    // To the transport
    output logic                                          remote_dispatch_valid_o,
    input  logic                                          remote_dispatch_ready_i,
    output data_t                                         remote_dispatch_desc_o,
    output logic [CoreTypeIdWidth-1:0]                    remote_dispatch_core_type_o,
    output chip_id_t                                      remote_dispatch_origin_chip_o,
    output remote_slot_t                                  remote_dispatch_proxy_slot_o,
    // Proxy slots: checkout head and done queue of each slot
    input  remote_tag_t     [NumCores-1:0][NumClusters-1:0] head_tag_i,
    input  logic            [NumCores-1:0][NumClusters-1:0] head_empty_i,
    input  logic            [NumCores-1:0][NumClusters-1:0] head_exec_i,
    input  logic            [NumCores-1:0][NumClusters-1:0] head_mismatch_i,
    input  logic            [NumCores-1:0][NumClusters-1:0] done_empty_i,
    // A reject for the slot came back, a remote done for no proxy was dropped
    // (bingo_hw_manager_done_queues)
    input  logic            [NumCores-1:0][NumClusters-1:0] reject_i,
    input  logic                                          done_stray_i,
    input  logic [31:0]                                   timeout_cycles_i,
    output logic            [NumCores-1:0][NumClusters-1:0] waiting_o,
    output logic            [NumCores-1:0][NumClusters-1:0] timeout_o,
    output logic [NumCores-1:0][NumClusters-1:0][31:0]    wait_cycles_o,
    output logic                                          timeout_any_o,
    output logic            [NumCores-1:0][NumClusters-1:0] rejected_o,
    output logic                                          mismatch_o
);
    // Routed exports: one grant per cycle, a replay export goes first
    always_comb begin : compose_route_remote_grant
        automatic logic taken;
        taken = replay_export_i;
        route_grant_o = '0;
        for (int unsigned cl = 0; cl < NumClusters; cl++) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                if (route_export_i[c][cl] && !checkout_full_i[c][cl] &&
                    type_ready_o[CoreTypeId[c][cl]] && !taken) begin
                    route_grant_o[c][cl] = 1'b1;
                    taken = 1'b1;
                end
            end
        end
    end

    always_comb begin : compose_export_input
        export_o      = '0;
        export_push_o = 1'b0;
        if (replay_export_i) begin
            export_push_o        = 1'b1;
            export_o.desc       = replay_desc_i;
            export_o.core_type  = CoreTypeId[replay_desc_i.assigned_core_id][replay_desc_i.assigned_cluster_id];
            export_o.proxy_slot = remote_slot_t'(replay_src_core_i + replay_src_cluster_i * NumCores);
        end
        for (int unsigned cl = 0; cl < NumClusters; cl++) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                if (route_grant_o[c][cl] && route_fire_i[c][cl]) begin
                    export_push_o        = 1'b1;
                    export_o.desc       = route_desc_i[c][cl];
                    export_o.core_type  = CoreTypeId[c][cl];
                    export_o.proxy_slot = remote_slot_t'(c + cl * NumCores);
                end
            end
        end
    end

    if (RemoteEn) begin : gen_export_queues
        // One export queue per peer
        logic [NumPeers-1:0] export_peer_full, export_peer_empty;
        logic [NumPeers-1:0] export_peer_req, export_peer_gnt;
        remote_export_t [NumPeers-1:0] export_peer_out;
        remote_export_t export_out;
        logic [PeerIdWidth-1:0] export_in_peer;
        assign export_in_peer = type_peer_i[export_o.core_type];
        for (genvar t = 0; t < 2**CoreTypeIdWidth; t++) begin : gen_export_type_ready
            assign type_ready_o[t] = (type_peer_i[t] < NumPeers) && !export_peer_full[type_peer_i[t]];
        end
        for (genvar p = 0; p < NumPeers; p++) begin : gen_export_fifo
            fifo_v3 #(
                .FALL_THROUGH ( 1'b0            ),
                .DEPTH        ( FifoDepth       ),
                .dtype        ( remote_export_t )
            ) i_export_fifo (
                .clk_i       ( clk_i                                            ),
                .rst_ni      ( rst_ni                                           ),
                .testmode_i  ( 1'b0                                             ),
                .flush_i     ( 1'b0                                             ),
                .full_o      ( export_peer_full[p]                              ),
                .empty_o     ( export_peer_empty[p]                             ),
                .usage_o     ( /*not used*/                                     ),
                .data_i      ( export_o                                         ),
                .push_i      ( export_push_o && (export_in_peer == PeerIdWidth'(p)) ),
                .data_o      ( export_peer_out[p]                               ),
                .pop_i       ( export_peer_gnt[p]                               )
            );
            // Only a peer that takes the export at once competes: the transport
            // takes a credit only when it sends, so the chosen peer stays ready
            assign export_peer_req[p] = !export_peer_empty[p] && peer_ready_i[p];
        end
        rr_arb_tree #(
            .NumIn     ( NumPeers        ),
            .DataType  ( remote_export_t ),
            .ExtPrio   ( 1'b0            ),
            .AxiVldRdy ( 1'b1            ),
            .LockIn    ( 1'b1            )
        ) i_export_arb (
            .clk_i   ( clk_i                   ),
            .rst_ni  ( rst_ni                  ),
            .flush_i ( 1'b0                    ),
            .rr_i    ( '0                      ),
            .req_i   ( export_peer_req         ),
            .gnt_o   ( export_peer_gnt         ),
            .data_i  ( export_peer_out         ),
            .req_o   ( remote_dispatch_valid_o ),
            .gnt_i   ( remote_dispatch_ready_i ),
            .data_o  ( export_out              ),
            .idx_o   ( /*not used*/            )
        );
        assign remote_dispatch_desc_o        = data_t'(export_out.desc);
        assign remote_dispatch_core_type_o   = export_out.core_type;
        assign remote_dispatch_origin_chip_o = chip_id_i;
        assign remote_dispatch_proxy_slot_o  = export_out.proxy_slot;
    end else begin : gen_no_export_queues
        assign type_ready_o                  = '0;
        assign remote_dispatch_valid_o       = 1'b0;
        assign remote_dispatch_desc_o        = '0;
        assign remote_dispatch_core_type_o   = '0;
        assign remote_dispatch_origin_chip_o = '0;
        assign remote_dispatch_proxy_slot_o  = '0;
    end

    // Proxy wait timeout
    logic [NumCores-1:0][NumClusters-1:0][31:0] proxy_wait_q;
    logic                                       remote_timeout_q;
    logic [NumCores-1:0][NumClusters-1:0]       remote_rejected_q;
    logic                                       remote_done_mismatch_q;
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                waiting_o[c][cl] = RemoteEn && head_tag_i[c][cl].exported &&
                                   !head_empty_i[c][cl] && head_exec_i[c][cl] &&
                                   done_empty_i[c][cl] && !remote_rejected_q[c][cl];
                timeout_o[c][cl] = waiting_o[c][cl] && (timeout_cycles_i != '0) &&
                                   (proxy_wait_q[c][cl] >= timeout_cycles_i);
            end
        end
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            proxy_wait_q     <= '0;
            remote_timeout_q <= 1'b0;
        end else begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (!waiting_o[c][cl])              proxy_wait_q[c][cl] <= '0;
                    else if (proxy_wait_q[c][cl] != '1) proxy_wait_q[c][cl] <= proxy_wait_q[c][cl] + 32'd1;
                end
            end
            if (|timeout_o) remote_timeout_q <= 1'b1;
        end
    end
    assign wait_cycles_o = proxy_wait_q;
    assign timeout_any_o = remote_timeout_q;

    // A proxy slot whose export was rejected (or timed out) stops retiring and
    // exporting (sticky, part of the top's replay_stuck_o)
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) remote_rejected_q <= '0;
        else         remote_rejected_q <= remote_rejected_q | reject_i | timeout_o;
    end
    assign rejected_o = remote_rejected_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) remote_done_mismatch_q <= 1'b0;
        else         remote_done_mismatch_q <= remote_done_mismatch_q | (|head_mismatch_i) | done_stray_i;
    end
    assign mismatch_o = remote_done_mismatch_q;
endmodule
