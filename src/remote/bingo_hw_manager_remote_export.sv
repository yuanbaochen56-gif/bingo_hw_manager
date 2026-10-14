// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level 3, origin side (bingo_hw_manager_top SubstituteLevelMask[2]): export
// queues towards other chiplets and the state of the proxy slots.
//
// A task whose logical slot D is fenced and has no live substitute on this
// chiplet (any enabled level) stays in D's checkout queue, marked exported,
// and a copy leaves through dispatch_o:
//   - outstanding tasks: rotated by the replay controller (step_i.exported)
//   - new tasks of a retired D: routed to D's checkout queue only
//     (route_i.remote); routed exports of different slots are granted one
//     per cycle (route_grant_o), after a replay export, among the slots
//     whose checkout queue and export queue have room
// Replay rotations and routed exports never coincide (the new tasks of
// fenced logical cores are held while a replay is pending).
// There is one export queue per peer of the transport; a proxy only exports
// tasks of its own type, so all of them go through one queue, in order. Only
// a peer that takes an export at once (peer_ready_i) competes.
//
// D is the proxy: the remote done of an exported task enters D's done queue
// (bingo_hw_manager_done_queues) and retires the head with the normal
// dep_set. A proxy waits while its head is an exported executing task whose
// remote done has not arrived (the dones come back in export order, so this
// is the time the remote chiplet takes for that one task). After
// timeout_cycles_i waiting cycles in a row (0: never) the done is taken as
// lost and the slot gives up as on a reject: proxy_o.rejected (sticky) stops it.
// Resending is not safe (the task may only be slow and would run twice).
// A done that does not belong to the exported head, or a remote done for a
// slot that is no proxy, sets proxy_o.mismatch (sticky).
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
    parameter type         remote_export_t = logic,
    parameter type         chip_id_t = logic,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         replay_step_t = logic,
    parameter type         route_t = logic,
    parameter type         checkout_t = logic,
    parameter type         done_t = logic,
    parameter type         done_remote_t = logic,
    parameter type         remote_dispatch_t = logic, // with a task_desc_full_t
    parameter type         proxy_t = logic
) (
    input  logic                                           clk_i,
    input  logic                                           rst_ni,
    input  chip_id_t                                       chip_id_i,
    // Replay rotation of an executing entry of the slot being moved (exported, source slot)
    input  replay_step_t                                   step_i,
    input  task_desc_full_t                                replay_desc_i,
    // Routed tasks that a proxy slot is to export (remote, fire), and the
    // checkout queues (full; the proxy slots' heads)
    input  route_t                                         route_i,
    input  task_desc_full_t [NumCores-1:0][NumClusters-1:0] route_desc_i,
    input  checkout_t                                      checkout_i,
    output logic            [NumCores-1:0][NumClusters-1:0] route_grant_o,
    // Per core type: its export queue has room
    output logic [2**CoreTypeIdWidth-1:0]                  type_ready_o,
    // The export pushed this cycle
    output logic                                           export_push_o,
    output remote_export_t                                 export_o,
    // Peer (export queue) of each core type, and the peers that take an export now
    input  logic [2**CoreTypeIdWidth-1:0][PeerIdWidth-1:0] type_peer_i,
    input  logic [NumPeers-1:0]                            peer_ready_i,
    // To the transport
    output logic                                           dispatch_valid_o,
    input  logic                                           dispatch_ready_i,
    output remote_dispatch_t                               dispatch_o,
    // Proxy slots: the done at the checkout head is not the exported head's,
    // the done queues (empty), and the remote dones and rejects
    // (bingo_hw_manager_done_queues)
    input  logic            [NumCores-1:0][NumClusters-1:0] head_mismatch_i,
    input  done_t                                          done_i,
    input  done_remote_t                                   remote_i,
    input  logic [31:0]                                    timeout_cycles_i,
    output proxy_t                                         proxy_o
);
    // The proxy slots (proxy_o)
    logic [NumCores-1:0][NumClusters-1:0]       waiting, timeout, rejected;
    logic [NumCores-1:0][NumClusters-1:0][31:0] wait_cycles;
    logic                                       timeout_any, mismatch;
    assign proxy_o = '{waiting:     waiting,
                       timeout:     timeout,
                       wait_cycles: wait_cycles,
                       rejected:    rejected,
                       timeout_any: timeout_any,
                       mismatch:    mismatch};


    // Routed exports: one grant per cycle, a replay export goes first
    always_comb begin : compose_route_remote_grant
        automatic logic taken;
        taken = step_i.exported;
        route_grant_o = '0;
        for (int unsigned cl = 0; cl < NumClusters; cl++) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                if (route_i.remote[c][cl] && !checkout_i.full[c][cl] &&
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
        if (step_i.exported) begin
            export_push_o        = 1'b1;
            export_o.desc       = replay_desc_i;
            export_o.core_type  = CoreTypeId[replay_desc_i.assigned_core_id][replay_desc_i.assigned_cluster_id];
            export_o.proxy_slot = remote_slot_t'(step_i.src_core + step_i.src_cluster * NumCores);
        end
        for (int unsigned cl = 0; cl < NumClusters; cl++) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                if (route_grant_o[c][cl] && route_i.fire[c][cl]) begin
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
            .req_o   ( dispatch_valid_o        ),
            .gnt_i   ( dispatch_ready_i        ),
            .data_o  ( export_out              ),
            .idx_o   ( /*not used*/            )
        );
        assign dispatch_o = '{desc:        export_out.desc,
                              core_type:   export_out.core_type,
                              origin_chip: chip_id_i,
                              proxy_slot:  export_out.proxy_slot};
    end else begin : gen_no_export_queues
        assign type_ready_o     = '0;
        assign dispatch_valid_o = 1'b0;
        assign dispatch_o       = '0;
    end

    // Proxy wait timeout
    logic [NumCores-1:0][NumClusters-1:0][31:0] proxy_wait_q;
    logic                                       remote_timeout_q;
    logic [NumCores-1:0][NumClusters-1:0]       remote_rejected_q;
    logic                                       remote_done_mismatch_q;
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                waiting[c][cl] = RemoteEn && checkout_i.head_tag[c][cl].exported &&
                                 !checkout_i.empty[c][cl] && checkout_i.head_exec[c][cl] &&
                                 done_i.empty[c][cl] && !remote_rejected_q[c][cl];
                timeout[c][cl] = waiting[c][cl] && (timeout_cycles_i != '0) &&
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
                    if (!waiting[c][cl])                proxy_wait_q[c][cl] <= '0;
                    else if (proxy_wait_q[c][cl] != '1) proxy_wait_q[c][cl] <= proxy_wait_q[c][cl] + 32'd1;
                end
            end
            if (|timeout) remote_timeout_q <= 1'b1;
        end
    end
    assign wait_cycles = proxy_wait_q;
    assign timeout_any = remote_timeout_q;

    // A proxy slot whose export was rejected (or timed out) stops retiring and
    // exporting (sticky, part of the top's replay_stuck_o)
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) remote_rejected_q <= '0;
        else         remote_rejected_q <= remote_rejected_q | remote_i.reject | timeout;
    end
    assign rejected = remote_rejected_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) remote_done_mismatch_q <= 1'b0;
        else         remote_done_mismatch_q <= remote_done_mismatch_q | (|head_mismatch_i) | remote_i.stray;
    end
    assign mismatch = remote_done_mismatch_q;
endmodule
