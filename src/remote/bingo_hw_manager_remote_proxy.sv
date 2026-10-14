// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level 3, origin side: the state of the proxy slots (the exports are
// bingo_hw_manager_remote_export).
//
// A slot D whose tasks are exported is their proxy: the remote done of an
// exported task enters D's done queue (bingo_hw_manager_done_queues) and
// retires the head with the normal dep_set. A proxy waits while its head is
// an exported executing task whose remote done has not arrived (the dones
// come back in export order, so this is the time the remote chiplet takes
// for that one task). After timeout_cycles_i waiting cycles in a row (0:
// never) the done is taken as lost and the slot gives up as on a reject:
// proxy_o.rejected (sticky) stops it. Resending is not safe (the task may
// only be slow and would run twice). A done that does not belong to the
// exported head, or a remote done for a slot that is no proxy, sets
// proxy_o.mismatch (sticky).
module bingo_hw_manager_remote_proxy #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter bit          RemoteEn = 1'b0,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         checkout_t = logic,
    parameter type         done_t = logic,
    parameter type         done_remote_t = logic,
    parameter type         proxy_t = logic
) (
    input  logic                                 clk_i,
    input  logic                                 rst_ni,
    // The checkout heads (side tag, empty, runs on a core), and a done at the
    // head of a done queue that is not the exported head's
    input  checkout_t                            checkout_i,
    input  logic [NumCores-1:0][NumClusters-1:0] head_mismatch_i,
    // The done queues (empty), and the remote dones and rejects
    // (bingo_hw_manager_done_queues)
    input  done_t                                done_i,
    input  done_remote_t                         remote_i,
    // Cycles a proxy may wait for the remote done of its head (0: forever)
    input  logic [31:0]                          timeout_cycles_i,
    output proxy_t                               proxy_o
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
