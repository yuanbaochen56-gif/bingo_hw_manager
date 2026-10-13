// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Ready queue of one physical slot: the ids of the tasks its core may start,
// in the order it starts them. Only the task id reaches the core.
//
// Push (at most one source per cycle, in this priority): a replayed entry
// that runs on a core, an imported task (level 3), or a routed task. A routed
// task is only offered here when it runs on a core (bingo_hw_manager_router
// drops dummy sets, CERF-skipped tasks, absorbed exits and exports before the
// queue), and it waits while the queue is full; replayed and imported entries
// are only pushed when the queue has room.
//
// InterfaceType (bingo_hw_manager_top READY_AND_DONE_QUEUE_INTERFACE_TYPE):
//   1: a FIFO the core reads through the CSR interface (pop_i). The watchdog
//      class of each task travels next to it (class_o, never in the ready
//      word), and a replay flushes the queue of a fenced core (flush_i).
//   0: an AXI-Lite read mailbox at base_addr_i; empty_o / data_o / class_o
//      are 0 and pop_i / flush_i are not used.
module bingo_hw_manager_ready_queue #(
    parameter int unsigned InterfaceType = 1,
    parameter int unsigned Depth = 8,
    parameter int unsigned WdClassWidth = 3,
    parameter int unsigned ChipIdWidth = 8,
    parameter int unsigned AxiAddrWidth = 48,
    parameter int unsigned AxiDataWidth = 32,
    parameter type         task_desc_t = logic,  // bingo_hw_manager_task_desc_t of the top
    parameter type         ready_desc_t = logic, // bingo_hw_manager_ready_task_desc_full_t of the top
    parameter type         req_lite_t = logic,
    parameter type         resp_lite_t = logic,
    parameter type         addr_t = logic [AxiAddrWidth-1:0]
) (
    input  logic                    clk_i,
    input  logic                    rst_ni,
    input  logic [ChipIdWidth-1:0]  chip_id_i,
    // Push
    input  logic                    route_push_i,
    input  task_desc_t              route_desc_i,
    input  logic                    replay_push_i,
    input  task_desc_t              replay_desc_i,
    input  logic                    import_push_i,
    input  task_desc_t              import_desc_i,
    output logic                    push_o,
    output ready_desc_t             push_data_o,
    output logic                    full_o,
    // FIFO side (InterfaceType 1)
    input  logic                    flush_i,
    input  logic                    pop_i,
    output logic                    empty_o,
    output ready_desc_t             data_o,
    output logic [WdClassWidth-1:0] class_o,
    // Mailbox side (InterfaceType 0)
    input  addr_t                   base_addr_i,
    input  req_lite_t               axi_lite_req_i,
    output resp_lite_t              axi_lite_resp_o
);
    // Replay pushes never collide with routed pushes: routing into the cluster
    // is blocked while a replay MOVE may push into it.
    task_desc_t source_desc;
    assign source_desc = replay_push_i ? replay_desc_i :
                         import_push_i ? import_desc_i : route_desc_i;
    assign push_data_o.task_id       = source_desc.task_id;
    assign push_data_o.reserved_bits = '0;
    assign push_o = (route_push_i & ~full_o) | replay_push_i | import_push_i;

    if (InterfaceType == 0) begin : gen_ready_queue_axi_lite_mailbox
        bingo_hw_manager_read_mailbox #(
            .MailboxDepth ( Depth        ),
            .IrqEdgeTrig  ( 1'b0         ),
            .IrqActHigh   ( 1'b1         ),
            .AxiAddrWidth ( AxiAddrWidth ),
            .AxiDataWidth ( AxiDataWidth ),
            .ChipIdWidth  ( ChipIdWidth  ),
            .req_lite_t   ( req_lite_t   ),
            .resp_lite_t  ( resp_lite_t  )
        ) i_bingo_hw_manager_ready_queue (
            .clk_i        ( clk_i           ),
            .rst_ni       ( rst_ni          ),
            .chip_id_i    ( chip_id_i       ),
            .test_i       ( 1'b0            ),
            .req_i        ( axi_lite_req_i  ),
            .resp_o       ( axi_lite_resp_o ),
            .irq_o        ( /*not used*/    ),
            .base_addr_i  ( base_addr_i     ),
            .mbox_data_i  ( push_data_o     ),
            .mbox_push_i  ( push_o          ),
            .mbox_full_o  ( full_o          ),
            .mbox_flush_i ( 1'b0            )
        );
        assign empty_o = 1'b0;
        assign data_o  = '0;
        assign class_o = '0;
    end else begin : gen_ready_queue_generic_fifo
        // Extra internal metadata never appears in the device's ready word.
        typedef struct packed {
            logic [WdClassWidth-1:0] wd_class;
            ready_desc_t             desc;
        } ready_entry_t;
        ready_entry_t entry_in, entry_out;
        assign entry_in.desc     = push_data_o;
        assign entry_in.wd_class = source_desc.wd_class;
        assign data_o            = entry_out.desc;
        assign class_o           = entry_out.wd_class;
        fifo_v3 #(
            .FALL_THROUGH ( 1'b0          ),
            .DEPTH        ( Depth         ),
            .dtype        ( ready_entry_t )
        ) i_ready_queue (
            .clk_i       ( clk_i     ),
            .rst_ni      ( rst_ni    ),
            .testmode_i  ( 1'b0      ),
            // Replay: the entries of a fenced core are moved from its checkout queue
            .flush_i     ( flush_i   ),
            .full_o      ( full_o    ),
            .empty_o     ( empty_o   ),
            .usage_o     ( /*not used*/ ),
            .data_i      ( entry_in  ),
            .push_i      ( push_o    ),
            .data_o      ( entry_out ),
            .pop_i       ( pop_i     )
        );
        assign axi_lite_resp_o = '0;
    end
endmodule
