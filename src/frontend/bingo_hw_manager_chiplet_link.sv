// Copyright 2025 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Fanchen Kong <fanchen.kong@kuleuven.be>
// - Xiaoling Yi  <xiaoling.yi@kuleuven.be>
// - Yunhao Deng  <yunhao.deng@kuleuven.be>

// Dependency sets between chiplets:
//   out  a retiring checkout head whose dependency set names another chiplet
//        (set_valid_i, one per slot, bingo_hw_manager_checkout_queue) is sent,
//        one at a time, as one AXI-Lite write of its descriptor to the chiplet
//        mailbox of the target chiplet, or of every chiplet
//        (bingo_hw_manager_chiplet_dep_set)
//   in   the writes of other chiplets queue up in this chiplet's mailbox; its
//        head (remote_set_o) goes to the dependency matrices
//        (bingo_hw_manager_dep_unit). Only the mailbox write register is writable
//        from outside, so a remote write cannot flush the queue (CTRL) or change
//        its thresholds and lose dependency sets (SLVERR, no effect)
module bingo_hw_manager_chiplet_link #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned ChipIdWidth = 8,
    parameter int unsigned HostAxiLiteAddrWidth = 48,
    parameter int unsigned HostAxiLiteDataWidth = 64,
    parameter int unsigned MailboxDepth = 32,
    parameter type         host_axi_lite_req_t = logic,
    parameter type         host_axi_lite_resp_t = logic,
    parameter type         task_desc_t = logic,      // bingo_hw_manager_task_desc_t of the top
    parameter type         task_desc_full_t = logic, // bingo_hw_manager_task_desc_full_t of the top
    // Dependent parameters, DO NOT OVERRIDE!
    parameter type         addr_t = logic [HostAxiLiteAddrWidth-1:0]
) (
    input  logic                                       clk_i,
    input  logic                                       rst_ni,
    input  logic [ChipIdWidth-1:0]                     chip_id_i,
    // Base address of the chiplet mailbox, the same on every chiplet
    input  addr_t                                      mailbox_base_addr_i,
    // Out: the checkout heads, and those whose dependency set goes to another chiplet
    input  task_desc_t [NumCores-1:0][NumClusters-1:0] set_i,
    input  logic [NumCores-1:0][NumClusters-1:0]       set_valid_i,
    output logic [NumCores-1:0][NumClusters-1:0]       set_ready_o,
    output host_axi_lite_req_t                         to_remote_axi_lite_req_o,
    input  host_axi_lite_resp_t                        to_remote_axi_lite_resp_i,
    // In: the mailbox other chiplets write, and its head
    input  host_axi_lite_req_t                         from_remote_axi_lite_req_i,
    output host_axi_lite_resp_t                        from_remote_axi_lite_resp_o,
    output task_desc_full_t                            remote_set_o,
    output logic                                       remote_set_valid_o,
    input  logic                                       remote_set_ready_i
);
    // The whole head descriptor, without the class in the reserved bits: it is
    // the write data, and the other chiplet reads only the dep-set fields and
    // the core id
    function automatic task_desc_full_t write_data(input task_desc_t d);
        task_desc_full_t f;
        f = '0;
        f.is_exit             = d.is_exit;
        f.dep_set_info        = d.dep_set_info;
        f.dep_check_info      = d.dep_check_info;
        f.assigned_core_id    = d.assigned_core_id;
        f.assigned_cluster_id = d.assigned_cluster_id;
        f.assigned_chiplet_id = d.assigned_chiplet_id;
        f.task_id             = d.task_id;
        f.task_type           = d.task_type;
        f.cond_exec_en        = d.cond_exec_en;
        f.cond_exec_group_id  = d.cond_exec_group_id;
        f.cond_exec_invert    = d.cond_exec_invert;
        return f;
    endfunction

    task_desc_full_t                             chiplet_dep_set_task_desc;
    logic                                        chiplet_dep_set_task_desc_valid;
    logic                                        chiplet_dep_set_task_desc_ready;
    task_desc_full_t [NumCores*NumClusters-1:0]  stream_arbiter_chiplet_dep_set_inp_task_desc;
    logic            [NumCores*NumClusters-1:0]  stream_arbiter_chiplet_dep_set_inp_valid;
    logic            [NumCores*NumClusters-1:0]  stream_arbiter_chiplet_dep_set_inp_ready;
    task_desc_full_t                             stream_arbiter_chiplet_dep_set_oup_task_desc;
    logic                                        stream_arbiter_chiplet_dep_set_oup_valid;
    logic                                        stream_arbiter_chiplet_dep_set_oup_ready;
    logic [HostAxiLiteDataWidth-1:0]             chiplet_done_queue_mbox_data;
    logic                                        chiplet_done_queue_mbox_empty;
    logic                                        chiplet_done_queue_mbox_pop;

    /////////////////////////////////////////////////////////
    // H2H Dep Set Interface
    /////////////////////////////////////////////////////////
    bingo_hw_manager_chiplet_dep_set #(
        .ChipIdWidth                                  (ChipIdWidth            ),
        .HostAxiLiteAddrWidth                         (HostAxiLiteAddrWidth   ),
        .HostAxiLiteDataWidth                         (HostAxiLiteDataWidth   ),
        .host_axi_lite_req_t                          (host_axi_lite_req_t    ),
        .host_axi_lite_resp_t                         (host_axi_lite_resp_t   ),
        .bingo_hw_manager_task_desc_full_t            (task_desc_full_t       )
    ) i_bingo_hw_manager_chiplet_dep_set (
        .clk_i                             (clk_i                              ),
        .rst_ni                            (rst_ni                             ),
        .chiplet_mailbox_base_addr_i       (mailbox_base_addr_i                ),
        .to_remote_chiplet_axi_lite_req_o  (to_remote_axi_lite_req_o           ),
        .to_remote_chiplet_axi_lite_resp_i (to_remote_axi_lite_resp_i          ),
        .chiplet_dep_set_task_desc_i       (chiplet_dep_set_task_desc          ),
        .chiplet_dep_set_task_desc_valid_i (chiplet_dep_set_task_desc_valid    ),
        .chiplet_dep_set_task_desc_ready_o (chiplet_dep_set_task_desc_ready    )
    );
    assign chiplet_dep_set_task_desc = stream_arbiter_chiplet_dep_set_oup_task_desc;
    assign chiplet_dep_set_task_desc_valid = stream_arbiter_chiplet_dep_set_oup_valid;

    /////////////////////////////////////////////////////////
    // Stream Arbiter for Chiplet Dep Set
    /////////////////////////////////////////////////////////
    stream_arbiter #(
        .DATA_T (task_desc_full_t     ),
        .N_INP  (NumCores * NumClusters)
    ) i_stream_arbiter_chiplet_dep_set (
        .clk_i      ( clk_i                                        ),
        .rst_ni     ( rst_ni                                       ),
        .inp_data_i ( stream_arbiter_chiplet_dep_set_inp_task_desc ),
        .inp_valid_i( stream_arbiter_chiplet_dep_set_inp_valid     ),
        .inp_ready_o( stream_arbiter_chiplet_dep_set_inp_ready     ),
        .oup_data_o ( stream_arbiter_chiplet_dep_set_oup_task_desc ),
        .oup_valid_o( stream_arbiter_chiplet_dep_set_oup_valid     ),
        .oup_ready_i( stream_arbiter_chiplet_dep_set_oup_ready     )
    );
    assign stream_arbiter_chiplet_dep_set_oup_ready = chiplet_dep_set_task_desc_ready;
    always_comb begin : compose_stream_arbiter_chiplet_dep_set_signals
        for (int unsigned cluster = 0; cluster < NumClusters; cluster = cluster + 1) begin
            for (int unsigned core = 0; core < NumCores; core = core + 1) begin
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NumCores] = write_data(set_i[core][cluster]);
                stream_arbiter_chiplet_dep_set_inp_valid[core + cluster * NumCores] = set_valid_i[core][cluster];
                set_ready_o[core][cluster] = stream_arbiter_chiplet_dep_set_inp_ready[core + cluster * NumCores];
            end
        end
    end

    //////////////////////////////////////////////////////////////////////
    // Chiplet from remote Done Queue
    //////////////////////////////////////////////////////////////////////
    bingo_hw_manager_write_mailbox #(
        .MailboxDepth(MailboxDepth                             ),
        .IrqEdgeTrig (1'b0                                     ),
        .IrqActHigh  (1'b1                                     ),
        .AxiAddrWidth(HostAxiLiteAddrWidth                     ),
        .AxiDataWidth(HostAxiLiteDataWidth                     ),
        .ChipIdWidth (ChipIdWidth                              ),
        .RegWriteEn  (1'b0                                     ),
        .req_lite_t  (host_axi_lite_req_t                      ),
        .resp_lite_t (host_axi_lite_resp_t                     )
    ) i_bingo_hw_manager_chiplet_done_queue (
        .clk_i       (clk_i                             ),
        .rst_ni      (rst_ni                            ),
        .chip_id_i   (chip_id_i                         ),
        .test_i      (1'b0                              ),
        .req_i       (from_remote_axi_lite_req_i        ),
        .resp_o      (from_remote_axi_lite_resp_o       ),
        .irq_o       (/*not used*/                      ),
        .base_addr_i (mailbox_base_addr_i               ),
        .mbox_data_o (chiplet_done_queue_mbox_data      ),
        .mbox_pop_i  (chiplet_done_queue_mbox_pop       ),
        .mbox_empty_o(chiplet_done_queue_mbox_empty     ),
        .mbox_flush_i('0                                )
    );
    assign remote_set_o       = task_desc_full_t'(chiplet_done_queue_mbox_data);
    assign remote_set_valid_o = !chiplet_done_queue_mbox_empty;
    assign chiplet_done_queue_mbox_pop = remote_set_ready_i && !chiplet_done_queue_mbox_empty;
endmodule
