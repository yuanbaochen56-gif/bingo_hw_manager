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
// When an imported task retires, its done leaves through remote_done_*_o
// instead of setting local dependencies. A task that no live core here may
// run, now or later (fenced is sticky), is taken and rejected back to its
// origin instead of blocking the link; so is an imported entry that a replay
// bounces (its core died and no other live core is left). One reject waits at
// a time, below every retiring done: the dones of a proxy slot reach its
// origin before its reject (once raised, the request is held until taken).
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
    parameter type         remote_slot_t = logic,
    parameter type         remote_tag_t = logic,
    parameter type         remote_done_t = logic,
    parameter type         chip_id_t = logic,
    // Dependent parameters, DO NOT OVERRIDE!
    parameter int unsigned NumSlots = NumCores * NumClusters
) (
    input  logic                                     clk_i,
    input  logic                                     rst_ni,
    input  chip_id_t                                 chip_id_i,
    // From the transport; desc_i is the exported descriptor, as a task of this chiplet
    input  logic                                     remote_dispatch_valid_i,
    output logic                                     remote_dispatch_ready_o,
    input  task_desc_t                               remote_dispatch_desc_i,
    input  logic [CoreTypeIdWidth-1:0]               remote_dispatch_core_type_i,
    input  chip_id_t                                 remote_dispatch_origin_chip_i,
    input  remote_slot_t                             remote_dispatch_proxy_slot_i,
    // Where an import may go now
    input  logic [NumCores-1:0][NumClusters-1:0]     fenced_i,
    input  logic                                     replay_pending_i,
    input  logic                                     replay_moving_i,
    input  logic [NumCores-1:0][NumClusters-1:0]     route_valid_i,
    input  logic [NumCores-1:0][NumClusters-1:0]     replay_push_i,
    input  logic [NumCores-1:0][NumClusters-1:0]     checkout_full_i,
    input  logic [NumCores-1:0][NumClusters-1:0]     ready_full_i,
    // The import taken this cycle: pushed into one slot, or rejected
    output logic                                     fire_o,
    output logic                                     reject_o,
    output logic [NumCores-1:0][NumClusters-1:0]     push_o,
    output task_desc_t                               desc_o,
    output remote_tag_t                              tag_o,
    // Its home slot and the slot it runs on
    output logic [CoreIdWidth-1:0]                  home_core_o,
    output logic [ClusterIdWidth-1:0]               home_cluster_o,
    output logic [CoreIdWidth-1:0]                  core_o,
    output logic [ClusterIdWidth-1:0]               cluster_o,
    // Dones of imported tasks: a retiring imported checkout head, per slot
    input  remote_done_t [NumSlots-1:0]              done_i,
    input  logic         [NumSlots-1:0]              done_valid_i,
    output logic         [NumSlots-1:0]              done_ready_o,
    // A bounced imported entry (replay found no live core for it)
    input  logic                                     bounce_i,
    input  remote_tag_t                              bounce_tag_i,
    input  task_desc_t                               bounce_desc_i,
    // A reject waits: the replay controller bounces nothing more meanwhile
    output logic                                     reject_pending_o,
    // To the transport
    output logic                                     remote_done_valid_o,
    input  logic                                     remote_done_ready_i,
    output remote_done_t                             remote_done_o
);
    if (RemoteEn) begin : gen_import
        logic home_found, sub_found;
        // Import: home slot = lowest slot of the requested type
        always_comb begin : find_import_home
            home_found     = 1'b0;
            home_core_o    = '0;
            home_cluster_o = '0;
            for (int cl = NumClusters - 1; cl >= 0; cl--) begin
                for (int c = NumCores - 1; c >= 0; c--) begin
                    if ((remote_dispatch_core_type_i != '0) &&
                        (CoreTypeId[c][cl] == remote_dispatch_core_type_i)) begin
                        home_found     = 1'b1;
                        home_core_o    = CoreIdWidth'(c);
                        home_cluster_o = ClusterIdWidth'(cl);
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
            .logical_core_i(home_core_o),
            .logical_cluster_i(home_cluster_o),
            .fenced_i(fenced_i),
            .found_o(sub_found),
            .core_o(core_o),
            .cluster_o(cluster_o)
        );
        // A task of the home slot, checked on its origin chiplet: no dependencies here
        always_comb begin : compose_import_desc
            desc_o                     = remote_dispatch_desc_i;
            desc_o.assigned_chiplet_id = chip_id_i;
            desc_o.assigned_cluster_id = home_cluster_o;
            desc_o.assigned_core_id    = home_core_o;
            desc_o.dep_check_info      = '0;
            desc_o.dep_set_info        = '0;
            desc_o.cond_exec_en        = 1'b0;
            desc_o.cond_exec_group_id  = '0;
            desc_o.cond_exec_invert    = 1'b0;
        end
        assign tag_o = '{exported:    1'b0,
                         imported:    1'b1,
                         origin_chip: remote_dispatch_origin_chip_i,
                         proxy_slot:  remote_dispatch_proxy_slot_i};
        assign fire_o = remote_dispatch_valid_i && home_found && sub_found &&
                        !replay_pending_i && !replay_moving_i &&
                        !route_valid_i[core_o][cluster_o] &&
                        !replay_push_i[core_o][cluster_o] &&
                        !checkout_full_i[core_o][cluster_o] &&
                        !ready_full_i[core_o][cluster_o];
        always_comb begin
            push_o = '0;
            push_o[core_o][cluster_o] = fire_o;
        end
        // No live core here may run it, now or later (fenced is sticky): take it
        // and reject it back to its origin instead of blocking the link. Waits
        // for a local replay, which may first bounce older imports of the type.
        assign reject_o = remote_dispatch_valid_i && !(home_found && sub_found) &&
                          !replay_pending_i && !replay_moving_i && !reject_pending_o;
        assign remote_dispatch_ready_o = fire_o || reject_o;

        // One pending reject (an import reject or a bounced imported entry)
        logic         reject_valid_q, reject_raised_q;
        remote_done_t reject_q;
        logic         reject_arb_valid, reject_arb_ready;
        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (!rst_ni) begin
                reject_valid_q  <= 1'b0;
                reject_q        <= '0;
                reject_raised_q <= 1'b0;
            end else begin
                if (reject_arb_valid && reject_arb_ready) begin
                    reject_valid_q  <= 1'b0;
                    reject_raised_q <= 1'b0;
                end else if (reject_arb_valid) begin
                    reject_raised_q <= 1'b1;
                end
                if (reject_o) begin
                    reject_valid_q <= 1'b1;
                    reject_q       <= '{chip:       remote_dispatch_origin_chip_i,
                                        proxy_slot: remote_dispatch_proxy_slot_i,
                                        task_id:    desc_o.task_id,
                                        reject:     1'b1};
                end else if (bounce_i) begin
                    reject_valid_q <= 1'b1;
                    reject_q       <= '{chip:       bounce_tag_i.origin_chip,
                                        proxy_slot: bounce_tag_i.proxy_slot,
                                        task_id:    bounce_desc_i.task_id,
                                        reject:     1'b1};
                end
            end
        end
        assign reject_pending_o = reject_valid_q;
        // Below every retiring done: the dones of a proxy slot reach its origin
        // before its reject (once raised, the request is held until taken)
        assign reject_arb_valid = reject_valid_q && (reject_raised_q || !(|done_valid_i));

        // Dones of imported tasks and rejects, back to their origin
        remote_done_t [NumSlots:0] remote_done_all_data;
        logic         [NumSlots:0] remote_done_all_valid, remote_done_all_ready;
        assign remote_done_all_data  = {reject_q, done_i};
        assign remote_done_all_valid = {reject_arb_valid, done_valid_i};
        assign {reject_arb_ready, done_ready_o} = remote_done_all_ready;
        stream_arbiter #(
            .DATA_T(remote_done_t),
            .N_INP (NumSlots + 1)
        ) i_remote_done_arbiter (
            .clk_i      ( clk_i                 ),
            .rst_ni     ( rst_ni                ),
            .inp_data_i ( remote_done_all_data  ),
            .inp_valid_i( remote_done_all_valid ),
            .inp_ready_o( remote_done_all_ready ),
            .oup_data_o ( remote_done_o         ),
            .oup_valid_o( remote_done_valid_o   ),
            .oup_ready_i( remote_done_ready_i   )
        );
    end else begin : gen_no_import
        assign remote_dispatch_ready_o = 1'b0;
        assign fire_o                  = 1'b0;
        assign reject_o                = 1'b0;
        assign push_o                  = '0;
        assign desc_o                  = '0;
        assign tag_o                   = '0;
        assign home_core_o             = '0;
        assign home_cluster_o          = '0;
        assign core_o                  = '0;
        assign cluster_o               = '0;
        assign done_ready_o            = '0;
        assign reject_pending_o        = 1'b0;
        assign remote_done_valid_o     = 1'b0;
        assign remote_done_o           = '0;
    end
endmodule
