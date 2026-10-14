// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level 3, executor side: the replies to the origin chiplets (the imports are
// bingo_hw_manager_remote_import). The done of a retiring imported task
// (done_i, one per slot) and a reject (reject_i: an import no live core here
// may run, or an imported entry that a replay bounced) leave through one
// stream (remote_done_o). One reject waits at a time (reject_pending_o),
// below every retiring done: the dones of a proxy slot reach its origin
// before its reject (once raised, the request is held until taken). Without
// RemoteEn every output is 0.
module bingo_hw_manager_remote_reply #(
    parameter int unsigned NumSlots = 8,
    parameter bit          RemoteEn = 1'b0,
    parameter type         remote_done_t = logic  // remote_done_t of the top
) (
    input  logic                        clk_i,
    input  logic                        rst_ni,
    // A reject to send, and a reject still waits
    input  logic                        reject_valid_i,
    input  remote_done_t                reject_i,
    output logic                        reject_pending_o,
    // Dones of imported tasks: a retiring imported checkout head, per slot
    input  remote_done_t [NumSlots-1:0] done_i,
    input  logic         [NumSlots-1:0] done_valid_i,
    output logic         [NumSlots-1:0] done_ready_o,
    // To the transport
    output logic                        remote_done_valid_o,
    input  logic                        remote_done_ready_i,
    output remote_done_t                remote_done_o
);
    if (RemoteEn) begin : gen_reply
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
                if (reject_valid_i) begin
                    reject_valid_q <= 1'b1;
                    reject_q       <= reject_i;
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
    end else begin : gen_no_reply
        assign reject_pending_o    = 1'b0;
        assign done_ready_o        = '0;
        assign remote_done_valid_o = 1'b0;
        assign remote_done_o       = '0;
    end
endmodule
