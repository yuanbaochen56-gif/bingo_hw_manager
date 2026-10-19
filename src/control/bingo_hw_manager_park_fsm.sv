// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Parking state of one logical slot (bingo_hw_manager_slot_table): four flag
// registers, of which five combinations occur:
//   hold parked unpark fail
//    0     0      0     0   NONE    not asked
//    1     0      0     0   HOLD    asked: no new task is admitted, the slot drains
//    0     1      0     0   PARKED  drained: its executing tasks follow the table
//    0     1      1     0   UNPARK  request dropped: its new tasks wait until none
//                                   of its tasks is left on the substitute
//    0     0      0     1   FAIL    no substitute: until the request drops
// The table entry of the slot is written by bingo_hw_manager_slot_table, on
// commit_o (the parking choice) and unpark_commit_o (the plain choice).
module bingo_hw_manager_park_fsm (
    input  logic clk_i,
    input  logic rst_ni,

    // Park request of the slot
    input  logic req_i,
    // The slot is fenced (this ends parking), or is the table target of a
    // fenced or parked slot (it runs that slot's tasks and cannot drain)
    input  logic fenced_i,
    input  logic target_i,
    // Its checkout and ready queues are empty and nothing is pushed into it
    input  logic drained_i,
    // The parking choice found another live slot that may take its tasks
    input  logic choice_ok_i,
    // Its table entry is recomputed this cycle, and the reason is that its
    // table substitute was just fenced
    input  logic recompute_i,
    input  logic sub_fenced_i,
    // Tasks of the slot still sit in another slot's checkout queue
    input  logic moved_i,

    output logic hold_o,
    output logic parked_o,
    output logic unpark_o,
    output logic fail_o,
    // HOLD -> PARKED, and UNPARK -> NONE: the table entry is written
    output logic commit_o,
    output logic unpark_commit_o
);

    logic hold_q, parked_q, unpark_q, fail_q;

    // The transitions. The always_ff below applies, after a fence of the slot
    // (which clears it), the first of them that holds, in this order:
    //   commit                        HOLD -> PARKED
    //   give_up, hold_target          HOLD -> FAIL
    //   drop_request                  HOLD -> NONE
    //   enter_hold, fail_at_once      NONE -> HOLD / FAIL
    //   clear_fail                    FAIL -> NONE
    //   lose_substitute               PARKED or UNPARK -> FAIL
    //   unpark_commit                 UNPARK -> NONE
    //   start_unpark                  PARKED -> UNPARK
    //   cancel_unpark                 UNPARK -> PARKED
    logic give_up, hold_target, drop_request, enter_hold, fail_at_once, clear_fail;
    logic lose_substitute, start_unpark, cancel_unpark;

    // Drained, with a substitute (not in the cycle the slot is fenced: that
    // table write belongs to the fence)
    assign commit_o = hold_q && !parked_q && !fenced_i && !target_i && drained_i && choice_ok_i;
    // No task of the slot left on the substitute, and no recompute this cycle
    assign unpark_commit_o = parked_q && unpark_q && !fenced_i && !recompute_i && !moved_i;
    // Drained, but no substitute
    assign give_up = hold_q && !parked_q && !fenced_i && drained_i && !choice_ok_i;
    // A slot in HOLD that became the table target of another slot runs that
    // slot's tasks and cannot drain on its own: it gives up
    assign hold_target = hold_q && target_i;
    // The request dropped before the slot drained
    assign drop_request = hold_q && !req_i;
    // A new request (not while a failed one is still asserted); a slot that runs
    // another slot's tasks cannot drain, so it fails at once
    assign enter_hold   = !hold_q && !parked_q && !fail_q && req_i && !target_i;
    assign fail_at_once = !hold_q && !parked_q && !fail_q && req_i && target_i;
    // The failed request dropped
    assign clear_fail = fail_q && !req_i;
    // The substitute of a parked slot was just fenced and no other is left
    assign lose_substitute = parked_q && sub_fenced_i && !choice_ok_i;
    // The request of a parked slot dropped: move back
    assign start_unpark = parked_q && !unpark_q && !req_i;
    // Asked again before the move back finished: stay parked
    assign cancel_unpark = unpark_q && req_i;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            hold_q   <= 1'b0;
            parked_q <= 1'b0;
            fail_q   <= 1'b0;
            unpark_q <= 1'b0;
        end else if (fenced_i) begin
            // A fence of the slot ends parking (any state -> NONE)
            hold_q   <= 1'b0;
            parked_q <= 1'b0;
            fail_q   <= 1'b0;
            unpark_q <= 1'b0;
        end else if (commit_o) begin
            hold_q   <= 1'b0;
            parked_q <= 1'b1;
            fail_q   <= 1'b0;
        end else if (give_up || hold_target) begin
            hold_q <= 1'b0;
            fail_q <= 1'b1;
        end else if (drop_request) begin
            hold_q <= 1'b0;
        end else if (enter_hold) begin
            hold_q <= 1'b1;
        end else if (fail_at_once) begin
            fail_q <= 1'b1;
        end else if (clear_fail) begin
            fail_q <= 1'b0;
        end else if (lose_substitute) begin
            parked_q <= 1'b0;
            unpark_q <= 1'b0;
            fail_q   <= 1'b1;
        end else if (unpark_commit_o) begin
            parked_q <= 1'b0;
            unpark_q <= 1'b0;
        end else if (start_unpark) begin
            unpark_q <= 1'b1;
        end else if (cancel_unpark) begin
            unpark_q <= 1'b0;
        end
    end

    assign hold_o   = hold_q;
    assign parked_o = parked_q;
    assign unpark_o = unpark_q;
    assign fail_o   = fail_q;

endmodule
