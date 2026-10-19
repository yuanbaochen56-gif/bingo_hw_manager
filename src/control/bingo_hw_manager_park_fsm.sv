// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Parking state of one logical slot (bingo_hw_manager_slot_table). A fence of
// the slot ends parking (any state -> NONE); otherwise:
//   NONE    not asked. A request enters HOLD, or FAIL at once if the slot
//           runs another slot's tasks (it cannot drain then).
//   HOLD    no new task is admitted, the slot drains. Drained with a
//           substitute: PARKED (commit_o). Drained without one, or made the
//           target of another slot: FAIL. Request dropped: NONE.
//   PARKED  its executing tasks follow the table. Request dropped: UNPARK.
//   UNPARK  its new tasks wait until none of its tasks is left on the
//           substitute, then NONE (unpark_commit_o). Asked again: PARKED.
//   FAIL    no substitute; until the request drops.
// In PARKED and UNPARK, a fence of the substitute with no other left: FAIL.
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

    // The state (parked: PARKED or UNPARK)
    output logic hold_o,
    output logic parked_o,
    output logic unpark_o,
    output logic fail_o,
    // HOLD -> PARKED, and UNPARK -> NONE: the table entry is written
    output logic commit_o,
    output logic unpark_commit_o
);

    typedef enum logic [2:0] {ParkNone, ParkHold, ParkParked, ParkUnpark, ParkFail} park_state_e;
    park_state_e state_q, state_d;

    // Not in the cycle the slot is fenced: that table write belongs to the fence
    assign commit_o        = (state_q == ParkHold) && !fenced_i && !target_i && drained_i && choice_ok_i;
    assign unpark_commit_o = (state_q == ParkUnpark) && !fenced_i && !recompute_i && !moved_i;

    always_comb begin
        state_d = state_q;
        if (fenced_i) begin
            state_d = ParkNone;
        end else begin
            unique case (state_q)
                ParkNone: begin
                    if (req_i) state_d = target_i ? ParkFail : ParkHold;
                end
                ParkHold: begin
                    if (commit_o)                                     state_d = ParkParked;
                    else if ((drained_i && !choice_ok_i) || target_i) state_d = ParkFail;
                    else if (!req_i)                                  state_d = ParkNone;
                end
                ParkParked: begin
                    if (sub_fenced_i && !choice_ok_i) state_d = ParkFail;
                    else if (!req_i)                  state_d = ParkUnpark;
                end
                ParkUnpark: begin
                    if (sub_fenced_i && !choice_ok_i) state_d = ParkFail;
                    else if (unpark_commit_o)         state_d = ParkNone;
                    else if (req_i)                   state_d = ParkParked;
                end
                ParkFail: begin
                    if (!req_i) state_d = ParkNone;
                end
                default: state_d = ParkNone;
            endcase
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) state_q <= ParkNone;
        else         state_q <= state_d;
    end

    assign hold_o   = (state_q == ParkHold);
    assign parked_o = (state_q == ParkParked) || (state_q == ParkUnpark);
    assign unpark_o = (state_q == ParkUnpark);
    assign fail_o   = (state_q == ParkFail);

endmodule
