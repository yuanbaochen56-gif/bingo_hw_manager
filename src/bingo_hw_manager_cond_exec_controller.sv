// Copyright 2025 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Fanchen Kong <fanchen.kong@kuleuven.be>

// DARTS Tier 1: Conditional Execution Register File (CERF)
//
// A register file that stores activation status for up to NumGroups
// "conditional execution groups." Each group corresponds to a logical unit
// (e.g., one expert in MoE, one exit branch in early exit).
//
// The scheduler queries the CERF combinationally to decide whether a
// conditionally-annotated task should execute or be skipped. Skipped
// tasks still propagate their dependency signals (via the checkout queue)
// but are never dispatched to a core.
//
// Write interface: single 32-bit bitmask write. SW writes the full
// CERF_STATE CSR and pulses CERF_WRITE_EN to latch the value.
// Clearing is simply writing 0.
//
// A second port applies one degradation update from the control plane:
// clear one group and set another, leaving every other group as it is.
// A host bitmask write in the same cycle wins, and policy_done_o stays
// low so the control plane can retry. The two ports never both commit.

module bingo_hw_manager_cond_exec_controller #(
    parameter int unsigned NumGroups = 32
) (
    input  logic                          clk_i,
    input  logic                          rst_ni,
    // Full state output (combinational)
    output logic [NumGroups-1:0]          cerf_state_o,
    // Write port: 32-bit bitmask + enable
    input  logic [NumGroups-1:0]          cerf_write_data_i,
    input  logic                          cerf_write_en_i,
    // Degradation: clear policy_clear_i, set policy_set_i. Loses to the
    // host bitmask write above. policy_done_o is the cycle the update commits.
    input  logic                          policy_req_i = 1'b0,
    input  logic [$clog2(NumGroups)-1:0]  policy_clear_i = '0,
    input  logic [$clog2(NumGroups)-1:0]  policy_set_i = '0,
    output logic                          policy_done_o
);
    logic [NumGroups-1:0] cerf_q;

    // Combinational full-state output
    assign cerf_state_o = cerf_q;

    // Host bitmask replaces the file. Otherwise one group is cleared and one
    // is set (the same index ends set). No request leaves the file unchanged.
    assign policy_done_o = policy_req_i & ~cerf_write_en_i;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            cerf_q <= '0;
        end else if (cerf_write_en_i) begin
            cerf_q <= cerf_write_data_i;
        end else if (policy_req_i) begin
            cerf_q <= (cerf_q & ~(NumGroups'(1) << policy_clear_i)) |
                      (NumGroups'(1) << policy_set_i);
        end
    end
endmodule
