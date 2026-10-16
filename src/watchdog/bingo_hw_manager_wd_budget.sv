// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Per-slot total-time budget of the watchdog.
//
// The watchdog's timer restarts at every heartbeat, so a core that keeps beating
// while it waits on a stuck accelerator (the DMA and GEMM wait loops beat on every
// poll) is never suspected. This module times the whole execution instead, against
// the compile-time estimate of the task (bingo_hw_manager_wd_est).
//
// 1. Elapsed time, per monitored slot (CoreMask): elapsed_q restarts at 0 on the
//    dispatch and counts event_i.tick pulses while the slot is busy and not fenced,
//    saturating at all ones. Heartbeats do not restart it; it holds after the done.
//
// 2. Budget (ctrl_i, BINGO_WD_BUDGET; reset 0 = off):
//      [0]     EN      budget on
//      [1]     ACTION  at the budget: 0 one late beat, 1 a fence
//      [4:2]   SH_B    budget = max(E_tot << SH_B, BFLOOR)
//      [6:5]   FRAC_B  late threshold at the done = E_tot + (E_tot >> 2) * FRAC_B
//      [31:12] BFLOOR  ticks
//    E_tot is the estimate of the slot's (core type, class) (est_i); a task away from
//    its own logical slot (task_i.moved) uses E_tot << MOVED_SH (BINGO_WD_LATE[8:7]),
//    as the learned bounds do. A slot without a valid estimate has no budget.
//
// 3. Per slot:
//    fence_o  ACTION 1, a busy slot whose elapsed time reached its budget (a level:
//             bingo_hw_manager_watchdog fences it as on a confirm timeout)
//    late_o   one-cycle pulse, at most one per execution: ACTION 0 and the elapsed
//             time just reached the budget, or a done at or after the late
//             threshold (fault precursor for bingo_hw_manager_risk)
module bingo_hw_manager_wd_budget #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    // Width of the elapsed-time counters (the watchdog timer's)
    parameter int unsigned CounterWidth = 24,
    parameter logic [NumCores-1:0][NumClusters-1:0] CoreMask = '1,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         wd_event_t = logic,
    parameter type         wd_status_t = logic,
    parameter type         wd_task_t = logic,
    parameter type         wd_est_t = logic
) (
    input  logic                                 clk_i,
    input  logic                                 rst_ni,
    input  logic [31:0]                          ctrl_i,       // BINGO_WD_BUDGET
    input  logic [31:0]                          late_ctrl_i,  // BINGO_WD_LATE ([8:7] MOVED_SH)
    input  wd_event_t                            event_i,      // dispatch, done, tick
    input  wd_status_t                           status_i,     // busy, fenced
    input  wd_task_t                             task_i,       // moved
    input  wd_est_t                              est_i,        // E_tot, valid
    output logic [NumCores-1:0][NumClusters-1:0] fence_o,
    output logic [NumCores-1:0][NumClusters-1:0] late_o
);
    for (genvar c = 0; c < NumCores; c++) begin : gen_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_cluster
            if (CoreMask[c][cl]) begin : gen_budget
                logic [CounterWidth-1:0] elapsed_q;
                // A late beat of this execution was reported
                logic                    reported_q;
                logic [1:0]              moved_shift;
                logic [31:0]             e_tot, budget, late_bound;
                logic                    timed, over, budget_late, done_late;

                assign moved_shift = task_i.moved[c][cl] ? late_ctrl_i[8:7] : 2'd0;
                assign e_tot = 32'(est_i.e_tot[c][cl]) << moved_shift;
                // E_tot < 2^23 after MOVED_SH and < 2^30 after SH_B: no overflow in 32 bits
                assign budget = (e_tot << ctrl_i[4:2]) > 32'(ctrl_i[31:12]) ?
                                (e_tot << ctrl_i[4:2]) : 32'(ctrl_i[31:12]);
                assign late_bound = e_tot + (e_tot >> 2) * ctrl_i[6:5];
                assign timed = ctrl_i[0] && est_i.valid[c][cl] && status_i.busy[c][cl] &&
                               !status_i.fenced[c][cl];
                assign over = timed && 32'(elapsed_q) >= budget;
                assign budget_late = over && !ctrl_i[1] && !reported_q;
                assign done_late = timed && event_i.done[c][cl] && !reported_q &&
                                   32'(elapsed_q) >= late_bound;
                assign fence_o[c][cl] = over && ctrl_i[1];
                assign late_o[c][cl] = budget_late || done_late;

                always_ff @(posedge clk_i or negedge rst_ni) begin
                    if (!rst_ni) begin
                        elapsed_q <= '0;
                        reported_q <= 1'b0;
                    end else if (!status_i.fenced[c][cl] && !event_i.done[c][cl] &&
                                 event_i.dispatch[c][cl]) begin
                        // the watchdog's priority: done before dispatch
                        elapsed_q <= '0;
                        reported_q <= 1'b0;
                    end else begin
                        if (status_i.busy[c][cl] && !status_i.fenced[c][cl] && !event_i.done[c][cl] &&
                            event_i.tick[c][cl] && elapsed_q != {CounterWidth{1'b1}})
                            elapsed_q <= elapsed_q + 1'b1;
                        if (budget_late || done_late)
                            reported_q <= 1'b1;
                    end
                end
            end else begin : gen_none
                assign fence_o[c][cl] = 1'b0;
                assign late_o[c][cl] = 1'b0;
            end
        end
    end
endmodule
