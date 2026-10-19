// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Frequency-aware watchdog ticks of the control plane (bingo_hw_manager_ctrl).
// The power levels are clock dividers. While a slot's domain runs at level L
// above the normal level N (slower), its core makes progress N/L times as fast,
// so its heartbeat gaps stretch by L/N in manager cycles. wd_tick_o advances
// the slot's watchdog timer only N/L of the cycles (a fractional accumulator),
// so the timeouts count cycles of the normal clock and can be set for the
// normal level instead of the slowest one. The level is the one the DFS path
// applied to the domain, or the one the host acknowledged in DVFS mode
// (pm_i.dvfs); without idle PM, or at or below the normal level, the timer
// advances every cycle. wd_fast_o marks a slot whose domain runs at a known
// level below the normal one (a faster clock, e.g. boost).
module bingo_hw_manager_wd_tick
    import bingo_hw_manager_pkg::*;
#(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2
) (
    input  logic clk_i,
    input  logic rst_ni,

    // Power state (bingo_hw_manager_pm), and the domain of every slot (>= 32: none)
    input  pm_view_t                                  pm_i,
    input  logic [NumCores-1:0][NumClusters-1:0][5:0] slot_domain_i,

    // Watchdog: the slot's timer advances this cycle
    output logic [NumCores-1:0][NumClusters-1:0]      wd_tick_o,
    // Adaptive bounds: the slot's domain runs faster than the normal level
    output logic [NumCores-1:0][NumClusters-1:0]      wd_fast_o
);

    logic [NumCores-1:0][NumClusters-1:0][7:0] slot_level;
    logic [NumCores-1:0][NumClusters-1:0]      slot_slow;
    logic [NumCores-1:0][NumClusters-1:0][8:0] tick_acc_q, tick_acc_d;
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                slot_level[c][cl] = pm_i.dvfs ? pm_i.dvfs_level :
                                    (slot_domain_i[c][cl] < 32) ? pm_i.domain_level[slot_domain_i[c][cl][4:0]] : 8'd0;
                slot_slow[c][cl]  = pm_i.enable && (pm_i.normal_level != '0) && (slot_level[c][cl] > pm_i.normal_level);
                wd_fast_o[c][cl]  = pm_i.enable && (pm_i.normal_level != '0) && (slot_level[c][cl] != '0) &&
                                    (slot_level[c][cl] < pm_i.normal_level);
                // Accumulate N per cycle, tick (and subtract L) once it reaches L
                if (!slot_slow[c][cl]) begin
                    wd_tick_o[c][cl]  = 1'b1;
                    tick_acc_d[c][cl] = '0;
                end else if ((tick_acc_q[c][cl] + 9'(pm_i.normal_level)) >= 9'(slot_level[c][cl])) begin
                    wd_tick_o[c][cl]  = 1'b1;
                    tick_acc_d[c][cl] = tick_acc_q[c][cl] + 9'(pm_i.normal_level) - 9'(slot_level[c][cl]);
                end else begin
                    wd_tick_o[c][cl]  = 1'b0;
                    tick_acc_d[c][cl] = tick_acc_q[c][cl] + 9'(pm_i.normal_level);
                end
            end
        end
    end
    // A slot at or below the normal level keeps its accumulator at 0: written
    // (and clocked) only while its domain is slow, or to clear it once
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            tick_acc_q <= '0;
        end else begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (slot_slow[c][cl] || (tick_acc_q[c][cl] != '0)) tick_acc_q[c][cl] <= tick_acc_d[c][cl];
                end
            end
        end
    end

endmodule
