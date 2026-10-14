// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Power and load view of the slots for the control plane (bingo_hw_manager_ctrl),
// fault-aware: a fenced slot is dead, so it no longer keeps its power domain at
// the normal level (pm_idle_o: polling for a task, or fenced) and no longer
// counts as load (load_clear_o). A slot that is only dead_suspect may still be
// working and keeps both.
// Idle entry delay (idle_delay_i, host-configured): a slot only counts as idle
// for the PM once it has been idle for idle_delay_i cycles, so a domain only
// drops to the idle level after all its slots were idle that long. Short gaps
// between tasks then keep the normal level and avoid the wake-up cost; 0 =
// at once (the PM's own behaviour).
// External access wake (access_hold_i, host-configured): a cluster whose memory
// is accessed from outside (cluster_access_i, e.g. the host reading its L1)
// runs on its own clock, so while it is accessed, and for access_hold_i cycles
// after the last access, none of its slots counts as idle, not even a fenced
// one. The access path is not held at the idle level; 0 = off. This only ever
// makes a slot idle later: the idle entry delay keeps counting the core's own
// polling, so once the hold ends a long idle core lets its domain drop at once.
// Recovery hold (recovery_hold_i, host-configured, 0 = off): a fence writes the
// slot mapping table first. On the following edge the live substitute of the
// fenced slot (unless that logical slot has already retired) does not count as
// idle for recovery_hold_i cycles, the recovery window. Entries rewritten
// because their substitute was fenced again load their new substitute the same
// way.
module bingo_hw_manager_power_view #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1
) (
    input  logic clk_i,
    input  logic rst_ni,

    // Watchdog: confirmed dead slots (sticky)
    input  logic [NumCores-1:0][NumClusters-1:0]                     fenced_i,
    // Slots polling their ready queue (idle)
    input  logic [NumCores-1:0][NumClusters-1:0]                     waiting_i,
    // Logical slots with no task left
    input  logic [NumCores-1:0][NumClusters-1:0]                     retired_i,
    // Slot mapping table, indexed by logical slot, and the entries written
    // this cycle
    input  logic [NumCores-1:0][NumClusters-1:0]                     smt_found_i,
    input  logic [NumCores-1:0][NumClusters-1:0][CoreIdWidth-1:0]    smt_core_i,
    input  logic [NumCores-1:0][NumClusters-1:0][ClusterIdWidth-1:0] smt_cluster_i,
    input  logic [NumCores-1:0][NumClusters-1:0]                     smt_recompute_i,
    // Cycles a slot must be idle before the PM may count it as idle (0: at once)
    input  logic [31:0]                                              idle_delay_i,
    // External access to a cluster's memory this cycle, and the cycles its
    // slots stay awake after the last one (0: off)
    input  logic [NumClusters-1:0]                                   cluster_access_i,
    input  logic [31:0]                                              access_hold_i,
    // Recovery hold in top-clock cycles, 32 bits (0: off, max 2^32-1).
    input  logic [31:0]                                              recovery_hold_i,

    // Power manager: slots that do not keep their domain at the normal level
    output logic [NumCores-1:0][NumClusters-1:0]                     pm_idle_o,
    // Post-edge recovery activity for the event logger (not a PM input).
    output logic [NumCores-1:0][NumClusters-1:0]                     rec_hold_o,
    // Slots otherwise PM-idle, awake only because their cluster is accessed.
    output logic [NumCores-1:0][NumClusters-1:0]                     access_only_o,
    // Load monitor: slots whose pending count is cleared
    output logic [NumCores-1:0][NumClusters-1:0]                     load_clear_o
);

    // ------------------------------------------------------------------
    // Recovery hold
    // ------------------------------------------------------------------
    // A fence writes the SMT first. On the following edge, load its current
    // live substitute, unless the logical slot has already retired. Include
    // affected dead entries when their previous substitute is fenced again.
    logic [NumCores-1:0][NumClusters-1:0] rec_pending_q, rec_load;
    logic [NumCores-1:0][NumClusters-1:0][31:0] rec_hold_q, rec_hold_d;
    always_comb begin
        rec_load = '0;
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                if (rec_pending_q[c][cl] && !retired_i[c][cl] && smt_found_i[c][cl] &&
                    !fenced_i[smt_core_i[c][cl]][smt_cluster_i[c][cl]])
                    rec_load[smt_core_i[c][cl]][smt_cluster_i[c][cl]] = 1'b1;
            end
        end
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                rec_hold_d[c][cl] = rec_hold_q[c][cl];
                if (fenced_i[c][cl]) rec_hold_d[c][cl] = '0;
                else if (rec_load[c][cl] && recovery_hold_i != 0)
                    rec_hold_d[c][cl] = recovery_hold_i;
                else if (rec_hold_q[c][cl] != 0)
                    rec_hold_d[c][cl] = rec_hold_q[c][cl] - 32'd1;
                // The logger samples this on the very edge the count changes,
                // so pending-event ages refer to load/zero, not one edge later.
                rec_hold_o[c][cl] = rec_hold_d[c][cl] != 0;
            end
        end
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rec_pending_q <= '0;
            rec_hold_q <= '0;
        end else begin
            rec_pending_q <= smt_recompute_i & fenced_i & ~retired_i;
            rec_hold_q <= rec_hold_d;
        end
    end

    // ------------------------------------------------------------------
    // Fault-aware power and load view
    // ------------------------------------------------------------------
    // Clusters accessed from outside within the last access_hold_i cycles
    logic [NumClusters-1:0]       cluster_awake;
    logic [NumClusters-1:0][31:0] access_cnt_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            access_cnt_q <= '0;
        end else begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (cluster_access_i[cl])        access_cnt_q[cl] <= access_hold_i;
                else if (access_cnt_q[cl] != '0) access_cnt_q[cl] <= access_cnt_q[cl] - 32'd1;
            end
        end
    end
    for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_cluster_awake
        assign cluster_awake[cl] = (access_hold_i != '0) && (cluster_access_i[cl] || (access_cnt_q[cl] != '0));
    end
    logic [NumCores-1:0][NumClusters-1:0]       slot_idle;
    logic [NumCores-1:0][NumClusters-1:0][31:0] idle_cnt_q;
    assign slot_idle = waiting_i | fenced_i;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            idle_cnt_q <= '0;
        end else begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (!slot_idle[c][cl])                    idle_cnt_q[c][cl] <= '0;
                    else if (idle_cnt_q[c][cl] != '1)         idle_cnt_q[c][cl] <= idle_cnt_q[c][cl] + 32'd1;
                end
            end
        end
    end
    always_comb begin
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                pm_idle_o[c][cl] = slot_idle[c][cl] && (idle_cnt_q[c][cl] >= idle_delay_i) && !cluster_awake[cl];
                access_only_o[c][cl] = slot_idle[c][cl] && (idle_cnt_q[c][cl] >= idle_delay_i) &&
                                       cluster_awake[cl] && rec_hold_q[c][cl] == 0;
                pm_idle_o[c][cl] &= rec_hold_q[c][cl] == 0;
            end
        end
    end
    assign load_clear_o = fenced_i;

endmodule
