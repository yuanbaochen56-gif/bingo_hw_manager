// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Heartbeat watchdog of every (core, cluster) slot. A slot is busy from the
// dispatch of a task to its done; while busy, its timer counts ticks (cycles of
// the normal clock, bingo_hw_manager_wd_tick) since the dispatch or the last
// heartbeat. Two thresholds act on the timer:
//   suspect  the slot is reported dead_suspect (reversible: a heartbeat or done
//            clears the timer)
//   confirm  the slot is fenced, for good (sticky until reset): the manager
//            stops using it and replays its tasks (bingo_hw_manager_replay_ctrl)
// A heartbeat or done that comes after the late threshold is a late beat, a
// fault precursor for the risk counter of bingo_hw_manager_ctrl.
// The parameters give the default thresholds; bound_i may replace them per
// slot (bingo_hw_manager_wd_bounds: per core type, learned by
// bingo_hw_manager_wd_adapt, or shorter for an at-risk slot). This module only
// compares; every choice of threshold is made there.
module bingo_hw_manager_watchdog #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CounterWidth = 24,
    parameter int unsigned HeartbeatTimeoutCycles = 100000,
    // Second, longer timeout after which a busy core without heartbeat is fenced
    // for good (sticky until reset): its outstanding tasks are then replayed on
    // another core. 0 disables fencing (detection only).
    parameter int unsigned ConfirmTimeoutCycles = 0,
    // Slots with CoreMask = 0 are never reported dead_suspect (e.g. a host slot
    // that does not send heartbeats, or a tied-off slot).
    parameter logic [NumCores-1:0][NumClusters-1:0] CoreMask = '1,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter int unsigned WdClassWidth = 3,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId = '0,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         wd_event_t = logic,
    parameter type         wd_bound_t = logic,
    parameter type         wd_status_t = logic
) (
    input  logic       clk_i,
    input  logic       rst_ni,
    input  wd_event_t  event_i,   // dispatch, done, heartbeat, waiting, tick of every slot
    input  wd_bound_t  bound_i,   // thresholds (0 or not valid: the parameters)
    output wd_status_t status_o
);
    localparam int unsigned MaxTimeoutCycles =
        (ConfirmTimeoutCycles > HeartbeatTimeoutCycles) ? ConfirmTimeoutCycles : HeartbeatTimeoutCycles;

    // The timer saturates at 2^CounterWidth-1, so the timeouts must fit in it;
    // otherwise the truncated compare value would silently shorten a timeout.
    if ($clog2(MaxTimeoutCycles + 1) > CounterWidth) begin : gen_counter_width_check
        initial begin
            $error("Watchdog timeout (%0d cycles) does not fit in a %0d-bit counter",
                   MaxTimeoutCycles, CounterWidth);
            $finish;
        end
    end
    // Fencing is irreversible, so it must come strictly after the reversible suspicion.
    if ((ConfirmTimeoutCycles != 0) && (ConfirmTimeoutCycles <= HeartbeatTimeoutCycles)) begin : gen_confirm_timeout_check
        initial begin
            $error("Watchdog confirm timeout (%0d) must exceed the heartbeat timeout (%0d)",
                   ConfirmTimeoutCycles, HeartbeatTimeoutCycles);
            $finish;
        end
    end

    // Internal registers to track core status and timers
    logic [CounterWidth-1:0] timer_q [NumCores][NumClusters];
    logic                    busy_q [NumCores][NumClusters];
    logic                    fenced_q [NumCores][NumClusters];

    // Watchdog logic: Update core status and timers
    always_comb begin
        for(int c=0; c<NumCores; c++) begin
            for(int cl=0; cl<NumClusters; cl++) begin
                automatic logic [CounterWidth-1:0] late_cycles =
                    bound_i.late[c][cl] != '0 ? bound_i.late[c][cl] : bound_i.late_global;
                status_o.busy[c][cl] = busy_q[c][cl];
                status_o.fenced[c][cl] = fenced_q[c][cl];
                // A fenced core stays dead_suspect, so the event log never shows a
                // fake recovery.
                status_o.dead_suspect[c][cl] = fenced_q[c][cl] ||
                                               (CoreMask[c][cl] && busy_q[c][cl] &&
                                                (timer_q[c][cl] >= ((bound_i.suspect[c][cl] != '0 || bound_i.suspect_valid[c][cl]) ?
                                                 bound_i.suspect[c][cl] : HeartbeatTimeoutCycles[CounterWidth-1:0])));
                status_o.available[c][cl] = event_i.waiting[c][cl] && !busy_q[c][cl] && !status_o.dead_suspect[c][cl];
                // Same priority as the timer update below: a fenced core is ignored
                status_o.late[c][cl] = (late_cycles != '0) && CoreMask[c][cl] && !fenced_q[c][cl] &&
                                       busy_q[c][cl] && (event_i.done[c][cl] || event_i.heartbeat[c][cl]) &&
                                       (timer_q[c][cl] >= late_cycles);
                // Matches done -> dispatch -> heartbeat priority. No fenced
                // interval or heartbeat concurrent with dispatch is learned.
                status_o.progress[c][cl] = CoreMask[c][cl] && !fenced_q[c][cl] && busy_q[c][cl] &&
                    (event_i.done[c][cl] || (!event_i.dispatch[c][cl] && event_i.heartbeat[c][cl]));
                status_o.interval[c][cl] = timer_q[c][cl];
            end
        end
    end

    // Sequential logic to update busy status and timers
    always_ff @(posedge clk_i or negedge rst_ni)begin
        if(!rst_ni) begin
            for (int c = 0; c < NumCores; c++) begin
                for (int cl = 0; cl < NumClusters; cl++) begin
                    busy_q[c][cl] <= 1'b0;
                    timer_q[c][cl] <= '0;
                    fenced_q[c][cl] <= 1'b0;
                end
            end
        end else begin
            for(int c=0; c<NumCores; c++) begin
                for(int cl=0; cl<NumClusters; cl++) begin
                    if (fenced_q[c][cl]) begin
                        // Fenced: ignore the core for good.
                        busy_q[c][cl] <= 1'b0;
                        timer_q[c][cl] <= '0;
                    // Check for task completion, new task dispatch, and heartbeat
                    end else if(event_i.done[c][cl]) begin
                        busy_q[c][cl] <= 1'b0;
                        timer_q[c][cl] <= '0;
                    end else if(event_i.dispatch[c][cl]) begin
                        busy_q[c][cl] <= 1'b1;
                        timer_q[c][cl] <= '0;
                    end else if (event_i.heartbeat[c][cl]) begin
                        timer_q[c][cl] <= '0; // Reset timer on heartbeat
                    end else if ((ConfirmTimeoutCycles != 0) && CoreMask[c][cl] && busy_q[c][cl] &&
                                 (timer_q[c][cl] >= ((bound_i.risk[c][cl] && bound_i.risk_use[c][cl] && bound_i.risk_confirm != '0) ?
                                  bound_i.risk_confirm : ((bound_i.confirm[c][cl] != '0) ?
                                  bound_i.confirm[c][cl] : ConfirmTimeoutCycles[CounterWidth-1:0])))) begin
                        // A done, dispatch or heartbeat in this cycle wins (branches above).
                        fenced_q[c][cl] <= 1'b1;
                        busy_q[c][cl] <= 1'b0;
                        timer_q[c][cl] <= '0;
`ifndef SYNTHESIS
                        if (bound_i.adapt_confirm[c][cl])
                            $display("[BINGO_ADAPT_CONFIRM] %0t core=%0d cluster=%0d type=%0d class=%0d threshold=%0d",
                                     $time, c, cl, CoreTypeId[c][cl], bound_i.wd_class[c][cl], bound_i.confirm[c][cl]);
                        else if (bound_i.risk[c][cl] && bound_i.risk_use[c][cl] && bound_i.risk_confirm != '0)
                            $display("[BINGO_RISK_CONFIRM] %0t core=%0d cluster=%0d threshold=%0d",
                                     $time, c, cl, bound_i.risk_confirm);
                        else if (bound_i.confirm[c][cl] != '0)
                            $display("[BINGO_TYPE_CONFIRM] %0t core=%0d cluster=%0d type=%0d threshold=%0d",
                                     $time, c, cl, CoreTypeId[c][cl], bound_i.confirm[c][cl]);
`endif
                    end else if (busy_q[c][cl] && event_i.tick[c][cl] && timer_q[c][cl] != {CounterWidth{1'b1}}) begin
                        timer_q[c][cl] <= timer_q[c][cl] + 1'b1; // Increment timer if core is busy and no heartbeat
                    end
                end
            end
        end
    end

endmodule
