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
    parameter logic [NumCores-1:0][NumClusters-1:0] CoreMask = '1
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  logic [NumCores-1:0][NumClusters-1:0] task_dispatched_i, // Signal indicating a new task has been dispatched to a core
    input  logic [NumCores-1:0][NumClusters-1:0] task_done_i,       // Signal indicating a core has completed its current task
    input  logic [NumCores-1:0][NumClusters-1:0] heartbeat_i,       // Heartbeat signal from each core to indicate it's alive
    input  logic [NumCores-1:0][NumClusters-1:0] waiting_task_i,    // Signal indicating a task is waiting to be dispatched to a core
    // The timer of a busy core only advances on a tick. bingo_hw_manager_ctrl
    // slows the ticks down while the core's power domain runs below the normal
    // level, so the timeouts count cycles of the normal clock.
    input  logic [NumCores-1:0][NumClusters-1:0] tick_i = '1,
    // Late beat (fault precursor): a heartbeat or done of a busy core whose
    // timer has reached late_cycles_i. 0 = off. The timer counts ticks, so
    // like the timeouts this counts cycles of the normal clock.
    input  logic [CounterWidth-1:0]               late_cycles_i = '0,

    output logic [NumCores-1:0][NumClusters-1:0] core_busy_o,      // Indicates when a core is currently executing a task
    output logic [NumCores-1:0][NumClusters-1:0] core_available_o,  // Indicates when a core is available for new tasks
    output logic [NumCores-1:0][NumClusters-1:0] core_dead_suspect_o,   // Indicates when a core is suspected to be dead (no heartbeat for too long)
    output logic [NumCores-1:0][NumClusters-1:0] core_fenced_o,     // Confirmed dead (sticky): the core is isolated from the manager
    output logic [NumCores-1:0][NumClusters-1:0] late_o             // One-cycle pulse per late beat
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
                core_busy_o[c][cl] = busy_q[c][cl];
                core_fenced_o[c][cl] = fenced_q[c][cl];
                // A fenced core stays dead_suspect, so the event log never shows a
                // fake recovery.
                core_dead_suspect_o[c][cl] = fenced_q[c][cl] ||
                                             (CoreMask[c][cl] && busy_q[c][cl] &&
                                              (timer_q[c][cl] >= HeartbeatTimeoutCycles[CounterWidth-1:0]));
                core_available_o[c][cl] = waiting_task_i[c][cl] && !busy_q[c][cl] && !core_dead_suspect_o[c][cl];
                // Same priority as the timer update below: a fenced core is ignored
                late_o[c][cl] = (late_cycles_i != '0) && CoreMask[c][cl] && !fenced_q[c][cl] &&
                                busy_q[c][cl] && (task_done_i[c][cl] || heartbeat_i[c][cl]) &&
                                (timer_q[c][cl] >= late_cycles_i);
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
                    end else if(task_done_i[c][cl]) begin
                        busy_q[c][cl] <= 1'b0;
                        timer_q[c][cl] <= '0;
                    end else if(task_dispatched_i[c][cl]) begin
                        busy_q[c][cl] <= 1'b1;
                        timer_q[c][cl] <= '0;
                    end else if (heartbeat_i[c][cl]) begin
                        timer_q[c][cl] <= '0; // Reset timer on heartbeat
                    end else if ((ConfirmTimeoutCycles != 0) && CoreMask[c][cl] && busy_q[c][cl] &&
                                 (timer_q[c][cl] >= ConfirmTimeoutCycles[CounterWidth-1:0])) begin
                        // A done, dispatch or heartbeat in this cycle wins (branches above).
                        fenced_q[c][cl] <= 1'b1;
                        busy_q[c][cl] <= 1'b0;
                        timer_q[c][cl] <= '0;
                    end else if (busy_q[c][cl] && tick_i[c][cl] && timer_q[c][cl] != {CounterWidth{1'b1}}) begin
                        timer_q[c][cl] <= timer_q[c][cl] + 1'b1; // Increment timer if core is busy and no heartbeat
                    end
                end
            end
        end
    end

endmodule
