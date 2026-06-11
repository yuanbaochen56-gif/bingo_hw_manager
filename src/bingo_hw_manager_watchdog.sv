module bingo_hw_manager_watchdog #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CounterWidth = 24,
    parameter int unsigned HeartbeatTimeoutCycles = 100000
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  logic [NumCores-1:0][NumClusters-1:0] task_dispatched_i, // Signal indicating a new task has been dispatched to a core
    input  logic [NumCores-1:0][NumClusters-1:0] task_done_i,       // Signal indicating a core has completed its current task
    input  logic [NumCores-1:0][NumClusters-1:0] heartbeat_i,       // Heartbeat signal from each core to indicate it's alive
    input  logic [NumCores-1:0][NumClusters-1:0] waiting_task_i,    // Signal indicating a task is waiting to be dispatched to a core

    output logic [NumCores-1:0][NumClusters-1:0] core_busy_o,      // Indicates when a core is currently executing a task
    output logic [NumCores-1:0][NumClusters-1:0] core_available_o,  // Indicates when a core is available for new tasks
    output logic [NumCores-1:0][NumClusters-1:0] core_dead_suspect_o    // Indicates when a core is suspected to be dead (no heartbeat for too long)
);

    // Internal registers to track core status and timers
    logic [CounterWidth-1:0] timer_q [NumCores][NumClusters];
    logic                    busy_q [NumCores][NumClusters];
    
    // Watchdog logic: Update core status and timers
    always_comb begin
        for(int c=0; c<NumCores; c++) begin
            for(int cl=0; cl<NumClusters; cl++) begin
                core_busy_o[c][cl] = busy_q[c][cl];
                core_dead_suspect_o[c][cl] =busy_q[c][cl] &&(timer_q[c][cl] >= HeartbeatTimeoutCycles[CounterWidth-1:0]);
                core_available_o[c][cl] = waiting_task_i[c][cl] && !busy_q[c][cl] && !core_dead_suspect_o[c][cl];
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
                end
            end
        end else begin
            for(int c=0; c<NumCores; c++) begin
                for(int cl=0; cl<NumClusters; cl++) begin
                    // Check for task completion, new task dispatch, and heartbeat
                    if(task_done_i[c][cl]) begin
                        busy_q[c][cl] <= 1'b0; 
                        timer_q[c][cl] <= '0;
                    end else if(task_dispatched_i[c][cl]) begin
                        busy_q[c][cl] <= 1'b1; 
                        timer_q[c][cl] <= '0;
                    end else if (heartbeat_i[c][cl]) begin
                        timer_q[c][cl] <= '0; // Reset timer on heartbeat
                    end else if (busy_q[c][cl] &&timer_q[c][cl] != {CounterWidth{1'b1}}) begin
                        timer_q[c][cl] <= timer_q[c][cl] + 1'b1; // Increment timer if core is busy and no heartbeat
                    end
                end
            end
        end
    end

endmodule