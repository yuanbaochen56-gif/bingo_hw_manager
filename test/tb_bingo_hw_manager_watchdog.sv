`timescale 1ns/1ps

module tb_bingo_hw_manager_watchdog;

    localparam int unsigned NUM_CORES     = 2;
    localparam int unsigned NUM_CLUSTERS  = 1;
    localparam int unsigned COUNTER_WIDTH = 8;
    localparam int unsigned TIMEOUT       = 4;

    logic clk_i;
    logic rst_ni;

    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] task_dispatched_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] task_done_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] heartbeat_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] waiting_task_i;

    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_busy_o;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_available_o;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_dead_suspect_o;

    bingo_hw_manager_watchdog #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CounterWidth(COUNTER_WIDTH),
        .HeartbeatTimeoutCycles(TIMEOUT)
    ) dut (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .task_dispatched_i(task_dispatched_i),
        .task_done_i(task_done_i),
        .heartbeat_i(heartbeat_i),
        .waiting_task_i(waiting_task_i),
        .core_busy_o(core_busy_o),
        .core_available_o(core_available_o),
        .core_dead_suspect_o(core_dead_suspect_o)
    );

    initial clk_i = 1'b0;
    always #5 clk_i = ~clk_i;

    task automatic clear_inputs;
    begin
        task_dispatched_i = '0;
        task_done_i       = '0;
        heartbeat_i       = '0;
        waiting_task_i    = '0;
    end
    endtask

    task automatic tick(input int unsigned cycles = 1);
    begin
        repeat (cycles) @(posedge clk_i);
        #1;
    end
    endtask

    task automatic pulse_dispatch(input int unsigned core);
    begin
        task_dispatched_i = '0;
        task_dispatched_i[core][0] = 1'b1;
        @(posedge clk_i);
        #1;
        task_dispatched_i[core][0] = 1'b0;
    end
    endtask

    task automatic pulse_done(input int unsigned core);
    begin
        task_done_i = '0;
        task_done_i[core][0] = 1'b1;
        @(posedge clk_i);
        #1;
        task_done_i[core][0] = 1'b0;
    end
    endtask

    task automatic pulse_heartbeat(input int unsigned core);
    begin
        heartbeat_i = '0;
        heartbeat_i[core][0] = 1'b1;
        @(posedge clk_i);
        #1;
        heartbeat_i[core][0] = 1'b0;
    end
    endtask

    task automatic expect_core_state(
        input int unsigned core,
        input logic expected_busy,
        input logic expected_available,
        input logic expected_dead
    );
    begin
        if (core_busy_o[core][0] !== expected_busy) begin
            $error("core %0d busy mismatch: expected %0b got %0b",
                   core, expected_busy, core_busy_o[core][0]);
        end
        if (core_available_o[core][0] !== expected_available) begin
            $error("core %0d available mismatch: expected %0b got %0b",
                   core, expected_available, core_available_o[core][0]);
        end
        if (core_dead_suspect_o[core][0] !== expected_dead) begin
            $error("core %0d dead_suspect mismatch: expected %0b got %0b",
                   core, expected_dead, core_dead_suspect_o[core][0]);
        end
    end
    endtask

    initial begin
        clear_inputs();

        rst_ni = 1'b0;
        repeat (2) @(posedge clk_i);
        rst_ni = 1'b1;
        tick();

        $display("Checking reset state");
        expect_core_state(0, 1'b0, 1'b0, 1'b0);
        expect_core_state(1, 1'b0, 1'b0, 1'b0);

        $display("Marking both cores as waiting for tasks");
        waiting_task_i[0][0] = 1'b1;
        waiting_task_i[1][0] = 1'b1;
        tick();
        expect_core_state(0, 1'b0, 1'b1, 1'b0);
        expect_core_state(1, 1'b0, 1'b1, 1'b0);

        $display("Dispatching task to core 0");
        pulse_dispatch(0);
        expect_core_state(0, 1'b1, 1'b0, 1'b0);
        expect_core_state(1, 1'b0, 1'b1, 1'b0);

        $display("Heartbeat should keep core 0 alive during a long run");
        tick(TIMEOUT - 1);
        expect_core_state(0, 1'b1, 1'b0, 1'b0);
        pulse_heartbeat(0);
        expect_core_state(0, 1'b1, 1'b0, 1'b0);

        $display("Letting the timeout expire without heartbeat");
        tick(TIMEOUT - 1);
        expect_core_state(0, 1'b1, 1'b0, 1'b0);
        tick(1);
        expect_core_state(0, 1'b1, 1'b0, 1'b1);
        expect_core_state(1, 1'b0, 1'b1, 1'b0);

        $display("Done should clear busy and dead_suspect");
        pulse_done(0);
        expect_core_state(0, 1'b0, 1'b1, 1'b0);
        expect_core_state(1, 1'b0, 1'b1, 1'b0);

        $display("All watchdog tests passed");
        #10;
        $finish;
    end

    initial begin
        #10000;
        $fatal("TIMEOUT");
    end

endmodule
