localparam int unsigned EXPECTED_TASK_COUNT      = 999;
localparam int unsigned DEADLOCK_THRESHOLD       = 1000000;
localparam int unsigned DEP_MATRIX_LOG_INTERVAL  = 0;

// Task 1: dispatch to core 0, which will become dead_suspect.
bingo_hw_manager_task_desc_full_t task_to_die = pack_normal_task(
    2'b00, 16'd1, 0, 0, 0,
    1'b0, '0,
    1'b0, 1'b0, 0, 0, '0
);

// Task 2: also targets logical core 0, but should be remapped to core 1
// because core 0 is dead_suspect.
bingo_hw_manager_task_desc_full_t task_after_dead = pack_normal_task(
    2'b00, 16'd2, 0, 0, 0,
    1'b0, '0,
    1'b0, 1'b0, 0, 0, '0
);

initial begin : remap_dead_suspect_test
    automatic axi_pkg::resp_t resp;
    automatic device_axi_lite_data_t core0_task_id;
    automatic device_axi_lite_data_t core1_task_id;
    automatic bit core1_read_done;

    wait (rst_ni);
    repeat (20) @(posedge clk_i);

    // --- Phase 1: dispatch task 1 to core 0 and let it become dead ---

    // Make core 0 available.
    fork
        begin
            csr_read(0, 0, 0, CSR_READY, core0_task_id);
        end
    join_none
    repeat (5) @(posedge clk_i);

    $display("[DEAD_REMAP] Push task 1 to logical core 0");
    task_queue_master[0].write(task_queue_base[0], '0, task_to_die, '1, resp);

    // Wait for core 0 to pick up task 1.
    fork : wait_task1
        begin
            wait (gen_dut[0].i_dut.core_busy[0][0] === 1'b1);
        end
        begin
            repeat (300) @(posedge clk_i);
            $fatal(1, "core 0 never became busy with task 1");
        end
    join_any
    disable wait_task1;

    // Do NOT send heartbeat — let the watchdog timer expire.
    $display("[DEAD_REMAP] Waiting for core 0 to become dead_suspect (no heartbeat)");
    fork : wait_dead
        begin
            wait (gen_dut[0].i_dut.core_dead_suspect[0][0] === 1'b1);
        end
        begin
            repeat (500) @(posedge clk_i);
            $fatal(1, "core 0 did not become dead_suspect after timeout");
        end
    join_any
    disable wait_dead;

    $display("[DEAD_REMAP] core 0 is now dead_suspect");

    // --- Phase 2: dispatch task 2 to logical core 0 — expect remap to core 1 ---

    core1_read_done = 1'b0;
    core1_task_id = '0;

    // Make core 1 available.
    fork
        begin : core1_reader
            csr_read(0, 0, 1, CSR_READY, core1_task_id);
            core1_read_done = 1'b1;
        end
    join_none

    repeat (5) @(posedge clk_i);

    if (gen_dut[0].i_dut.core_available[1][0] !== 1'b1) begin
        $fatal(1, "core 1 should be available before remap dispatch");
    end

    $display("[DEAD_REMAP] Push task 2 to logical core 0 (expect remap to core 1)");
    task_queue_master[0].write(task_queue_base[0], '0, task_after_dead, '1, resp);

    fork : wait_core1_or_timeout
        begin
            wait (core1_read_done);
        end
        begin
            repeat (300) @(posedge clk_i);
            if (!core1_read_done) begin
                dump_queue_state();
                $fatal(1, "task 2 was not remapped to core 1");
            end
        end
    join_any
    disable wait_core1_or_timeout;

    if (core1_task_id[TaskIdWidth-1:0] !== bingo_hw_manager_task_id_t'(16'd2)) begin
        $fatal(1, "core 1 read wrong task id: expected 2 got %0d",
               core1_task_id[TaskIdWidth-1:0]);
    end

    $display("Dead-suspect remap test passed — task 2 remapped from dead core 0 to core 1");
    $finish;
end
