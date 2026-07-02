localparam int unsigned EXPECTED_TASK_COUNT      = 999;
localparam int unsigned DEADLOCK_THRESHOLD       = 1000000;
localparam int unsigned DEP_MATRIX_LOG_INTERVAL  = 0;

// Task A is logically assigned to core 0, but core 1 will be the only
// available physical executor. It sets a dependency for logical core 2.
bingo_hw_manager_task_desc_full_t remap_source_task = pack_normal_task(
    2'b00, 16'd1, 0, 0, 0,
    1'b0, '0,
    1'b1, 1'b0, 0, 0, bingo_hw_manager_dep_code_t'(3'b100)
);

// Task B waits on logical core 0. If task A runs on physical core 1, the
// dep-matrix set must still use logical core 0 as the source column.
bingo_hw_manager_task_desc_full_t remap_dependent_task = pack_normal_task(
    2'b00, 16'd2, 0, 0, 2,
    1'b1, bingo_hw_manager_dep_code_t'(3'b001),
    1'b0, 1'b0, 0, 0, '0
);

initial begin : remap_full_flow_test
    automatic axi_pkg::resp_t resp;
    automatic bit core1_read_done;
    automatic bit core2_read_done;
    automatic device_axi_lite_data_t core1_task_id;
    automatic device_axi_lite_data_t core2_task_id;
    automatic bingo_hw_manager_done_info_full_t done_info;
    automatic device_axi_lite_data_t done_payload;

    wait (rst_ni);
    repeat (20) @(posedge clk_i);

    core1_read_done = 1'b0;
    core2_read_done = 1'b0;
    core1_task_id = '0;
    core2_task_id = '0;

    // Make physical core 1 available and keep its CSR read pending. The
    // remapped source task should complete this read.
    fork
        begin : core1_ready_reader
            csr_read(0, 0, 1, CSR_READY, core1_task_id);
            core1_read_done = 1'b1;
        end
    join_none

    repeat (5) @(posedge clk_i);

    if (gen_dut[0].i_dut.core_available[1][0] !== 1'b1) begin
        $fatal(1, "core 1 should be available before remap dispatch");
    end

    if (gen_dut[0].i_dut.core_available[0][0] !== 1'b0) begin
        $fatal(1, "core 0 should not be available; no CSR read is pending");
    end

    $display("[REMAP_FULL] Push source task 1: logical core 0, expected physical core 1");
    task_queue_master[0].write(task_queue_base[0], '0, remap_source_task, '1, resp);

    $display("[REMAP_FULL] Push dependent task 2: waits on logical core 0");
    task_queue_master[0].write(task_queue_base[0], '0, remap_dependent_task, '1, resp);

    fork : wait_core1_or_timeout
        begin
            wait (core1_read_done);
        end
        begin
            repeat (300) @(posedge clk_i);
            if (!core1_read_done) begin
                dump_queue_state();
                $fatal(1, "physical core 1 did not receive remapped task 1");
            end
        end
    join_any
    disable wait_core1_or_timeout;

    if (core1_task_id[TaskIdWidth-1:0] !== bingo_hw_manager_task_id_t'(16'd1)) begin
        $fatal(1, "physical core 1 read wrong task id: expected 1 got %0d",
               core1_task_id[TaskIdWidth-1:0]);
    end

    // Start a blocking read on logical/physical core 2 before completing task 1.
    // Task 2 should be released only after task 1 sets logical core 0's column.
    fork
        begin : core2_ready_reader
            csr_read(0, 0, 2, CSR_READY, core2_task_id);
            core2_read_done = 1'b1;
        end
    join_none

    repeat (5) @(posedge clk_i);

    done_info = '0;
    done_info.task_id = core1_task_id[TaskIdWidth-1:0];
    done_info.assigned_cluster_id = bingo_hw_manager_assigned_cluster_id_t'(0);
    done_info.assigned_core_id = bingo_hw_manager_assigned_core_id_t'(1);
    done_payload = device_axi_lite_data_t'(done_info);

    $display("[REMAP_FULL] Complete source task 1 from physical core 1");
    csr_write(0, 0, 1, CSR_DONE, done_payload);

    fork : wait_core2_or_timeout
        begin
            wait (core2_read_done);
        end
        begin
            repeat (300) @(posedge clk_i);
            if (!core2_read_done) begin
                dump_dep_matrix_state();
                dump_queue_state();
                $fatal(1, "dependent task 2 was not released by logical core 0 dep_set");
            end
        end
    join_any
    disable wait_core2_or_timeout;

    if (core2_task_id[TaskIdWidth-1:0] !== bingo_hw_manager_task_id_t'(16'd2)) begin
        $fatal(1, "core 2 read wrong task id: expected 2 got %0d",
               core2_task_id[TaskIdWidth-1:0]);
    end

    $display("Top-level remap full-flow test passed");
    $finish;
end
