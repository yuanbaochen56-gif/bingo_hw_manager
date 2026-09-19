localparam int unsigned EXPECTED_TASK_COUNT      = 999;
localparam int unsigned DEADLOCK_THRESHOLD       = 1000000;
localparam int unsigned DEP_MATRIX_LOG_INTERVAL  = 0;

bingo_hw_manager_task_desc_full_t passthrough_task = pack_normal_task(
    2'b00, 16'd1, 0, 0, 0,
    1'b0, '0,
    1'b0, 1'b0, 0, 0, '0
);

initial begin : remap_passthrough_test
    automatic axi_pkg::resp_t resp;
    automatic bit core0_read_done;
    automatic device_axi_lite_data_t core0_task_id;

    wait (rst_ni);
    repeat (20) @(posedge clk_i);

    core0_read_done = 1'b0;
    core0_task_id = '0;

    // Make logical core 0 available by issuing a blocking CSR read.
    fork
        begin : core0_ready_reader
            csr_read(0, 0, 0, CSR_READY, core0_task_id);
            core0_read_done = 1'b1;
        end
    join_none

    repeat (5) @(posedge clk_i);

    if (gen_dut[0].i_dut.core_available[0][0] !== 1'b1) begin
        $fatal(1, "core 0 should be available before dispatch");
    end

    $display("[PASSTHROUGH] Push task 1 targeting logical core 0");
    task_queue_master[0].write(task_queue_base[0], '0, passthrough_task, '1, resp);

    fork : wait_core0_or_timeout
        begin
            wait (core0_read_done);
        end
        begin
            repeat (300) @(posedge clk_i);
            if (!core0_read_done) begin
                dump_queue_state();
                $fatal(1, "core 0 did not receive task 1 — unexpected remap?");
            end
        end
    join_any
    disable wait_core0_or_timeout;

    if (core0_task_id[TaskIdWidth-1:0] !== bingo_hw_manager_task_id_t'(16'd1)) begin
        $fatal(1, "core 0 read wrong task id: expected 1 got %0d",
               core0_task_id[TaskIdWidth-1:0]);
    end

    $display("Remap passthrough test passed — task stayed on logical core 0");
    $finish;
end
