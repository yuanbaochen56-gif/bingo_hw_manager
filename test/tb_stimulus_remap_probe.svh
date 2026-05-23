localparam int unsigned EXPECTED_TASK_COUNT      = 999;
localparam int unsigned DEADLOCK_THRESHOLD       = 1000000;
localparam int unsigned DEP_MATRIX_LOG_INTERVAL  = 0;

bingo_hw_manager_task_desc_full_t remap_probe_task = pack_normal_task(
    2'b00, 16'd1, 0, 0, 0,//task info
    1'b1, bingo_hw_manager_dep_code_t'(3'b001),  // Set a dependency code  so the task will block in the waiting queue and trigger remap selection.
    1'b0, 1'b0, 0, 0, '0// set info
);

initial begin : remap_probe_test
    automatic axi_pkg::resp_t resp;
    automatic device_axi_lite_data_t unused_task_id;

    wait (rst_ni);
    repeat (20) @(posedge clk_i);

    // Make physical core 1 look available to the watchdog/remap selector.
    // The read blocks because no task is routed to core 1's ready queue.
    fork
        begin
            csr_read(0, 0, 1, CSR_READY, unused_task_id); // cluster 0, core 1
        end
    join_none

    repeat (5) @(posedge clk_i);

    if (gen_dut[0].i_dut.core_available[1][0] !== 1'b1) begin
        $fatal(1, "core 1 should be available before remap selection");
    end

    task_queue_master[0].write(task_queue_base[0], '0, remap_probe_task, '1, resp);

    repeat (10) @(posedge clk_i);
    #1;

    if (gen_dut[0].i_dut.remap_select_valid[0] !== 1'b1) begin
        $fatal(1, "logical core 0 remap_select_valid should assert");
    end

    if (gen_dut[0].i_dut.remap_physical_core[0] !== bingo_hw_manager_assigned_core_id_t'(1)) begin
        $fatal(1, "logical core 0 should fall back to physical core 1, got %0d",
               gen_dut[0].i_dut.remap_physical_core[0]);
    end

    if (gen_dut[0].i_dut.remap_physical_cluster[0] !== bingo_hw_manager_assigned_cluster_id_t'(0)) begin
        $fatal(1, "logical core 0 should stay in cluster 0");
    end

    disable fork;

    $display("Top-level remap probe test passed");
    $finish;
end
