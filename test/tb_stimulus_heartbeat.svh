// =============================================================================
// Top-level CSR heartbeat path test
// =============================================================================

localparam int unsigned EXPECTED_TASK_COUNT      = 999;
localparam int unsigned DEADLOCK_THRESHOLD       = 1000000;
localparam int unsigned DEP_MATRIX_LOG_INTERVAL  = 0;

localparam device_axi_lite_addr_t CSR_HEARTBEAT = device_axi_lite_addr_t'(12'h5fd);

initial begin : heartbeat_path_test
    wait (rst_ni);
    repeat (20) @(posedge clk_i);

    // Send heartbeat CSR write from chip 0, cluster 0, core 0.
    csr_write(0, 0, 0, CSR_HEARTBEAT, device_axi_lite_data_t'(32'h0000_1234));

    // csr_write returns after csr_req_ready is observed. heartbeat_valid is
    // combinational from the CSR request, so sample immediately after return.
    #1;

    if (gen_dut[0].i_dut.heartbeat_valid[0][0] !== 1'b1) begin
        $fatal(1, "heartbeat_valid[0][0] did not assert after CSR heartbeat write");
    end

    @(posedge clk_i);
    #1;

    if (gen_dut[0].i_dut.heartbeat_valid[0][0] !== 1'b0) begin
        $fatal(1, "heartbeat_valid[0][0] should deassert after CSR heartbeat write completes");
    end

    if (gen_dut[0].i_dut.core_dead_suspect[0][0] !== 1'b0) begin
        $fatal(1, "core 0 should not be dead_suspect after heartbeat");
    end

    $display("Top-level CSR heartbeat path test passed");
    $finish;
end