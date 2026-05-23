`timescale 1ns/1ps

module tb_bingo_hw_manager_core_remap;

    localparam int unsigned NUM_CORES = 3;
    localparam int unsigned NUM_CLUSTERS = 1;
    localparam int unsigned CORE_ID_WIDTH = 2;
    localparam int unsigned CLUSTER_ID_WIDTH = 1;

    logic req_valid_i;
    logic [CORE_ID_WIDTH-1:0] logical_core_i;
    logic [CLUSTER_ID_WIDTH-1:0] logical_cluster_i;

    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_available_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_dead_suspect_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] ready_queue_full_i;

    logic select_valid_o;
    logic [CORE_ID_WIDTH-1:0] physical_core_o;
    logic [CLUSTER_ID_WIDTH-1:0] physical_cluster_o;

    bingo_hw_manager_core_remap #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CoreIdWidth(CORE_ID_WIDTH),
        .ClusterIdWidth(CLUSTER_ID_WIDTH)
    ) dut (
        .req_valid_i(req_valid_i),
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .core_available_i(core_available_i),
        .core_dead_suspect_i(core_dead_suspect_i),
        .ready_queue_full_i(ready_queue_full_i),
        .select_valid_o(select_valid_o),
        .physical_core_o(physical_core_o),
        .physical_cluster_o(physical_cluster_o)
    );

    task automatic clear_inputs;
    begin
        req_valid_i = 1'b0;
        logical_core_i = '0;
        logical_cluster_i = '0;
        core_available_i = '0;
        core_dead_suspect_i = '0;
        ready_queue_full_i = '0;
    end
    endtask

    task automatic expect_select(
        input logic expected_valid,
        input logic [CORE_ID_WIDTH-1:0] expected_core,
        input logic [CLUSTER_ID_WIDTH-1:0] expected_cluster
    );
    begin
        #1;
        if (select_valid_o !== expected_valid) begin
            $fatal(1, "select_valid mismatch: expected %0b got %0b",
                   expected_valid, select_valid_o);
        end

        if (physical_core_o !== expected_core) begin
            $fatal(1, "physical_core mismatch: expected %0d got %0d",
                   expected_core, physical_core_o);
        end

        if (physical_cluster_o !== expected_cluster) begin
            $fatal(1, "physical_cluster mismatch: expected %0d got %0d",
                   expected_cluster, physical_cluster_o);
        end
    end
    endtask

    initial begin
        clear_inputs();

        $display("Checking idle request");
        logical_core_i = 2'd1;
        logical_cluster_i = 1'd0;
        core_available_i[1][0] = 1'b1;
        expect_select(1'b0, 2'd1, 1'd0);

        $display("Checking logical core preference");
        req_valid_i = 1'b1;
        expect_select(1'b1, 2'd1, 1'd0);

        $display("Checking fallback to first available same-cluster core");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        logical_cluster_i = 1'd0;
        core_available_i[0][0] = 1'b0;
        core_available_i[1][0] = 1'b1;
        core_available_i[2][0] = 1'b1;
        expect_select(1'b1, 2'd1, 1'd0);

        $display("Checking fallback skips full ready queue");
        ready_queue_full_i[1][0] = 1'b1;
        expect_select(1'b1, 2'd2, 1'd0);

        $display("Checking fallback skips dead-suspect core");
        ready_queue_full_i[1][0] = 1'b0;
        core_dead_suspect_i[1][0] = 1'b1;
        expect_select(1'b1, 2'd2, 1'd0);

        $display("Checking no available core");
        core_available_i = '0;
        expect_select(1'b0, 2'd0, 1'd0);

        $display("Checking out-of-range logical core");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd3;
        logical_cluster_i = 1'd0;
        core_available_i = '1;
        expect_select(1'b0, 2'd3, 1'd0);

        $display("All core remap tests passed");
        $finish;
    end

endmodule
