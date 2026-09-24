`timescale 1ns/1ps

// Unit test of bingo_hw_manager_core_remap (remap only on a dead core).
module tb_bingo_hw_manager_core_remap;

    localparam int unsigned NUM_CORES = 3;
    localparam int unsigned NUM_CLUSTERS = 2;
    localparam int unsigned CORE_ID_WIDTH = 2;
    localparam int unsigned CLUSTER_ID_WIDTH = 1;
    // Second instance: logical core 0 may only use core 2, core 2 may not be
    // replaced at all ([logical][physical]).
    localparam logic [NUM_CORES-1:0][NUM_CORES-1:0] RESTRICTED_MASK = '{
        3'b000,  // logical 2: no substitute
        3'b111,  // logical 1: any core
        3'b100   // logical 0: only core 2
    };

    logic req_valid_i;
    logic [CORE_ID_WIDTH-1:0] logical_core_i;
    logic [CLUSTER_ID_WIDTH-1:0] logical_cluster_i;
    logic remappable_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_dead_suspect_i;

    logic select_valid_o;
    logic [CORE_ID_WIDTH-1:0] physical_core_o;
    logic [CLUSTER_ID_WIDTH-1:0] physical_cluster_o;

    logic select_valid_r;
    logic [CORE_ID_WIDTH-1:0] physical_core_r;
    logic [CLUSTER_ID_WIDTH-1:0] physical_cluster_r;

    bingo_hw_manager_core_remap #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CoreIdWidth(CORE_ID_WIDTH),
        .ClusterIdWidth(CLUSTER_ID_WIDTH)
    ) dut (
        .req_valid_i(req_valid_i),
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .remappable_i(remappable_i),
        .core_dead_suspect_i(core_dead_suspect_i),
        .select_valid_o(select_valid_o),
        .physical_core_o(physical_core_o),
        .physical_cluster_o(physical_cluster_o)
    );

    bingo_hw_manager_core_remap #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CoreIdWidth(CORE_ID_WIDTH),
        .ClusterIdWidth(CLUSTER_ID_WIDTH),
        .AllowMask(RESTRICTED_MASK)
    ) dut_restricted (
        .req_valid_i(req_valid_i),
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .remappable_i(remappable_i),
        .core_dead_suspect_i(core_dead_suspect_i),
        .select_valid_o(select_valid_r),
        .physical_core_o(physical_core_r),
        .physical_cluster_o(physical_cluster_r)
    );

    task automatic clear_inputs;
    begin
        req_valid_i = 1'b0;
        logical_core_i = '0;
        logical_cluster_i = '0;
        remappable_i = 1'b1;
        core_dead_suspect_i = '0;
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

    task automatic expect_restricted_core(input logic [CORE_ID_WIDTH-1:0] expected_core);
    begin
        #1;
        if (physical_core_r !== expected_core) begin
            $fatal(1, "restricted mask: physical_core mismatch: expected %0d got %0d",
                   expected_core, physical_core_r);
        end
    end
    endtask

    initial begin
        clear_inputs();

        $display("Checking idle request");
        logical_core_i = 2'd1;
        expect_select(1'b0, 2'd1, 1'd0);

        $display("Checking healthy logical core keeps its task");
        req_valid_i = 1'b1;
        expect_select(1'b1, 2'd1, 1'd0);

        $display("Checking a dead core in another cluster does not matter");
        core_dead_suspect_i[1][1] = 1'b1;
        expect_select(1'b1, 2'd1, 1'd0);

        $display("Checking dead logical core remaps to the lowest alive core of its cluster");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        core_dead_suspect_i[0][0] = 1'b1;
        expect_select(1'b1, 2'd1, 1'd0);

        $display("Checking dead candidates are skipped");
        core_dead_suspect_i[1][0] = 1'b1;
        expect_select(1'b1, 2'd2, 1'd0);

        $display("Checking no alive substitute keeps the logical core");
        core_dead_suspect_i[2][0] = 1'b1;
        expect_select(1'b1, 2'd0, 1'd0);

        $display("Checking remap stays inside the requested cluster");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd2;
        logical_cluster_i = 1'd1;
        core_dead_suspect_i[2][1] = 1'b1;
        core_dead_suspect_i[0][1] = 1'b1;
        expect_select(1'b1, 2'd1, 1'd1);

        $display("Checking non-remappable (dummy-set / skipped) task stays on its dead core");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        remappable_i = 1'b0;
        core_dead_suspect_i[0][0] = 1'b1;
        expect_select(1'b1, 2'd0, 1'd0);

        $display("Checking AllowMask restricts the substitute");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        core_dead_suspect_i[0][0] = 1'b1;
        expect_restricted_core(2'd2);          // core 1 alive but not allowed
        core_dead_suspect_i[2][0] = 1'b1;
        expect_restricted_core(2'd0);          // only allowed core dead -> stay
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd2;
        core_dead_suspect_i[2][0] = 1'b1;
        expect_restricted_core(2'd2);          // logical 2 has no substitute
        logical_core_i = 2'd1;
        core_dead_suspect_i[1][0] = 1'b1;
        expect_restricted_core(2'd0);          // logical 1: lowest alive core

        $display("Checking out-of-range logical core");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd3;
        expect_select(1'b0, 2'd3, 1'd0);

        $display("All core remap tests passed");
        $finish;
    end

endmodule
