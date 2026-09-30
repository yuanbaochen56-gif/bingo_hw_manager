`timescale 1ns/1ps

// Unit test of bingo_hw_manager_core_remap (remap only once a fenced core is retired).
module tb_bingo_hw_manager_core_remap;

    localparam int unsigned NUM_CORES = 3;
    localparam int unsigned NUM_CLUSTERS = 2;
    localparam int unsigned CORE_ID_WIDTH = 2;
    localparam int unsigned CLUSTER_ID_WIDTH = 1;
    // Second instance, CoreTypeId[core][cluster] differs per cluster:
    //   cluster 0: core 0 type 1, core 1 type 2, core 2 type 1
    //   cluster 1: core 0 type 3, core 1 type 3, core 2 type 0 (never substituted)
    localparam logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0][3:0] RESTRICTED_TYPES = '{
        '{4'd0, 4'd1},  // core 2: cluster 1, cluster 0
        '{4'd3, 4'd2},  // core 1
        '{4'd3, 4'd1}   // core 0
    };
    // Third instance, types shared across the clusters:
    //   cluster 0: core 0 type 1, core 1 type 2, core 2 type 2
    //   cluster 1: core 0 type 2, core 1 type 1, core 2 type 4 (unique)
    localparam logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0][3:0] CROSS_TYPES = '{
        '{4'd4, 4'd2},  // core 2: cluster 1, cluster 0
        '{4'd1, 4'd2},  // core 1
        '{4'd2, 4'd1}   // core 0
    };

    logic req_valid_i;
    logic [CORE_ID_WIDTH-1:0] logical_core_i;
    logic [CLUSTER_ID_WIDTH-1:0] logical_cluster_i;
    logic remappable_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_fenced_i;
    logic [NUM_CORES-1:0][NUM_CLUSTERS-1:0] core_retired_i;

    logic select_valid_o;
    logic [CORE_ID_WIDTH-1:0] physical_core_o;
    logic [CLUSTER_ID_WIDTH-1:0] physical_cluster_o;

    logic select_valid_r;
    logic [CORE_ID_WIDTH-1:0] physical_core_r;
    logic [CLUSTER_ID_WIDTH-1:0] physical_cluster_r;

    logic select_valid_x;
    logic [CORE_ID_WIDTH-1:0] physical_core_x;
    logic [CLUSTER_ID_WIDTH-1:0] physical_cluster_x;

    logic select_valid_l1;
    logic [CORE_ID_WIDTH-1:0] physical_core_l1;
    logic [CLUSTER_ID_WIDTH-1:0] physical_cluster_l1;

    // Levels 1 and 2 (substitutes in other clusters allowed)
    bingo_hw_manager_core_remap #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CoreIdWidth(CORE_ID_WIDTH),
        .ClusterIdWidth(CLUSTER_ID_WIDTH),
        .SubstituteLevelMask(3'b011)
    ) dut (
        .req_valid_i(req_valid_i),
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .remappable_i(remappable_i),
        .core_fenced_i(core_fenced_i),
        .core_retired_i(core_retired_i),
        .select_valid_o(select_valid_o),
        .physical_core_o(physical_core_o),
        .physical_cluster_o(physical_cluster_o)
    );

    bingo_hw_manager_core_remap #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CoreIdWidth(CORE_ID_WIDTH),
        .ClusterIdWidth(CLUSTER_ID_WIDTH),
        .CoreTypeIdWidth(4),
        .CoreTypeId(RESTRICTED_TYPES)
    ) dut_restricted (
        .req_valid_i(req_valid_i),
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .remappable_i(remappable_i),
        .core_fenced_i(core_fenced_i),
        .core_retired_i(core_retired_i),
        .select_valid_o(select_valid_r),
        .physical_core_o(physical_core_r),
        .physical_cluster_o(physical_cluster_r)
    );

    bingo_hw_manager_core_remap #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CoreIdWidth(CORE_ID_WIDTH),
        .ClusterIdWidth(CLUSTER_ID_WIDTH),
        .CoreTypeIdWidth(4),
        .CoreTypeId(CROSS_TYPES),
        .SubstituteLevelMask(3'b011)
    ) dut_cross (
        .req_valid_i(req_valid_i),
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .remappable_i(remappable_i),
        .core_fenced_i(core_fenced_i),
        .core_retired_i(core_retired_i),
        .select_valid_o(select_valid_x),
        .physical_core_o(physical_core_x),
        .physical_cluster_o(physical_cluster_x)
    );

    // Same types, level 1 only (default): never leaves the cluster
    bingo_hw_manager_core_remap #(
        .NumCores(NUM_CORES),
        .NumClusters(NUM_CLUSTERS),
        .CoreIdWidth(CORE_ID_WIDTH),
        .ClusterIdWidth(CLUSTER_ID_WIDTH),
        .CoreTypeIdWidth(4),
        .CoreTypeId(CROSS_TYPES)
    ) dut_l1 (
        .req_valid_i(req_valid_i),
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .remappable_i(remappable_i),
        .core_fenced_i(core_fenced_i),
        .core_retired_i(core_retired_i),
        .select_valid_o(select_valid_l1),
        .physical_core_o(physical_core_l1),
        .physical_cluster_o(physical_cluster_l1)
    );

    task automatic expect_l1(
        input logic expected_valid,
        input logic [CORE_ID_WIDTH-1:0] expected_core
    );
    begin
        #1;
        if (select_valid_l1 !== expected_valid) begin
            $fatal(1, "level 1 only: select_valid mismatch: expected %0b got %0b",
                   expected_valid, select_valid_l1);
        end
        if (expected_valid && ((physical_core_l1 !== expected_core) ||
                               (physical_cluster_l1 !== logical_cluster_i))) begin
            $fatal(1, "level 1 only: expected core %0d cluster %0d, got core %0d cluster %0d",
                   expected_core, logical_cluster_i, physical_core_l1, physical_cluster_l1);
        end
    end
    endtask

    task automatic clear_inputs;
    begin
        req_valid_i = 1'b0;
        logical_core_i = '0;
        logical_cluster_i = '0;
        remappable_i = 1'b1;
        core_fenced_i = '0;
        core_retired_i = '0;
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

    task automatic expect_restricted(
        input logic expected_valid,
        input logic [CORE_ID_WIDTH-1:0] expected_core
    );
    begin
        #1;
        if (select_valid_r !== expected_valid) begin
            $fatal(1, "restricted types: select_valid mismatch: expected %0b got %0b",
                   expected_valid, select_valid_r);
        end
        if (expected_valid && (physical_core_r !== expected_core)) begin
            $fatal(1, "restricted types: physical_core mismatch: expected %0d got %0d",
                   expected_core, physical_core_r);
        end
        if (expected_valid && (physical_cluster_r !== logical_cluster_i)) begin
            $fatal(1, "restricted types: physical_cluster mismatch: expected %0d got %0d",
                   logical_cluster_i, physical_cluster_r);
        end
    end
    endtask

    task automatic expect_cross(
        input logic expected_valid,
        input logic [CORE_ID_WIDTH-1:0] expected_core,
        input logic [CLUSTER_ID_WIDTH-1:0] expected_cluster
    );
    begin
        #1;
        if (select_valid_x !== expected_valid) begin
            $fatal(1, "cross-cluster types: select_valid mismatch: expected %0b got %0b",
                   expected_valid, select_valid_x);
        end
        if (expected_valid && ((physical_core_x !== expected_core) ||
                               (physical_cluster_x !== expected_cluster))) begin
            $fatal(1, "cross-cluster types: expected core %0d cluster %0d, got core %0d cluster %0d",
                   expected_core, expected_cluster, physical_core_x, physical_cluster_x);
        end
    end
    endtask

    // Fenced and retired (outstanding tasks already replayed)
    task automatic retire(input int unsigned core, input int unsigned cluster);
    begin
        core_fenced_i[core][cluster]  = 1'b1;
        core_retired_i[core][cluster] = 1'b1;
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

        $display("Checking a retired core in another cluster does not matter");
        retire(1, 1);
        expect_select(1'b1, 2'd1, 1'd0);

        $display("Checking a fenced core whose tasks are still being replayed is held");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        core_fenced_i[0][0] = 1'b1;
        expect_select(1'b0, 2'd0, 1'd0);

        $display("Checking retired logical core remaps to the lowest live core of its cluster");
        retire(0, 0);
        expect_select(1'b1, 2'd1, 1'd0);

        $display("Checking fenced candidates are skipped");
        core_fenced_i[1][0] = 1'b1;
        expect_select(1'b1, 2'd2, 1'd0);

        $display("Checking a cluster without live substitute hands over to another cluster");
        core_fenced_i[2][0] = 1'b1;
        expect_select(1'b1, 2'd0, 1'd1);

        $display("Checking no live substitute in the chiplet holds the task");
        core_fenced_i[0][1] = 1'b1;
        core_fenced_i[1][1] = 1'b1;
        core_fenced_i[2][1] = 1'b1;
        expect_select(1'b0, 2'd0, 1'd0);

        $display("Checking remap prefers the requested cluster over a lower one");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd2;
        logical_cluster_i = 1'd1;
        retire(2, 1);
        core_fenced_i[0][1] = 1'b1;
        expect_select(1'b1, 2'd1, 1'd1);

        $display("Checking non-remappable (dummy-set / skipped) task");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        remappable_i = 1'b0;
        core_fenced_i[0][0] = 1'b1;
        expect_select(1'b0, 2'd0, 1'd0);      // held while the core is replayed
        core_retired_i[0][0] = 1'b1;
        expect_select(1'b1, 2'd0, 1'd0);      // then stays on its retired core

        $display("Checking CoreTypeId restricts the substitute, per cluster");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        retire(0, 0);
        expect_restricted(1'b1, 2'd2);        // core 1 live but of another type
        core_fenced_i[2][0] = 1'b1;
        expect_restricted(1'b0, 2'd0);        // only same-type core fenced -> hold
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd1;
        retire(1, 0);
        expect_restricted(1'b0, 2'd1);        // type 2 appears once in cluster 0
        clear_inputs();
        req_valid_i = 1'b1;
        logical_cluster_i = 1'd1;
        logical_core_i = 2'd2;
        retire(2, 1);
        expect_restricted(1'b0, 2'd2);        // type 0 never hands over its tasks
        logical_core_i = 2'd1;
        retire(1, 1);
        expect_restricted(1'b1, 2'd0);        // cluster 1: core 0 has type 3 too
        clear_inputs();
        req_valid_i = 1'b1;
        logical_cluster_i = 1'd1;
        logical_core_i = 2'd0;
        retire(0, 1);
        core_fenced_i[1][0] = 1'b1;           // other cluster does not matter
        expect_restricted(1'b1, 2'd1);

        $display("Checking CoreTypeId shared across clusters");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        retire(0, 0);
        expect_cross(1'b1, 2'd1, 1'd1);       // type 1: only core 1 of cluster 1
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd1;
        retire(1, 0);
        expect_cross(1'b1, 2'd2, 1'd0);       // type 2: own cluster before core 0 of cluster 1
        core_fenced_i[2][0] = 1'b1;
        expect_cross(1'b1, 2'd0, 1'd1);       // then the other cluster
        core_fenced_i[0][1] = 1'b1;
        expect_cross(1'b0, 2'd0, 1'd0);       // every type-2 core fenced: hold
        clear_inputs();
        req_valid_i = 1'b1;
        logical_cluster_i = 1'd1;
        logical_core_i = 2'd0;
        retire(0, 1);
        expect_cross(1'b1, 2'd1, 1'd0);       // type 2 of cluster 1: lowest of cluster 0
        logical_core_i = 2'd2;
        retire(2, 1);
        expect_cross(1'b0, 2'd0, 1'd0);       // type 4 exists once in the chiplet

        $display("Checking level 1 only keeps the substitute in the cluster");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd0;
        retire(0, 0);
        expect_l1(1'b0, 2'd0);                // type 1 only in cluster 1: hold
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd1;
        retire(1, 0);
        expect_l1(1'b1, 2'd2);                // type 2: core 2 of cluster 0
        core_fenced_i[2][0] = 1'b1;
        expect_l1(1'b0, 2'd0);                // not core 0 of cluster 1
        expect_cross(1'b1, 2'd0, 1'd1);       // which level 2 would take

        $display("Checking out-of-range logical core");
        clear_inputs();
        req_valid_i = 1'b1;
        logical_core_i = 2'd3;
        expect_select(1'b0, 2'd3, 1'd0);

        $display("All core remap tests passed");
        $finish;
    end

endmodule
