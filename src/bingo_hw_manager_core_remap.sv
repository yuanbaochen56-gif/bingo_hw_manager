module bingo_hw_manager_core_remap #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1
) (
    input  logic req_valid_i,
    // Inputs from the core selector about the requested logical core/cluster
    input  logic [CoreIdWidth-1:0] logical_core_i,
    input  logic [ClusterIdWidth-1:0] logical_cluster_i,
    // Inputs from the watchdog about core status
    input  logic [NumCores-1:0][NumClusters-1:0] core_available_i,
    input  logic [NumCores-1:0][NumClusters-1:0] core_dead_suspect_i,
    input  logic [NumCores-1:0][NumClusters-1:0] ready_queue_full_i,
    // Outputs to the core selector
    output logic select_valid_o,
    output logic [CoreIdWidth-1:0] physical_core_o,
    output logic [ClusterIdWidth-1:0] physical_cluster_o
);
    logic logical_core_in_range;
    logic logical_cluster_in_range;

    assign logical_core_in_range = (int'(logical_core_i) < NumCores);
    assign logical_cluster_in_range = (int'(logical_cluster_i) < NumClusters);

    always_comb begin
        select_valid_o = 1'b0;
        physical_core_o = logical_core_i;
        physical_cluster_o = logical_cluster_i;

        if (req_valid_i && logical_core_in_range && logical_cluster_in_range) begin
            // Prefer the requested logical core when it can accept work.
            if (core_available_i[logical_core_i][logical_cluster_i] &&
                !core_dead_suspect_i[logical_core_i][logical_cluster_i] &&
                !ready_queue_full_i[logical_core_i][logical_cluster_i]) begin

                select_valid_o = 1'b1;
                physical_core_o = logical_core_i;
                physical_cluster_o = logical_cluster_i;

            end else begin
                // Fall back to the first available core in the same cluster.
                for (int unsigned c = 0; c < NumCores; c++) begin
                    if (!select_valid_o &&
                        core_available_i[c][logical_cluster_i] &&
                        !core_dead_suspect_i[c][logical_cluster_i] &&
                        !ready_queue_full_i[c][logical_cluster_i]) begin

                        select_valid_o = 1'b1;
                        physical_core_o = CoreIdWidth'(c);//this is the new core to be selected
                        physical_cluster_o = logical_cluster_i;//this is the same cluster as the requested core
                    end
                end
            end
        end
    end

endmodule
