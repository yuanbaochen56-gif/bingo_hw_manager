// Core remap selector.
//
// Picks the physical core that receives a task assigned to logical core
// `logical_core_i` in cluster `logical_cluster_i`.
//
// Semantics (remap only on a dead core):
// - A healthy logical core always keeps its own tasks. Being busy, or not
//   blocked in a ready-queue read right now, is NOT a reason to move work: the
//   compiler relies on per-core in-order execution (same-core HOL), and a
//   dummy-set task only waits for its source because both sit in the same
//   core's checkout FIFO.
// - Only when the logical core is dead_suspect and the task really executes
//   on a core (remappable_i), the task goes to the lowest-indexed core of the
//   same cluster that is not dead_suspect and is allowed by AllowMask. The
//   choice only depends on the set of dead cores, so consecutive tasks of one
//   dead core all land on the same substitute, in order.
// - Without an allowed substitute the task stays on its logical core.
// - Back-pressure (full ready/checkout queue) is left to the downstream
//   handshake, so the selection is valid whenever a request is present.
module bingo_hw_manager_core_remap #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // AllowMask[logical][physical] = 1: `physical` may run the tasks of
    // `logical` while `logical` is dead_suspect. Only set it for cores that can
    // run the same kernels ('0 disables remapping, e.g. heterogeneous clusters).
    parameter logic [NumCores-1:0][NumCores-1:0] AllowMask = '1
) (
    input  logic req_valid_i,
    // Requested logical core/cluster (from the task descriptor)
    input  logic [CoreIdWidth-1:0] logical_core_i,
    input  logic [ClusterIdWidth-1:0] logical_cluster_i,
    // 1 if the task executes on a core (normal / gating task). Dummy-set and
    // CERF-skipped tasks never leave their logical core.
    input  logic remappable_i,
    // Core status from the watchdog
    input  logic [NumCores-1:0][NumClusters-1:0] core_dead_suspect_i,
    // Selected physical core/cluster
    output logic select_valid_o,
    output logic [CoreIdWidth-1:0] physical_core_o,
    output logic [ClusterIdWidth-1:0] physical_cluster_o
);
    logic logical_in_range;

    assign logical_in_range = (int'(logical_core_i) < NumCores) &&
                              (int'(logical_cluster_i) < NumClusters);

    always_comb begin
        select_valid_o     = req_valid_i && logical_in_range;
        physical_core_o    = logical_core_i;
        physical_cluster_o = logical_cluster_i;

        if (req_valid_i && logical_in_range && remappable_i &&
            core_dead_suspect_i[logical_core_i][logical_cluster_i]) begin
            // Scan downwards so that the lowest-indexed candidate wins.
            for (int c = NumCores - 1; c >= 0; c--) begin
                if (AllowMask[logical_core_i][c] &&
                    (c != int'(logical_core_i)) &&
                    !core_dead_suspect_i[c][logical_cluster_i]) begin
                    physical_core_o = CoreIdWidth'(c);
                end
            end
        end
    end

endmodule
