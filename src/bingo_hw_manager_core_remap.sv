// Core remap selector.
//
// Picks the physical core that receives a task assigned to logical core
// `logical_core_i` in cluster `logical_cluster_i`.
//
// Semantics (remap only after a dead core's outstanding tasks were replayed):
// - A logical core that is not fenced always keeps its own tasks. Being busy,
//   slow or dead_suspect is NOT a reason to move work: the compiler relies on
//   per-core in-order execution (same-core HOL), and a dummy-set task only
//   waits for its source because both sit in the same core's checkout FIFO.
// - A fenced (confirmed dead) logical core that is not retired yet still has
//   outstanding tasks waiting to be replayed: its new tasks are held
//   (select_valid_o = 0) so they cannot overtake the replayed ones.
// - Once retired, an executing task (remappable_i) goes to the lowest-indexed
//   core of the same cluster that is not fenced and is allowed by AllowMask.
//   The choice only depends on the set of fenced cores, so consecutive tasks of
//   one dead core land on the same substitute, in order. Without an allowed
//   substitute the task is held.
// - Dummy-set and CERF-skipped tasks (remappable_i = 0) never execute on a
//   core; they stay on their logical core even when it is retired (its checkout
//   FIFO still retires them).
// - Back-pressure (full ready/checkout queue) is left to the downstream
//   handshake, so the selection is valid whenever a request is present.
module bingo_hw_manager_core_remap #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    // AllowMask[logical][physical] = 1: `physical` may run the tasks of
    // `logical` once `logical` is retired. Only set it for cores that can run
    // the same kernels ('0 disables remapping, e.g. heterogeneous clusters).
    parameter logic [NumCores-1:0][NumCores-1:0] AllowMask = '1
) (
    input  logic req_valid_i,
    // Requested logical core/cluster (from the task descriptor)
    input  logic [CoreIdWidth-1:0] logical_core_i,
    input  logic [ClusterIdWidth-1:0] logical_cluster_i,
    // 1 if the task executes on a core (normal / gating task). Dummy-set and
    // CERF-skipped tasks never leave their logical core.
    input  logic remappable_i,
    // Core status from the watchdog / replay controller
    input  logic [NumCores-1:0][NumClusters-1:0] core_fenced_i,
    input  logic [NumCores-1:0][NumClusters-1:0] core_retired_i,
    // Selected physical core/cluster
    output logic select_valid_o,
    output logic [CoreIdWidth-1:0] physical_core_o,
    output logic [ClusterIdWidth-1:0] physical_cluster_o
);
    logic logical_in_range;
    logic logical_fenced;
    logic logical_retired;

    assign logical_in_range = (int'(logical_core_i) < NumCores) &&
                              (int'(logical_cluster_i) < NumClusters);
    assign logical_fenced   = logical_in_range && core_fenced_i[logical_core_i][logical_cluster_i];
    assign logical_retired  = logical_in_range && core_retired_i[logical_core_i][logical_cluster_i];

    always_comb begin
        select_valid_o     = req_valid_i && logical_in_range;
        physical_core_o    = logical_core_i;
        physical_cluster_o = logical_cluster_i;

        if (req_valid_i && logical_in_range && logical_fenced) begin
            if (!logical_retired) begin
                // Outstanding tasks of this core are still being replayed.
                select_valid_o = 1'b0;
            end else if (remappable_i) begin
                select_valid_o = 1'b0;
                // Scan downwards so that the lowest-indexed candidate wins.
                for (int c = NumCores - 1; c >= 0; c--) begin
                    if (AllowMask[logical_core_i][c] &&
                        (c != int'(logical_core_i)) &&
                        !core_fenced_i[c][logical_cluster_i]) begin
                        select_valid_o  = 1'b1;
                        physical_core_o = CoreIdWidth'(c);
                    end
                end
            end
        end
    end

endmodule
