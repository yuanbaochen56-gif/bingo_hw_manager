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
// - Once retired, an executing task (remappable_i) goes to the substitute of
//   bingo_hw_manager_substitute_sel: a live core with the same non-zero
//   CoreTypeId as the logical core, in the logical cluster if there is one
//   (level 1), else in another cluster (level 2, if SubstituteLevelMask[1]).
//   The choice only depends on the set of fenced cores, so consecutive tasks
//   of one dead core land on the same substitute, in order. Without such a
//   substitute the task is held, unless level 3 (SubstituteLevelMask[2]) is
//   enabled and the core has a non-zero type that the transport can export
//   (remote_type_en_i): then it stays on its (retired) logical slot, which acts
//   as the proxy of a remote chiplet, and remote_o tells the top to export it
//   instead of pushing it to the ready queue.
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
    // Type of each (core, cluster) slot (see bingo_hw_manager_top): a core may
    // run the tasks of another core of the chiplet iff both have the same
    // non-zero type.
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    // Substitute levels (see bingo_hw_manager_top SubstituteLevelMask)
    parameter logic [2:0] SubstituteLevelMask = 3'b001
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
    // Level 3: core types with an export target (index: CoreTypeId)
    input  logic [2**CoreTypeIdWidth-1:0] remote_type_en_i,
    // Selected physical core/cluster
    output logic select_valid_o,
    output logic [CoreIdWidth-1:0] physical_core_o,
    output logic [ClusterIdWidth-1:0] physical_cluster_o,
    // Level 3: the selected slot is the logical one, used as a remote proxy
    output logic remote_o
);
    logic logical_in_range;
    logic logical_fenced;
    logic logical_retired;
    logic                      sub_found;
    logic [CoreIdWidth-1:0]    sub_core;
    logic [ClusterIdWidth-1:0] sub_cluster;
    logic                      logical_typed;
    logic                      logical_exportable;

    assign logical_in_range = (int'(logical_core_i) < NumCores) &&
                              (int'(logical_cluster_i) < NumClusters);
    assign logical_fenced   = logical_in_range && core_fenced_i[logical_core_i][logical_cluster_i];
    assign logical_retired  = logical_in_range && core_retired_i[logical_core_i][logical_cluster_i];
    assign logical_typed    = logical_in_range && (CoreTypeId[logical_core_i][logical_cluster_i] != '0);
    assign logical_exportable = logical_typed && remote_type_en_i[CoreTypeId[logical_core_i][logical_cluster_i]];

    // The logical core is fenced whenever its substitute is used, so the
    // selector never returns the logical core itself here.
    bingo_hw_manager_substitute_sel #(
        .NumCores(NumCores),
        .NumClusters(NumClusters),
        .CoreIdWidth(CoreIdWidth),
        .ClusterIdWidth(ClusterIdWidth),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreTypeId(CoreTypeId),
        .LevelMask(SubstituteLevelMask)
    ) i_substitute_sel (
        .logical_core_i(logical_core_i),
        .logical_cluster_i(logical_cluster_i),
        .fenced_i(core_fenced_i),
        .found_o(sub_found),
        .core_o(sub_core),
        .cluster_o(sub_cluster)
    );

    always_comb begin
        select_valid_o     = req_valid_i && logical_in_range;
        physical_core_o    = logical_core_i;
        physical_cluster_o = logical_cluster_i;
        remote_o           = 1'b0;

        if (req_valid_i && logical_in_range && logical_fenced) begin
            if (!logical_retired) begin
                // Outstanding tasks of this core are still being replayed.
                select_valid_o = 1'b0;
            end else if (remappable_i) begin
                select_valid_o = sub_found;
                if (sub_found) begin
                    physical_core_o    = sub_core;
                    physical_cluster_o = sub_cluster;
                end else if (SubstituteLevelMask[2] && logical_exportable) begin
                    select_valid_o = 1'b1;
                    remote_o       = 1'b1;
                end
            end
        end
    end

endmodule
