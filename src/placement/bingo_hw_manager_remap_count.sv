// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Where the tasks of each logical slot sit when they are not in its own
// checkout queue. A checkout entry keeps the logical core and cluster of its
// descriptor, so an entry pushed into physical slot (p, pcl) that names
// another logical slot X (remapped, replayed or parked there) is counted for
// X until it leaves that queue again:
//   outstanding_o[X]       tasks of X in the checkout queue of any other slot
//   moved_in_o[p][pcl][X]  of those, the ones in the queue of slot (p, pcl)
//                          (ForeignStuckDrain only, else 0)
// The router holds a dummy-set of X back while X has outstanding tasks (its
// dep_set must fire after them, see bingo_hw_manager_router), and a stuck slot
// may only drain X's heads once every task of X elsewhere sits in that slot
// (see bingo_hw_manager_checkout_queue).
module bingo_hw_manager_remap_count #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CoreIdWidth = 2,
    parameter int unsigned ClusterIdWidth = 1,
    parameter int unsigned Width = 6,
    parameter bit          ForeignStuckDrain = 1'b1,
    parameter type         task_desc_t = logic,  // bingo_hw_manager_task_desc_t of the top
    parameter type         checkout_t = logic    // bingo_hw_manager/typedef.svh
) (
    input  logic             clk_i,
    input  logic             rst_ni,
    // Checkout queue of every physical slot: the entry pushed, the head popped
    input  checkout_t        checkout_i,
    output logic [Width-1:0] outstanding_o [NumCores][NumClusters],
    output logic [Width-1:0] moved_in_o [NumCores][NumClusters][NumCores][NumClusters]
);
    // A checkout entry of physical slot (p, cl) that belongs to another logical
    // slot of this chiplet (remapped or replayed; checkout entries keep the
    // logical core and cluster)
    function automatic logic foreign_entry(
        input task_desc_t entry, input int unsigned p, input int unsigned cl);
        return ((entry.assigned_core_id != CoreIdWidth'(p)) ||
                (entry.assigned_cluster_id != ClusterIdWidth'(cl))) &&
               (int'(entry.assigned_core_id) < NumCores) &&
               (int'(entry.assigned_cluster_id) < NumClusters);
    endfunction

    logic [Width-1:0] remap_outstanding_d [NumCores][NumClusters];
    logic [Width-1:0] remap_outstanding_q [NumCores][NumClusters];
    logic [Width-1:0] moved_in_d [NumCores][NumClusters][NumCores][NumClusters];
    logic [Width-1:0] moved_in_q [NumCores][NumClusters][NumCores][NumClusters];

    // Track tasks per logical core that sit in the checkout queue of another
    // physical core, in its own or another cluster.
    always_comb begin : update_remap_outstanding
        remap_outstanding_d = remap_outstanding_q;
        for (int unsigned p = 0; p < NumCores; p++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (checkout_i.push[p][cl] && foreign_entry(checkout_i.push_desc[p][cl], p, cl)) begin
                    remap_outstanding_d[checkout_i.push_desc[p][cl].assigned_core_id][checkout_i.push_desc[p][cl].assigned_cluster_id] =
                        remap_outstanding_d[checkout_i.push_desc[p][cl].assigned_core_id][checkout_i.push_desc[p][cl].assigned_cluster_id] + 1'b1;
                end
                if (checkout_i.pop[p][cl] && foreign_entry(checkout_i.head[p][cl], p, cl)) begin
                    remap_outstanding_d[checkout_i.head[p][cl].assigned_core_id][checkout_i.head[p][cl].assigned_cluster_id] =
                        remap_outstanding_d[checkout_i.head[p][cl].assigned_core_id][checkout_i.head[p][cl].assigned_cluster_id] - 1'b1;
                end
            end
        end
    end

    // Per (physical slot, logical core) split of the same counts
    if (ForeignStuckDrain) begin : gen_moved_in
        always_comb begin : update_moved_in
            moved_in_d = moved_in_q;
            for (int unsigned p = 0; p < NumCores; p++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    if (checkout_i.push[p][cl] && foreign_entry(checkout_i.push_desc[p][cl], p, cl)) begin
                        moved_in_d[p][cl][checkout_i.push_desc[p][cl].assigned_core_id][checkout_i.push_desc[p][cl].assigned_cluster_id] =
                            moved_in_d[p][cl][checkout_i.push_desc[p][cl].assigned_core_id][checkout_i.push_desc[p][cl].assigned_cluster_id] + 1'b1;
                    end
                    if (checkout_i.pop[p][cl] && foreign_entry(checkout_i.head[p][cl], p, cl)) begin
                        moved_in_d[p][cl][checkout_i.head[p][cl].assigned_core_id][checkout_i.head[p][cl].assigned_cluster_id] =
                            moved_in_d[p][cl][checkout_i.head[p][cl].assigned_core_id][checkout_i.head[p][cl].assigned_cluster_id] - 1'b1;
                    end
                end
            end
        end
        always_ff @(posedge clk_i or negedge rst_ni) begin : moved_in_regs
            if (!rst_ni) begin
                for (int unsigned p = 0; p < NumCores; p++)
                    for (int unsigned pcl = 0; pcl < NumClusters; pcl++)
                        for (int unsigned c = 0; c < NumCores; c++)
                            for (int unsigned cl = 0; cl < NumClusters; cl++)
                                moved_in_q[p][pcl][c][cl] <= '0;
            end else begin
                moved_in_q <= moved_in_d;
            end
        end
    end else begin : gen_no_moved_in
        always_comb begin
            for (int unsigned p = 0; p < NumCores; p++)
                for (int unsigned pcl = 0; pcl < NumClusters; pcl++)
                    for (int unsigned c = 0; c < NumCores; c++)
                        for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                            moved_in_d[p][pcl][c][cl] = '0;
                            moved_in_q[p][pcl][c][cl] = '0;
                        end
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin : remap_outstanding_regs
        if (!rst_ni) begin
            for (int unsigned c = 0; c < NumCores; c++) begin
                for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                    remap_outstanding_q[c][cl] <= '0;
                end
            end
        end else begin
            remap_outstanding_q <= remap_outstanding_d;
        end
    end

    assign outstanding_o = remap_outstanding_q;
    assign moved_in_o    = moved_in_q;
endmodule
