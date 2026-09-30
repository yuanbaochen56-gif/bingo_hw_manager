// =============================================================================
// Level 3 over the real transport: random graphs, random fault
// =============================================================================
// Two chiplets, 3 cores of distinct types (no local substitute), exports
// through remote_link with random stalls. Chiplet c gets N_TASKS[c] normal
// tasks on random cores (ids 1.. on chiplet 0, 101.. on chiplet 1), each may
// depend on one earlier task of its chiplet (unique tag per dep-matrix cell,
// at most one consumer per task). One random task of chiplet 0 gets a random
// fault (hang / zombie / slow) on its core: the tasks of a fenced core run on
// chiplet 1. The harness checks retire order, done order per proxy slot and
// the link errors.
// EXPECTED: all tasks complete.

localparam int unsigned N_TASKS_0               = 24;
localparam int unsigned N_TASKS_1               = 12;
localparam int unsigned EXPECTED_TASK_COUNT     = N_TASKS_0 + N_TASKS_1;
localparam int unsigned DEADLOCK_THRESHOLD      = 20000;
localparam int unsigned DEP_MATRIX_LOG_INTERVAL = 0;

bingo_hw_manager_task_desc_full_t rr_tasks [2][N_TASKS_0 + 1];
int unsigned                      rr_core_of [2][N_TASKS_0 + 1];

task automatic rr_graph(input int chip, input int unsigned n, input int unsigned id_base);
    automatic bit          has_cons  [N_TASKS_0 + 1];
    automatic int unsigned cell_tags [NUM_CORES_PER_CLUSTER][NUM_CORES_PER_CLUSTER];
    automatic int unsigned n_edges = 0;
    for (int r = 0; r < NUM_CORES_PER_CLUSTER; r++)
        for (int c = 0; c < NUM_CORES_PER_CLUSTER; c++) cell_tags[r][c] = 0;
    for (int unsigned i = 1; i <= n; i++) begin
        rr_core_of[chip][i] = $urandom_range(0, NUM_CORES_PER_CLUSTER - 1);
        has_cons[i] = 1'b0;
        rr_tasks[chip][i] = pack_normal_task(2'b00, bingo_hw_manager_task_id_t'(id_base + i),
                                             bingo_hw_manager_assigned_chiplet_id_t'(chip), 0,
                                             bingo_hw_manager_assigned_core_id_t'(rr_core_of[chip][i]),
                                             1'b0, '0, 1'b0, 1'b0, bingo_hw_manager_assigned_chiplet_id_t'(chip), 0, '0);
    end
    for (int unsigned i = 2; i <= n; i++) begin
        if ($urandom_range(0, 99) < 70) begin
            automatic int unsigned j   = $urandom_range(1, i - 1);
            automatic int unsigned row = rr_core_of[chip][i];
            automatic int unsigned col = rr_core_of[chip][j];
            if (!has_cons[j] && (cell_tags[row][col] < (1 << DEP_TAG_WIDTH))) begin
                automatic bingo_hw_manager_dep_tag_t tag = bingo_hw_manager_dep_tag_t'(cell_tags[row][col]);
                cell_tags[row][col]++;
                has_cons[j] = 1'b1;
                n_edges++;
                rr_tasks[chip][j].dep_set_info.dep_set_en      = 1'b1;
                rr_tasks[chip][j].dep_set_info.dep_set_code    = bingo_hw_manager_dep_code_t'(1 << row);
                rr_tasks[chip][j].dep_set_info.dep_set_tag     = tag;
                rr_tasks[chip][i].dep_check_info.dep_check_en   = 1'b1;
                rr_tasks[chip][i].dep_check_info.dep_check_code = bingo_hw_manager_dep_code_t'(1 << col);
                rr_tasks[chip][i].dep_check_info.dep_check_tag  = tag;
            end
        end
    end
    $display("[RANDOM] chiplet %0d: %0d tasks, %0d edges", chip, n, n_edges);
endtask

initial begin : random_graph
    rr_graph(0, N_TASKS_0, 0);
    rr_graph(1, N_TASKS_1, 100);
    fault_task_id = $urandom_range(1, N_TASKS_0);
    fault_core    = rr_core_of[0][fault_task_id];   // flat id on chiplet 0
    fault_mode    = $urandom_range(0, 2);
    $display("[RANDOM] %0d tasks; fault mode %0d on task %0d (chiplet 0 core %0d)",
             EXPECTED_TASK_COUNT, fault_mode, fault_task_id, fault_core);
end

for (genvar gc = 0; gc < 2; gc++) begin : gen_rr_push
    initial begin
        automatic axi_pkg::resp_t resp;
        wait (rst_ni);
        repeat (20) @(posedge clk_i);
        for (int unsigned i = 1; i <= ((gc == 0) ? N_TASKS_0 : N_TASKS_1); i++) begin
            task_queue_master[gc].write(task_queue_base[gc], '0, rr_tasks[gc][i], '1, resp);
        end
    end
end

final begin
    if (fault_mode == FAULT_SLOW) begin
        if (fenced_export[0] !== '0) $error("[RANDOM] slow core must not be fenced");
        if (remote_export_count[0] != 0) $error("[RANDOM] slow core: no export expected, got %0d", remote_export_count[0]);
    end else begin
        if (retired_export[0][fault_core][0] !== 1'b1) $error("[RANDOM] faulty core %0d must be retired", fault_core);
        if (remote_export_count[0] == 0) $error("[RANDOM] the faulty task must be exported");
    end
    if (remote_done_in_count[0] != remote_export_count[0]) begin
        $error("[RANDOM] %0d exports, %0d dones back", remote_export_count[0], remote_done_in_count[0]);
    end
    if (fenced_export[1] !== '0) $error("[RANDOM] chiplet 1 must not fence a core");
    $display("[RANDOM] exported %0d, credit stall %0d cycles, max outstanding %0d",
             remote_export_count[0], remote_credit_stall[0], remote_max_outstanding[0]);
end
