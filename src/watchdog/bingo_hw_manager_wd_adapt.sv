// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Learned watchdog thresholds. For every (core type, task class) the module
// learns how long a healthy execution runs without progress, and gives every
// slot thresholds derived from it, which can only be shorter than the slot's
// static ones (h_eff_i, c_eff_i: those of its core type, or the parameters).
// Times are ticks of the watchdog timer (bingo_hw_manager_watchdog), so they
// count cycles of the normal clock.
//
// Table table_q[type][class], type = CoreTypeId of the slot, class = class of
// the task it runs (class 0 is never learned and gets no threshold):
//   maximum  M  the longest progress interval of a learned execution
//   samples  N  learned executions, saturating at 15
//   load     L  the highest load during a learned execution (load: monitored,
//               unfenced, busy slots)
// An execution is learned in the cycle of its done, with the longest of its
// progress intervals (from the dispatch or a heartbeat to the next heartbeat
// or the done); completions in the same cycle are folded in slot order, and a
// new entry applies from the next cycle. Not learned: a fenced slot, class 0,
// a type beyond the table, a late execution (below), and with LOCAL_ONLY an
// execution that ran away from its own slot (replayed, remapped or imported)
// or at a level faster than normal at any time. Learning does not depend on
// the enables; only reset clears the table.
//
// An entry is valid for a slot when M >= 1, N >= WARMUP and, with LOAD_GUARD,
// the highest load since the slot's dispatch is at most L. With a valid entry,
// and M' = M << MOVED_SH while the slot runs a task away from its own slot
// (else M' = M):
//   ctrl_i EN:       H = min(max(M' << SH_H, FLOOR), h_eff)
//                    C = min(max(M' << SH_C, 2 H), c_eff)
//   late_ctrl_i EN:  Late = max((M' + M'/4 * FRAC) << SH_L, LFLOOR), used only
//                    below the slot's suspect threshold (H if valid, else h_eff)
// An execution whose longest interval reaches Late is late and is not learned,
// so a step slowdown does not raise M; a gradual one is still learned.
//
// Simulation only: +bingo_adapt_log prints [BINGO_ADAPT_LEARN] when M grows or
// N reaches WARMUP, and with LOCAL_ONLY [BINGO_ADAPT_SKIP] for an execution it
// keeps out; +bingo_adapt_trace prints [TRACE] A7_EXEC per dispatch and
// progress, and with both and the late enable [TRACE] P9_TABLE per table change.
module bingo_hw_manager_wd_adapt #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    // Width of the watchdog timer (status_i.interval) and of M
    parameter int unsigned CounterWidth = 24,
    // The table covers core types 0 .. 2**WdAdaptTypeBits-1
    parameter int unsigned WdAdaptTypeBits = 2,
    // Classes per type (class 0 unused)
    parameter int unsigned WdClassWidth = 3,
    parameter int unsigned CoreTypeIdWidth = 4,
    // A slot with CoreMask = 0 (e.g. the host) never learns, is not counted in
    // the load and gets no learned threshold
    parameter logic [NumCores-1:0][NumClusters-1:0] CoreMask = '1,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId = '0,
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         wd_event_t = logic,
    parameter type         wd_status_t = logic,
    parameter type         wd_task_t = logic,
    parameter type         wd_learned_t = logic
) (
    input logic clk_i,
    input logic rst_ni,
    // BINGO_WD_ADAPT (reset 0: no learned H and C):
    //   [0] EN  [1] LOAD_GUARD  [4:2] SH_H  [7:5] SH_C
    //   [11:8] WARMUP (0 counts as 1)  [31:12] FLOOR (ticks)
    input logic [31:0] ctrl_i,
    // BINGO_WD_LATE (reset 0: no Late, learning as without it):
    //   [0] EN (Late, and late executions are not learned)  [3:1] SH_L
    //   [6] LOCAL_ONLY  [8:7] MOVED_SH  [10:9] FRAC  [31:12] LFLOOR (ticks)
    input logic [31:0] late_ctrl_i,
    // What the watchdog times: dispatch, an execution starts (a done in the same
    // cycle wins, as in the watchdog); done, it ends and is learned in this cycle
    input wd_event_t   event_i,
    // What the watchdog reports: progress (a done, or a heartbeat outside a
    // dispatch cycle, of a monitored, unfenced, busy slot), interval (ticks since
    // the dispatch or the last heartbeat), busy, fenced
    input wd_status_t  status_i,
    // The task of every slot: class, moved, fast
    input wd_task_t    task_i,
    // Static suspect and confirm thresholds of the slot (its core type's, or
    // the parameters): the caps of H and C; h_eff also bounds Late while H is
    // not valid
    input logic [NumCores-1:0][NumClusters-1:0][31:0] h_eff_i,
    input logic [NumCores-1:0][NumClusters-1:0][31:0] c_eff_i,
    // H (h_valid: EN, a valid entry, H >= 1), C (c_valid: H valid and C > H; the
    // top uses C only if it is shorter than the confirm threshold it would use
    // otherwise), Late (late_valid: late EN, a valid entry, below the suspect
    // threshold; a heartbeat or done at or after it is a late beat, the fault
    // precursor of the risk counter, bingo_hw_manager_risk)
    output wd_learned_t learned_o
);
    logic [NumCores-1:0][NumClusters-1:0][31:0] h_ad, c_ad, late_ad;
    logic [NumCores-1:0][NumClusters-1:0]       h_valid, c_valid, late_valid;
    assign learned_o = '{h: h_ad, c: c_ad, late: late_ad,
                         h_valid: h_valid, c_valid: c_valid, late_valid: late_valid};

    localparam int unsigned Types = 2**WdAdaptTypeBits;
    localparam int unsigned Classes = 2**WdClassWidth;
    localparam int unsigned LoadWidth = $clog2(NumCores * NumClusters + 1);
    localparam int unsigned LateMathWidth = CounterWidth + 7 > 64 ? CounterWidth + 7 : 64;
    typedef struct packed {
        logic [CounterWidth-1:0] maximum;
        logic [3:0] samples;
        logic [LoadWidth-1:0] load;
    } entry_t;
    entry_t [Types-1:0][Classes-1:0] table_q, table_d;
    logic [NumCores-1:0][NumClusters-1:0][LoadWidth-1:0] load_max_q, load_max_d;
    logic [NumCores-1:0][NumClusters-1:0][CounterWidth-1:0] imax_q, imax_d;
    // The current execution has run at a level faster than normal
    logic [NumCores-1:0][NumClusters-1:0] fast_q, fast_d;
    logic [LoadWidth-1:0] load_now, load_next;
    logic [3:0] warmup;

    assign warmup = ctrl_i[11:8] == 0 ? 4'd1 : ctrl_i[11:8];

    // Only thresholds depend on EN. History survives disable/re-enable and
    // is cleared only by reset, as are the watchdog's timers.
    always_comb begin
        load_now = '0;
        load_next = '0;
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                if (CoreMask[c][cl] && !status_i.fenced[c][cl]) begin
                    if (status_i.busy[c][cl]) load_now = load_now + 1'b1;
                    if (!event_i.done[c][cl] && (status_i.busy[c][cl] || event_i.dispatch[c][cl]))
                        load_next = load_next + 1'b1;
                end
            end
        end
        table_d = table_q;
        load_max_d = load_max_q;
        imax_d = imax_q;
        fast_d = fast_q;
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                automatic int type_id = int'(CoreTypeId[c][cl]);
                automatic int class_id = int'(task_i.wd_class[c][cl]);
                automatic logic [LoadWidth-1:0] peak =
                    load_now > load_max_q[c][cl] ? load_now : load_max_q[c][cl];
                automatic logic [CounterWidth-1:0] sample_max =
                    status_i.interval[c][cl] > imax_q[c][cl] ? status_i.interval[c][cl] : imax_q[c][cl];
                automatic logic fast_seen = fast_q[c][cl] || task_i.fast[c][cl];
                // BINGO_WD_LATE[6]: learn only executions comparable with the
                // ones on their own slot at the normal level
                if (CoreMask[c][cl] && !status_i.fenced[c][cl] && status_i.busy[c][cl] &&
                    event_i.done[c][cl] && class_id != 0 && type_id < Types &&
                    !(late_ctrl_i[0] && late_valid[c][cl] &&
                      sample_max >= late_ad[c][cl]) &&
                    !(late_ctrl_i[6] && (task_i.moved[c][cl] || fast_seen))) begin
                    // Each completed execution contributes its largest segment.
                    // Fold simultaneous completions in a fixed slot order.
                    if (sample_max > table_d[type_id][class_id].maximum)
                        table_d[type_id][class_id].maximum = sample_max;
                    if (table_d[type_id][class_id].samples != 4'd15)
                        table_d[type_id][class_id].samples =
                            table_d[type_id][class_id].samples + 1'b1;
                    if (peak > table_d[type_id][class_id].load)
                        table_d[type_id][class_id].load = peak;
                end
                if (CoreMask[c][cl] && !status_i.fenced[c][cl] && status_i.busy[c][cl]) begin
                    load_max_d[c][cl] = peak;
                    fast_d[c][cl] = fast_seen;
                    if (status_i.progress[c][cl] && !event_i.done[c][cl])
                        imax_d[c][cl] = sample_max;
                end
                // Match watchdog priority: done wins over dispatch. Heartbeats,
                // idle and fenced cycles never clear this execution's history.
                if (CoreMask[c][cl] && !status_i.fenced[c][cl] &&
                    event_i.dispatch[c][cl] && !event_i.done[c][cl]) begin
                    imax_d[c][cl] = '0;
                    load_max_d[c][cl] = load_next;
                    fast_d[c][cl] = task_i.fast[c][cl];
                end
            end
        end
    end

    // Wide arithmetic precedes both caps; a large shift must never wrap.
    always_comb begin
        h_ad = '0;
        c_ad = '0;
        h_valid = '0;
        c_valid = '0;
        late_ad = '0;
        late_valid = '0;
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                automatic int type_id = int'(CoreTypeId[c][cl]);
                automatic int class_id = int'(task_i.wd_class[c][cl]);
                automatic logic [LoadWidth-1:0] peak =
                    load_now > load_max_q[c][cl] ? load_now : load_max_q[c][cl];
                automatic logic [63:0] h, confirm;
                automatic logic [LateMathWidth-1:0] late_bound;
                automatic logic entry_valid;
                // BINGO_WD_LATE[8:7]: an execution away from its own slot uses
                // the learned maximum shifted left by this many more bits
                automatic logic [1:0] moved_shift = task_i.moved[c][cl] ? late_ctrl_i[8:7] : 2'd0;
                h = '0;
                confirm = '0;
                late_bound = '0;
                entry_valid = CoreMask[c][cl] && type_id < Types && class_id != 0 &&
                    table_q[type_id][class_id].maximum >= 1 &&
                    table_q[type_id][class_id].samples >= warmup &&
                    (!ctrl_i[1] || peak <= table_q[type_id][class_id].load);
                if (ctrl_i[0] && entry_valid) begin
                    h = (64'(table_q[type_id][class_id].maximum) << moved_shift) << ctrl_i[4:2];
                    if (h < ctrl_i[31:12]) h = 64'(ctrl_i[31:12]);
                    if (h > h_eff_i[c][cl]) h = 64'(h_eff_i[c][cl]);
                    confirm = (64'(table_q[type_id][class_id].maximum) << moved_shift) << ctrl_i[7:5];
                    if (confirm < (h << 1)) confirm = h << 1;
                    if (confirm > c_eff_i[c][cl]) confirm = 64'(c_eff_i[c][cl]);
                    h_ad[c][cl] = 32'(h);
                    c_ad[c][cl] = 32'(confirm);
                    h_valid[c][cl] = h >= 1;
                    c_valid[c][cl] = h >= 1 && confirm > h;
                end
                // Prediction shares history/guards, not the detection EN bit.
                // Compare wide before narrowing; equality with suspect is invalid.
                if (late_ctrl_i[0] && entry_valid) begin
                    // BINGO_WD_LATE[10:9] FRAC: (1 + FRAC / 4) times the maximum,
                    // then SH_L
                    late_bound = LateMathWidth'(table_q[type_id][class_id].maximum) << moved_shift;
                    late_bound = (late_bound + (late_bound >> 2) * late_ctrl_i[10:9]) <<
                                 late_ctrl_i[3:1];
                    if (late_bound < late_ctrl_i[31:12])
                        late_bound = LateMathWidth'(late_ctrl_i[31:12]);
                    if (late_bound < (h_valid[c][cl] ? h : 64'(h_eff_i[c][cl]))) begin
                        late_ad[c][cl] = 32'(late_bound);
                        late_valid[c][cl] = 1'b1;
                    end
                end
            end
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            table_q <= '0;
            load_max_q <= '0;
            imax_q <= '0;
            fast_q <= '0;
        end else begin
            table_q <= table_d;
            load_max_q <= load_max_d;
            imax_q <= imax_d;
            fast_q <= fast_d;
        end
    end

`ifndef SYNTHESIS
    bit learn_log, execution_log;
    // The static flag-image control can opt into identical observation without
    // enabling adaptive thresholds. Default-image regressions print nothing new.
    initial learn_log = $test$plusargs("bingo_adapt_log");
    initial execution_log = $test$plusargs("bingo_adapt_trace");
    always @(posedge clk_i) begin
        if (rst_ni && execution_log) begin
            for (int c = 0; c < NumCores; c++) begin
                for (int cl = 0; cl < NumClusters; cl++) begin
                    if (CoreMask[c][cl] && !status_i.fenced[c][cl] &&
                        (event_i.dispatch[c][cl] || status_i.progress[c][cl]))
                        $display("[TRACE] A7_EXEC %0t core=%0d cluster=%0d type=%0d class=%0d event=%s interval=%0d imax=%0d load=%0d",
                                 $time, c, cl, CoreTypeId[c][cl], task_i.wd_class[c][cl],
                                 event_i.done[c][cl] ? "done" :
                                 event_i.dispatch[c][cl] ? "dispatch" : "heartbeat",
                                 status_i.interval[c][cl], imax_q[c][cl], load_max_q[c][cl]);
                end
            end
        end
        if (rst_ni && learn_log && late_ctrl_i[6]) begin
            for (int c = 0; c < NumCores; c++) begin
                for (int cl = 0; cl < NumClusters; cl++) begin
                    if (CoreMask[c][cl] && !status_i.fenced[c][cl] && status_i.busy[c][cl] && event_i.done[c][cl] &&
                        task_i.wd_class[c][cl] != 0 && int'(CoreTypeId[c][cl]) < Types &&
                        (task_i.moved[c][cl] || fast_q[c][cl] || task_i.fast[c][cl]))
                        $display("[BINGO_ADAPT_SKIP] %0t core=%0d cluster=%0d type=%0d class=%0d moved=%0d fast=%0d",
                                 $time, c, cl, CoreTypeId[c][cl], task_i.wd_class[c][cl], task_i.moved[c][cl],
                                 fast_q[c][cl] || task_i.fast[c][cl]);
                end
            end
        end
        if (rst_ni && learn_log) begin
            for (int t = 0; t < Types; t++) begin
                for (int k = 1; k < Classes; k++) begin
                    if (table_d[t][k].maximum > table_q[t][k].maximum ||
                        (table_q[t][k].samples < warmup && table_d[t][k].samples >= warmup))
                        $display("[BINGO_ADAPT_LEARN] %0t type=%0d class=%0d max=%0d n=%0d load=%0d",
                                 $time, t, k, table_d[t][k].maximum,
                                 table_d[t][k].samples, table_d[t][k].load);
                    if (late_ctrl_i[0] && execution_log && table_d[t][k] != table_q[t][k])
                        $display("[TRACE] P9_TABLE %0t type=%0d class=%0d max=%0d n=%0d load=%0d",
                                 $time, t, k, table_d[t][k].maximum,
                                 table_d[t][k].samples, table_d[t][k].load);
                end
            end
        end
    end
`endif
endmodule
