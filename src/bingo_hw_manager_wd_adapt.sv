module bingo_hw_manager_wd_adapt #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CounterWidth = 24,
    parameter int unsigned WdAdaptTypeBits = 2,
    parameter int unsigned WdClassWidth = 3,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0] CoreMask = '1,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId = '0
) (
    input logic clk_i,
    input logic rst_ni,
    input logic [31:0] ctrl_i,
    input logic [31:0] late_ctrl_i = '0,
    input logic [NumCores-1:0][NumClusters-1:0] progress_i,
    input logic [NumCores-1:0][NumClusters-1:0][CounterWidth-1:0] interval_i,
    input logic [NumCores-1:0][NumClusters-1:0] dispatch_i,
    input logic [NumCores-1:0][NumClusters-1:0] done_i,
    input logic [NumCores-1:0][NumClusters-1:0] fenced_i,
    input logic [NumCores-1:0][NumClusters-1:0] busy_i,
    input logic [NumCores-1:0][NumClusters-1:0][WdClassWidth-1:0] class_i,
    // The slot's task runs away from its own logical slot (replayed, remapped
    // or imported), latched at dispatch like class_i.
    input logic [NumCores-1:0][NumClusters-1:0] moved_i = '0,
    // The slot's power domain runs faster than the normal level this cycle.
    input logic [NumCores-1:0][NumClusters-1:0] fast_i = '0,
    input logic [NumCores-1:0][NumClusters-1:0][31:0] h_eff_i,
    input logic [NumCores-1:0][NumClusters-1:0][31:0] c_eff_i,
    output logic [NumCores-1:0][NumClusters-1:0][31:0] h_ad_o,
    output logic [NumCores-1:0][NumClusters-1:0][31:0] c_ad_o,
    output logic [NumCores-1:0][NumClusters-1:0] h_valid_o,
    output logic [NumCores-1:0][NumClusters-1:0] c_valid_o,
    output logic [NumCores-1:0][NumClusters-1:0][31:0] late_ad_o,
    output logic [NumCores-1:0][NumClusters-1:0] late_valid_o
);
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
                if (CoreMask[c][cl] && !fenced_i[c][cl]) begin
                    if (busy_i[c][cl]) load_now = load_now + 1'b1;
                    if (!done_i[c][cl] && (busy_i[c][cl] || dispatch_i[c][cl]))
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
                automatic int class_id = int'(class_i[c][cl]);
                automatic logic [LoadWidth-1:0] peak =
                    load_now > load_max_q[c][cl] ? load_now : load_max_q[c][cl];
                automatic logic [CounterWidth-1:0] sample_max =
                    interval_i[c][cl] > imax_q[c][cl] ? interval_i[c][cl] : imax_q[c][cl];
                automatic logic fast_seen = fast_q[c][cl] || fast_i[c][cl];
                // BINGO_WD_LATE[6]: learn only executions comparable with the
                // ones on their own slot at the normal level
                if (CoreMask[c][cl] && !fenced_i[c][cl] && busy_i[c][cl] &&
                    done_i[c][cl] && class_id != 0 && type_id < Types &&
                    !(late_ctrl_i[0] && late_valid_o[c][cl] &&
                      sample_max >= late_ad_o[c][cl]) &&
                    !(late_ctrl_i[6] && (moved_i[c][cl] || fast_seen))) begin
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
                if (CoreMask[c][cl] && !fenced_i[c][cl] && busy_i[c][cl]) begin
                    load_max_d[c][cl] = peak;
                    fast_d[c][cl] = fast_seen;
                    if (progress_i[c][cl] && !done_i[c][cl])
                        imax_d[c][cl] = sample_max;
                end
                // Match watchdog priority: done wins over dispatch. Heartbeats,
                // idle and fenced cycles never clear this execution's history.
                if (CoreMask[c][cl] && !fenced_i[c][cl] &&
                    dispatch_i[c][cl] && !done_i[c][cl]) begin
                    imax_d[c][cl] = '0;
                    load_max_d[c][cl] = load_next;
                    fast_d[c][cl] = fast_i[c][cl];
                end
            end
        end
    end

    // Wide arithmetic precedes both caps; a large shift must never wrap.
    always_comb begin
        h_ad_o = '0;
        c_ad_o = '0;
        h_valid_o = '0;
        c_valid_o = '0;
        late_ad_o = '0;
        late_valid_o = '0;
        for (int c = 0; c < NumCores; c++) begin
            for (int cl = 0; cl < NumClusters; cl++) begin
                automatic int type_id = int'(CoreTypeId[c][cl]);
                automatic int class_id = int'(class_i[c][cl]);
                automatic logic [LoadWidth-1:0] peak =
                    load_now > load_max_q[c][cl] ? load_now : load_max_q[c][cl];
                automatic logic [63:0] h, confirm;
                automatic logic [LateMathWidth-1:0] late_bound;
                automatic logic entry_valid;
                // BINGO_WD_LATE[8:7]: an execution away from its own slot uses
                // the learned maximum shifted left by this many more bits
                automatic logic [1:0] moved_shift = moved_i[c][cl] ? late_ctrl_i[8:7] : 2'd0;
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
                    h_ad_o[c][cl] = 32'(h);
                    c_ad_o[c][cl] = 32'(confirm);
                    h_valid_o[c][cl] = h >= 1;
                    c_valid_o[c][cl] = h >= 1 && confirm > h;
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
                    if (late_bound < (h_valid_o[c][cl] ? h : 64'(h_eff_i[c][cl]))) begin
                        late_ad_o[c][cl] = 32'(late_bound);
                        late_valid_o[c][cl] = 1'b1;
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
                    if (CoreMask[c][cl] && !fenced_i[c][cl] &&
                        (dispatch_i[c][cl] || progress_i[c][cl]))
                        $display("[TRACE] A7_EXEC %0t core=%0d cluster=%0d type=%0d class=%0d event=%s interval=%0d imax=%0d load=%0d",
                                 $time, c, cl, CoreTypeId[c][cl], class_i[c][cl],
                                 done_i[c][cl] ? "done" :
                                 dispatch_i[c][cl] ? "dispatch" : "heartbeat",
                                 interval_i[c][cl], imax_q[c][cl], load_max_q[c][cl]);
                end
            end
        end
        if (rst_ni && learn_log && late_ctrl_i[6]) begin
            for (int c = 0; c < NumCores; c++) begin
                for (int cl = 0; cl < NumClusters; cl++) begin
                    if (CoreMask[c][cl] && !fenced_i[c][cl] && busy_i[c][cl] && done_i[c][cl] &&
                        class_i[c][cl] != 0 && int'(CoreTypeId[c][cl]) < Types &&
                        (moved_i[c][cl] || fast_q[c][cl] || fast_i[c][cl]))
                        $display("[BINGO_ADAPT_SKIP] %0t core=%0d cluster=%0d type=%0d class=%0d moved=%0d fast=%0d",
                                 $time, c, cl, CoreTypeId[c][cl], class_i[c][cl], moved_i[c][cl],
                                 fast_q[c][cl] || fast_i[c][cl]);
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
