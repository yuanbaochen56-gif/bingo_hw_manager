`timescale 1ns/1ps
module tb_bingo_hw_manager_wd_adapt_execution;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 0;
    logic [1:0][0:0] dispatch, done, hb, busy, fenced, suspect, progress;
    logic [1:0][0:0][15:0] interval, h_thr, c_thr;
    logic [1:0][0:0][31:0] h_ad, c_ad;
    logic [1:0][0:0] hv, cv;
    logic [31:0] ctrl;
    for (genvar c = 0; c < 2; c++) begin
        assign h_thr[c][0] = hv[c][0] ? 16'(h_ad[c][0]) : 16'd40000;
        assign c_thr[c][0] = cv[c][0] ? 16'(c_ad[c][0]) : 16'd60000;
    end
    bingo_hw_manager_watchdog #(
        .NumCores(2), .NumClusters(1), .CounterWidth(16), .CoreTypeId({4'd1, 4'd1}),
        .HeartbeatTimeoutCycles(40000), .ConfirmTimeoutCycles(60000)
    ) wd (
        .clk_i(clk), .rst_ni(rst), .task_dispatched_i(dispatch), .task_done_i(done),
        .heartbeat_i(hb), .waiting_task_i('0), .tick_i(2'b11),
        .suspect_thr_i(h_thr), .confirm_thr_i(c_thr), .suspect_thr_valid_i(hv),
        .adapt_confirm_i(cv), .wd_class_i({3'd1, 3'd1}),
        .core_busy_o(busy), .core_fenced_o(fenced), .core_dead_suspect_o(suspect),
        .core_available_o(), .late_o(), .progress_o(progress), .interval_o(interval)
    );
    bingo_hw_manager_wd_adapt #(
        .NumCores(2), .NumClusters(1), .CounterWidth(16), .CoreTypeId({4'd1, 4'd1})
    ) adapt (
        .clk_i(clk), .rst_ni(rst), .ctrl_i(ctrl), .progress_i(progress),
        .interval_i(interval), .dispatch_i(dispatch), .done_i(done),
        .fenced_i(fenced), .busy_i(busy), .class_i({3'd1, 3'd1}),
        .h_eff_i({32'd40000, 32'd40000}), .c_eff_i({32'd60000, 32'd60000}),
        .h_ad_o(h_ad), .c_ad_o(c_ad), .h_valid_o(hv), .c_valid_o(cv)
    );
    task automatic step();
        @(posedge clk); #1;
        if (suspect || fenced)
            $fatal(1, "[ADAPT_EXECUTION] healthy execution suspected or fenced");
    endtask
    task automatic ticks(input int count);
        repeat (count) begin step(); @(negedge clk); end
    endtask
    initial begin
        ctrl = 32'h7D0147;
        dispatch = '0; done = '0; hb = '0;
        step();
        @(negedge clk); rst = 1; dispatch = '1;
        step();
        @(negedge clk); dispatch = '0;
        ticks(33);
        hb = '1;
        step();
        if (hv || cv || adapt.table_q[1][1].samples != 0 ||
            adapt.imax_q[0][0] != 33 || adapt.imax_q[1][0] != 33)
            $fatal(1, "[ADAPT_EXECUTION] prologue changed its own thresholds");
        @(negedge clk); hb = '0;
        ticks(8160);
        hb[0][0] = 1;
        step();
        if (hv || adapt.table_q[1][1].samples != 0 || adapt.imax_q[0][0] != 8160)
            $fatal(1, "[ADAPT_EXECUTION] body heartbeat learned before done");
        @(negedge clk); hb = '0; done[0][0] = 1;
        step();
        if (adapt.table_q[1][1].samples != 1 || adapt.table_q[1][1].maximum != 8160 ||
            adapt.table_q[1][1].load != 2 || !hv[1][0] ||
            h_ad[1][0] != 16320 || c_ad[1][0] != 32640)
            $fatal(1, "[ADAPT_EXECUTION] concurrent class did not use completed peer");
        @(negedge clk); done = '0; dispatch[0][0] = 1;
        step();
        if (adapt.imax_q[0][0] != 0 || h_thr[0][0] != 16320 || c_thr[0][0] != 32640)
            $fatal(1, "[ADAPT_EXECUTION] next execution did not reset imax/use body bound");
        @(negedge clk); dispatch = '0; done = '1;
        step();
        if (adapt.table_q[1][1].samples != 3 || adapt.table_q[1][1].maximum != 8163)
            $fatal(1, "[ADAPT_EXECUTION] simultaneous completions lost or N counted segments");
        $display("Adaptive complete-execution tests passed");
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1, "[ADAPT_EXECUTION] timeout");
    end
endmodule
