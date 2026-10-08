`timescale 1ns/1ps
module tb_bingo_hw_manager_wd_adapt_boundary;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 0;
    logic [0:0][0:0] dispatch, done, hb, ticks, busy, fenced, suspect, progress;
    logic [0:0][0:0][7:0] interval;
    logic [0:0][0:0][31:0] h_ad, c_ad;
    logic [0:0][0:0] hv, cv;
    logic [0:0][0:0][7:0] h_thr, c_thr;
    logic [31:0] ctrl;
    assign h_thr = hv ? 8'(h_ad) : 8'd40;
    assign c_thr = cv ? 8'(c_ad) : 8'd100;
    bingo_hw_manager_watchdog #(
        .NumCores(1), .NumClusters(1), .CounterWidth(8),
        .HeartbeatTimeoutCycles(40), .ConfirmTimeoutCycles(100), .CoreTypeId(4'd1)
    ) wd (
        .clk_i(clk), .rst_ni(rst), .task_dispatched_i(dispatch), .task_done_i(done),
        .heartbeat_i(hb), .waiting_task_i('0), .tick_i(ticks),
        .suspect_thr_i(h_thr), .confirm_thr_i(c_thr), .suspect_thr_valid_i(hv),
        .adapt_confirm_i(cv), .wd_class_i(3'd1),
        .core_busy_o(busy), .core_fenced_o(fenced), .core_dead_suspect_o(suspect),
        .core_available_o(), .late_o(), .progress_o(progress), .interval_o(interval)
    );
    bingo_hw_manager_wd_adapt #(
        .NumCores(1), .NumClusters(1), .CounterWidth(8), .CoreTypeId(4'd1)
    ) adapt (
        .clk_i(clk), .rst_ni(rst), .ctrl_i(ctrl), .progress_i(progress),
        .interval_i(interval), .dispatch_i(dispatch), .done_i(done),
        .fenced_i(fenced), .busy_i(busy), .class_i(3'd1), .h_eff_i(32'd40),
        .c_eff_i(32'd100), .h_ad_o(h_ad), .c_ad_o(c_ad), .h_valid_o(hv), .c_valid_o(cv)
    );
    task automatic step();
        @(posedge clk); #1;
    endtask
    task automatic setup();
        @(negedge clk);
        rst = 0; dispatch = 0; done = 0; hb = 0; ticks = 1;
        ctrl = (5 << 12) | (1 << 8) | (2 << 5) | (1 << 2) | 1;
        step();
        @(negedge clk); rst = 1; dispatch = 1;
        step();
        @(negedge clk); dispatch = 0;
        repeat (4) begin step(); @(negedge clk); end
        hb = 1;
        #1;
        if (!progress || interval != 4)
            $fatal(1, "[ADAPT_BOUNDARY] wrong timer snapshot");
        step();
        if (!hv || h_ad != 8 || c_ad != 16 || wd.timer_q[0][0] != 0)
            $fatal(1, "[ADAPT_BOUNDARY] learning did not take effect within one edge");
        @(negedge clk); hb = 0;
    endtask
    initial begin
        for (int completion = 0; completion < 2; completion++) begin
            setup();
            repeat (16) begin step(); @(negedge clk); end
            if (wd.timer_q[0][0] != 16 || fenced)
                $fatal(1, "[ADAPT_BOUNDARY] missed C_ad boundary");
            if (completion) done = 1;
            else hb = 1;
            step();
            if (fenced || suspect || wd.timer_q[0][0] != 0)
                $fatal(1, "[ADAPT_BOUNDARY] done/heartbeat lost fence priority");
        end
        setup();
        for (int cycle = 1; cycle <= 34; cycle++) begin
            ticks = cycle % 2 == 1;
            step();
            if (suspect !== (cycle >= 15) || fenced !== (cycle >= 32))
                $fatal(1, "[ADAPT_BOUNDARY] two-cycle ticks cycle=%0d", cycle);
            @(negedge clk);
        end
        hb = 1; done = 1;
        #1;
        if (progress)
            $fatal(1, "[ADAPT_BOUNDARY] fenced progress escaped");
        step();
        if (adapt.table_q[1][1].samples != 1 || adapt.table_q[1][1].maximum != 4)
            $fatal(1, "[ADAPT_BOUNDARY] fenced termination interval learned");
        setup();
        dispatch = 1; hb = 1;
        #1;
        if (progress)
            $fatal(1, "[ADAPT_BOUNDARY] heartbeat at dispatch was learned");
        step();
        if (adapt.table_q[1][1].samples != 1 || wd.timer_q[0][0] != 0)
            $fatal(1, "[ADAPT_BOUNDARY] dispatch priority changed");
        @(negedge clk);
        rst = 0; dispatch = 0; done = 0; hb = 0; ticks = 1;
        ctrl = (1 << 8) | 1; // floor=0, shifts=0
        step();
        @(negedge clk); rst = 1; dispatch = 1;
        step();
        @(negedge clk); dispatch = 0; hb = 1;
        step();
        if (adapt.table_q[1][1].maximum != 0 || adapt.table_q[1][1].samples != 1 ||
            hv || cv || suspect || fenced || h_thr != 40 || c_thr != 100)
            $fatal(1, "[ADAPT_BOUNDARY] M=0/floor=0 did not retain static thresholds");
        @(negedge clk); hb = 0;
        repeat (10) begin step(); @(negedge clk); end
        if (hv || cv || suspect || fenced)
            $fatal(1, "[ADAPT_BOUNDARY] zero interval triggered suspicion");
        ctrl[31:12] = 5;
        #1;
        if (hv || cv || h_thr != 40 || c_thr != 100)
            $fatal(1, "[ADAPT_BOUNDARY] floor incorrectly validated M=0");
        hb = 1;
        step();
        if (!hv || !cv || h_ad != 10 || c_ad != 20)
            $fatal(1, "[ADAPT_BOUNDARY] positive sample did not enable adaptation");
        $display("Adaptive watchdog boundary tests passed");
        $finish;
    end
    initial begin
        #100000;
        $fatal(1, "[ADAPT_BOUNDARY] timeout");
    end
endmodule
