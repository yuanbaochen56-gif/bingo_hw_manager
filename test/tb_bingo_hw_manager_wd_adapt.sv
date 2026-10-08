`timescale 1ns/1ps
module tb_bingo_hw_manager_wd_adapt;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 0;
    logic [31:0] ctrl;
    logic [4:0][0:0] progress, dispatch, done, fenced, busy;
    logic [4:0][0:0][15:0] interval;
    logic [4:0][0:0][2:0] classes;
    logic [4:0][0:0][31:0] h_eff, c_eff, h_ad, c_ad;
    logic [4:0][0:0] hv, cv;

    bingo_hw_manager_wd_adapt #(
        .NumCores(5), .NumClusters(1), .CounterWidth(16),
        .CoreMask(5'b01111), .CoreTypeId({4'd1, 4'd4, 4'd2, 4'd1, 4'd1})
    ) dut (
        .clk_i(clk), .rst_ni(rst), .ctrl_i(ctrl),
        .progress_i(progress), .interval_i(interval), .dispatch_i(dispatch),
        .done_i(done), .fenced_i(fenced), .busy_i(busy), .class_i(classes),
        .h_eff_i(h_eff), .c_eff_i(c_eff), .h_ad_o(h_ad), .c_ad_o(c_ad),
        .h_valid_o(hv), .c_valid_o(cv)
    );

    function automatic logic [31:0] config_word(
        input bit en = 1, guard = 1, input int sh_h = 1, sh_c = 2,
        warmup = 3, floor_ticks = 5);
        return (floor_ticks << 12) | (warmup << 8) | (sh_c << 5) |
               (sh_h << 2) | (int'(guard) << 1) | int'(en);
    endfunction

    task automatic step();
        @(posedge clk); #1;
    endtask

    task automatic reset();
        @(negedge clk);
        rst = 0; progress = '0; dispatch = '0; done = '0;
        fenced = '0; busy = '0; interval = '0; classes = '0;
        ctrl = config_word();
        for (int c = 0; c < 5; c++) begin
            h_eff[c][0] = 100; c_eff[c][0] = 200;
        end
        step();
        @(negedge clk); rst = 1;
        step();
    endtask

    task automatic sample(input int core, x, cls, load = 1);
        @(negedge clk);
        progress = '0; dispatch = '0; done = '0; busy = '0;
        busy[core][0] = 1;
        for (int c = 0; c < load-1; c++) busy[c+1][0] = 1;
        classes[core][0] = 3'(cls);
        interval[core][0] = 16'(x);
        progress[core][0] = 1;
        step();
        @(negedge clk); progress = '0;
    endtask

    task automatic check_bound(input int core, h, c, input bit hvalid = 1, cvalid = 1);
        #1;
        if (hv[core][0] !== hvalid || cv[core][0] !== cvalid ||
            (hvalid && h_ad[core][0] != h) || (hvalid && c_ad[core][0] != c))
            $fatal(1, "[ADAPT_TB] core=%0d H=%0d C=%0d valid=%0b/%0b expected=%0d/%0d %0b/%0b",
                   core, h_ad[core][0], c_ad[core][0], hv[core][0], cv[core][0],
                   h, c, hvalid, cvalid);
    endtask

    initial begin
        reset();
        sample(0, 5, 1);
        check_bound(0, 0, 0, 0, 0);
        sample(0, 3, 1);
        check_bound(0, 0, 0, 0, 0);
        sample(0, 8, 1);
        check_bound(0, 16, 32);
        if (dut.table_q[1][1].maximum != 8 || dut.table_q[1][1].samples != 3 ||
            dut.table_q[1][1].load != 1)
            $fatal(1, "[ADAPT_TB] M/N/L or exact warmup transition");
        repeat (20) sample(0, 1, 1);
        if (dut.table_q[1][1].samples != 15 || dut.table_q[1][1].maximum != 8)
            $fatal(1, "[ADAPT_TB] saturation or monotonic max");
        sample(0, 99, 0);
        if (dut.table_q[1][0] != '0)
            $fatal(1, "[ADAPT_TB] class zero learned");
        check_bound(0, 0, 0, 0, 0);
        sample(4, 88, 1);
        if (dut.table_q[1][1].maximum != 8)
            $fatal(1, "[ADAPT_TB] masked host learned");
        check_bound(4, 0, 0, 0, 0);
        sample(3, 77, 1);
        check_bound(3, 0, 0, 0, 0);
        sample(2, 19, 1);
        if (dut.table_q[2][1].maximum != 19 || dut.table_q[1][1].maximum != 8)
            $fatal(1, "[ADAPT_TB] types share history");
        ctrl = config_word(1, 0, 1, 2, 1);
        check_bound(2, 38, 76);
        fenced[0][0] = 1;
        sample(0, 199, 1);
        if (dut.table_q[1][1].maximum != 8)
            $fatal(1, "[ADAPT_TB] fenced interval learned");
        fenced = '0;
        ctrl = config_word(0, 0, 1, 2, 1);
        sample(0, 21, 2);
        check_bound(0, 0, 0, 0, 0);
        ctrl = config_word(1, 0, 1, 2, 1);
        check_bound(0, 42, 84);
        ctrl = config_word(1, 0, 1, 2, 0, 90);
        check_bound(0, 90, 180);
        h_eff[0][0] = 50; c_eff[0][0] = 60;
        check_bound(0, 50, 60);
        c_eff[0][0] = 49;
        check_bound(0, 50, 49, 1, 0);
        c_eff[0][0] = 0;
        check_bound(0, 50, 0, 1, 0);
        h_eff[0][0] = 100; c_eff[0][0] = 200;
        ctrl = config_word(1, 0, 7, 7, 1, 0);
        sample(0, 65535, 3);
        check_bound(0, 100, 200);

        reset();
        ctrl = config_word(1, 1, 1, 2, 1);
        sample(0, 6, 1);
        check_bound(0, 12, 24);
        @(negedge clk); busy[1][0] = 1;
        step();
        check_bound(0, 0, 0, 0, 0);
        @(negedge clk); busy[1][0] = 0;
        step();
        check_bound(0, 0, 0, 0, 0); // peak is sticky within an interval
        ctrl[1] = 0;
        check_bound(0, 12, 24);
        ctrl[1] = 1;
        sample(0, 5, 1);
        if (dut.table_q[1][1].load != 2)
            $fatal(1, "[ADAPT_TB] load peak not learned");
        check_bound(0, 12, 24);

        reset();
        ctrl = config_word(1, 0, 1, 2, 2);
        @(negedge clk);
        busy[0][0] = 1; busy[1][0] = 1;
        progress[0][0] = 1; progress[1][0] = 1;
        classes[0][0] = 4; classes[1][0] = 4;
        interval[0][0] = 7; interval[1][0] = 11;
        step();
        if (dut.table_q[1][4].samples != 2 || dut.table_q[1][4].maximum != 11 ||
            dut.table_q[1][4].load != 2)
            $fatal(1, "[ADAPT_TB] simultaneous same-entry samples lost");
        check_bound(0, 22, 44);
        check_bound(1, 22, 44);
        @(negedge clk); progress = '0;
        reset();
        ctrl = config_word(1, 0, 0, 0, 1, 0);
        sample(0, 0, 1);
        check_bound(0, 0, 0, 0, 0);
        ctrl = config_word(1, 0, 0, 0, 1, 5);
        check_bound(0, 0, 0, 0, 0); // a positive floor must not validate M=0
        sample(0, 1, 1);
        check_bound(0, 5, 10);
        h_eff[0][0] = 0;
        check_bound(0, 0, 0, 0, 0); // even a learned M cannot validate H_ad=0
        $display("Watchdog adaptive bound tests passed");
        $finish;
    end
    initial begin
        #100000;
        $fatal(1, "[ADAPT_TB] timeout");
    end
endmodule
