// Exercise the real top's full-width validation and the real watchdog together.
localparam int unsigned EXPECTED_TASK_COUNT = 999;
localparam int unsigned DEADLOCK_THRESHOLD = 100000;
localparam int unsigned DEP_MATRIX_LOG_INTERVAL = 0;
localparam int CW = $clog2((WATCHDOG_CONFIRM_TIMEOUT > 20 ?
                          WATCHDOG_CONFIRM_TIMEOUT : 20) + 1) + 1;
logic unit_rst = 0;
logic [1:0][0:0] ud = '0, done = '0, hb = '0, ticks = '1;
logic [1:0][0:0] ubusy, suspect, ufence;

bingo_hw_manager_watchdog #(
    .NumCores(2), .NumClusters(1), .CounterWidth(CW),
    .HeartbeatTimeoutCycles(20), .ConfirmTimeoutCycles(WATCHDOG_CONFIRM_TIMEOUT),
    .CoreTypeId(8'h21)
) unit_wd (
    .clk_i(clk_i), .rst_ni(unit_rst),
    .task_dispatched_i(ud), .task_done_i(done), .heartbeat_i(hb),
    .waiting_task_i('0), .tick_i(ticks), .risk_i('1),
    .risk_confirm_i(CW'(risk_confirm[0])),
    .risk_confirm_valid_i(gen_dut[0].i_dut.wd_risk_confirm_valid),
    .suspect_thr_i(gen_dut[0].i_dut.wd_suspect_thr),
    .confirm_thr_i(gen_dut[0].i_dut.wd_confirm_thr),
    .core_busy_o(ubusy), .core_dead_suspect_o(suspect), .core_fenced_o(ufence),
    .core_available_o(), .late_o()
);

task automatic step();
    @(posedge clk_i); #1;
endtask

task automatic start_unit(input logic [31:0] h, c, r);
    @(negedge clk_i);
    unit_rst = 0; ud = '0; done = '0; hb = '0; ticks = '1;
    wd_type_h[0] = '0; wd_type_c[0] = '0;
    wd_type_h[0][0] = 1; wd_type_c[0][0] = 2; // unused type must not leak
    wd_type_h[0][1] = h; wd_type_c[0][1] = c;
    risk_confirm[0] = r;
    step();
    @(negedge clk_i);
    unit_rst = 1; ud = '1;
    step();
    @(negedge clk_i); ud = '0;
endtask

task automatic check_thresholds(input logic [31:0] h, c, r, input int period = 1);
    automatic int heff, ceff, threshold, hs;
    automatic bit rvalid;
    start_unit(h, c, r);
    heff = h != 0 && h <= 20 ? h : 20;
    ceff = WATCHDOG_CONFIRM_TIMEOUT != 0 && c != 0 &&
           c <= WATCHDOG_CONFIRM_TIMEOUT && c > heff ? c : WATCHDOG_CONFIRM_TIMEOUT;
    for (int cycle = 1; cycle <= 2*80+5; cycle++) begin
        ticks = (cycle % period == 1 % period) ? '1 : '0;
        step();
        for (int core = 0; core < 2; core++) begin
            hs = core == 0 ? heff : 20;
            threshold = core == 0 ? ceff : WATCHDOG_CONFIRM_TIMEOUT;
            rvalid = WATCHDOG_CONFIRM_TIMEOUT != 0 && r > hs && r < threshold;
            if (rvalid) threshold = r;
            if (ufence[core][0] !== (threshold != 0 && cycle >= (threshold-1)*period+2))
                $fatal(1, "[TYPE_THR] core=%0d h=%0d c=%0d r=%0d period=%0d cycle=%0d fence=%0b",
                       core, h, c, r, period, cycle, ufence[core][0]);
            if (suspect[core][0] !== (cycle >= (hs-1)*period+1))
                $fatal(1, "[TYPE_THR] core=%0d wrong suspect at cycle=%0d", core, cycle);
        end
        @(negedge clk_i);
    end
endtask

initial begin
    wait (rst_ni); repeat (20) @(posedge clk_i);
    check_thresholds(8, 30, 0);
    check_thresholds(0, 0, 0);
    check_thresholds(21, 30, 0);
    check_thresholds(8, 8, 0);
    check_thresholds(8, 7, 0);
    check_thresholds(8, 81, 0);
    check_thresholds(8 + (1 << CW), 30 + (1 << CW), 0);
    check_thresholds(8, 30, 15); // valid only for the shorter type
    check_thresholds(8, 30, 30); // R == C_t is invalid for this type
    check_thresholds(8, 30, 40); // invalid for A, valid for B
    check_thresholds(8, 30, 8);
    check_thresholds(8, 30, 7);
    check_thresholds(8, 30, 15 + (1 << CW));
    check_thresholds(8, 30, 0, 2);
    if (WATCHDOG_CONFIRM_TIMEOUT != 0) begin
        for (int completion = 0; completion < 2; completion++) begin
            start_unit(8, 30, 0);
            repeat (30) begin step(); @(negedge clk_i); end
            if (unit_wd.timer_q[0][0] != 30 || ufence[0][0])
                $fatal(1, "[TYPE_THR] missed C_t boundary fixture");
            if (completion) done[0][0] = 1'b1;
            else hb[0][0] = 1'b1;
            step();
            if (ufence[0][0] || suspect[0][0] || unit_wd.timer_q[0][0] != 0)
                $fatal(1, "[TYPE_THR] done/heartbeat lost priority at C_t");
        end
    end
    $display("Watchdog type threshold tests passed");
    $finish;
end
