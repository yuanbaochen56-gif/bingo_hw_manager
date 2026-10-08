logic clk = 0;
always #5 clk = ~clk;
logic rst = 0;
logic [31:0] ctrl, late_ctrl;
logic [2:0][0:0] progress, dispatch, done, fenced, busy;
logic [2:0][0:0][31:0] interval, h_eff, c_eff, h_ad, c_ad, late_ad;
logic [2:0][0:0][2:0] classes;
logic [2:0][0:0] hv, cv, lv;

bingo_hw_manager_wd_adapt #(
    .NumCores(3), .NumClusters(1), .CounterWidth(32),
    .CoreTypeId({4'd4, 4'd1, 4'd1})
) dut (
    .clk_i(clk), .rst_ni(rst), .ctrl_i(ctrl), .late_ctrl_i(late_ctrl),
    .progress_i(progress), .interval_i(interval), .dispatch_i(dispatch), .done_i(done),
    .fenced_i(fenced), .busy_i(busy), .class_i(classes),
    .h_eff_i(h_eff), .c_eff_i(c_eff), .h_ad_o(h_ad), .c_ad_o(c_ad),
    .h_valid_o(hv), .c_valid_o(cv), .late_ad_o(late_ad), .late_valid_o(lv)
);

task automatic step();
    @(posedge clk); #1;
endtask

task automatic reset();
    @(negedge clk);
    rst = 0; progress = '0; dispatch = '0; done = '0; fenced = '0;
    busy = '0; classes = '0; interval = '0;
    ctrl = (100 << 12) | (1 << 8) | (2 << 5) | (1 << 2) | 1;
    late_ctrl = 0;
    h_eff = {3{32'd1000}}; c_eff = {3{32'd2000}};
    step();
    @(negedge clk); rst = 1;
    step();
endtask

task automatic start(input int cls = 1, load = 1);
    @(negedge clk); busy = '0; progress = '0; done = '0; dispatch = '0;
    busy[0][0] = 1; busy[1][0] = load == 2;
    classes[0][0] = 3'(cls); dispatch[0][0] = 1;
    step();
    @(negedge clk); dispatch = '0;
endtask

task automatic complete(input logic [31:0] value);
    @(negedge clk);
    interval[0][0] = value; progress[0][0] = 1; done[0][0] = 1;
    step();
    @(negedge clk); progress = '0; done = '0; busy = '0;
    #1;
endtask

task automatic sample(input logic [31:0] value, input int cls = 1, load = 1);
    start(cls, load);
    complete(value);
endtask

task automatic expect_late(input bit valid, input int threshold = 0);
    #1;
    if (lv[0][0] !== valid || (valid && late_ad[0][0] != threshold))
        $fatal(1, "[P9_UNIT] late valid=%0b threshold=%0d, expected %0b/%0d",
               lv[0][0], late_ad[0][0], valid, threshold);
endtask

task automatic expect_entry(input int maximum, samples, load);
    if (dut.table_q[1][1].maximum != maximum || dut.table_q[1][1].samples != samples ||
        dut.table_q[1][1].load != load)
        $fatal(1, "[P9_UNIT] M/N/L changed unexpectedly");
endtask

initial begin
    #100000;
    $fatal(1, "[P9_UNIT] timeout");
end
