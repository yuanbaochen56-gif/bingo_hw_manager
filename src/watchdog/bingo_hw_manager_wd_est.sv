// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Compile-time task-time estimates per (core type, task class), read per slot.
//
// 1. Table est_q[type][class] (type = CoreTypeId < 2**WdAdaptTypeBits, class 1 ..
//    2**WdClassWidth-1), 10 bits per entry, reset 0 (no estimate):
//      [7:0]  E_tot in timer ticks, 8-bit float: 0 = none, else (16 + m) << e with
//             e = [7:4] and m = [3:0] (17 .. 1015808 ticks; the compiler rounds up)
//      [9:8]  S: the longest interval without progress is E_int = E_tot >> S
//    A write pulse (we_i, BINGO_WD_EST) stores data_i[9:0] at type data_i[19:16] and
//    class data_i[14:12]; a type out of range or class 0 is ignored. The new value
//    is read from the next cycle.
//
// 2. Per slot (est_o): the entry of its core type and of the class of its task
//    (class_i, latched at dispatch). valid: a monitored slot (CoreMask), an
//    in-range type, class != 0 and an entry != 0.
//    bingo_hw_manager_wd_adapt uses E_int while the slot's class has no learned
//    bound yet (warm-up).
module bingo_hw_manager_wd_est #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    // The table covers core types 0 .. 2**WdAdaptTypeBits-1 (as the learned one)
    parameter int unsigned WdAdaptTypeBits = 2,
    parameter int unsigned WdClassWidth = 3,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0] CoreMask = '1,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId = '0,
    // Port type (bingo_hw_manager/typedef.svh)
    parameter type         wd_est_t = logic
) (
    input  logic                                                   clk_i,
    input  logic                                                   rst_ni,
    // BINGO_WD_EST: [9:0] entry, [14:12] class, [19:16] core type; we_i: one-cycle pulse
    input  logic                                                   we_i,
    input  logic [31:0]                                            data_i,
    input  logic [NumCores-1:0][NumClusters-1:0][WdClassWidth-1:0] class_i,
    output wd_est_t                                                est_o
);
    localparam int unsigned Types = 2**WdAdaptTypeBits;
    localparam int unsigned Classes = 2**WdClassWidth;

    if (WdAdaptTypeBits > 4 || WdClassWidth > 3) begin : gen_field_check
        $error("BINGO_WD_EST holds a 4-bit core type and a 3-bit class");
    end

    logic [Types-1:0][Classes-1:0][9:0] est_q;
    logic [WdAdaptTypeBits-1:0]         wr_type;
    logic [WdClassWidth-1:0]            wr_class;
    logic                               wr;

    assign wr_type  = data_i[16 +: WdAdaptTypeBits];
    assign wr_class = data_i[12 +: WdClassWidth];
    assign wr       = we_i && (4'(data_i[19:16]) >> WdAdaptTypeBits) == '0 && wr_class != '0;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            est_q <= '0;
        end else if (wr) begin
            est_q[wr_type][wr_class] <= data_i[9:0];
        end
    end

    for (genvar c = 0; c < NumCores; c++) begin : gen_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_cluster
            localparam int unsigned Type = CoreTypeId[c][cl];
            logic [9:0]  entry;
            logic [19:0] e_tot;
            if (CoreMask[c][cl] && Type < Types) begin : gen_read
                assign entry = est_q[Type][class_i[c][cl]];
            end else begin : gen_none
                assign entry = '0;
            end
            // (16 + m) << e
            assign e_tot = 20'({1'b1, entry[3:0]}) << entry[7:4];
            assign est_o.valid[c][cl] = class_i[c][cl] != '0 && entry[7:0] != '0;
            assign est_o.e_tot[c][cl] = est_o.valid[c][cl] ? e_tot : '0;
            assign est_o.e_int[c][cl] = est_o.valid[c][cl] ? e_tot >> entry[9:8] : '0;
        end
    end
endmodule
