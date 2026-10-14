// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Thresholds of the watchdog (bingo_hw_manager_watchdog) for every slot, from
// four sources, in timer ticks:
//   parameters   HeartbeatTimeoutCycles (H), ConfirmTimeoutCycles (C)
//   core type    the host's H and C of the slot's core type (type_h_i,
//                type_c_i; A5); one that is 0 or does not fit below the
//                parameters keeps the parameter
//   learned      the suspect, confirm and late bounds of the slot's class
//                (bingo_hw_manager_wd_adapt), each used only when valid
//   risk         the host's confirm threshold R for an at-risk slot
//                (bingo_hw_manager_ctrl), valid only between the slot's H and C
// The choices:
//   suspect  learned, else the type's
//   confirm  learned if strictly shorter than the risk / type threshold that
//            would apply, else R for an at-risk slot with a valid R (applied by
//            the watchdog through risk_use), else the type's (0: the parameter)
//   late     learned if not longer than the host's late threshold, else the
//            host's (0: off)
// The type's H and C (h_static, c_static) are also the limits of the learning.
// Combinational; the watchdog only compares.
module bingo_hw_manager_wd_bounds #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned CounterWidth = 24,
    parameter int unsigned HeartbeatTimeoutCycles = 100000,
    parameter int unsigned ConfirmTimeoutCycles = 0,
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter int unsigned WdClassWidth = 3,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId = '0,
    parameter type         data_t = logic [31:0],   // host CSR word of the late threshold
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         wd_learned_t = logic,
    parameter type         wd_bound_t = logic
) (
    input  logic [2**CoreTypeIdWidth-1:0][31:0]                    type_h_i,
    input  logic [2**CoreTypeIdWidth-1:0][31:0]                    type_c_i,
    input  data_t                                                  late_i,          // host late threshold
    input  logic [31:0]                                            risk_confirm_i,  // host R
    input  logic [NumCores-1:0][NumClusters-1:0]                   risk_i,          // at-risk slots
    input  wd_learned_t                                            learned_i,
    input  logic [NumCores-1:0][NumClusters-1:0][WdClassWidth-1:0] class_i,         // passed on for the log
    output wd_bound_t                                              bound_o
);
    logic [CounterWidth-1:0] late_global;
    logic [NumCores-1:0][NumClusters-1:0][31:0]               h_static, c_static;
    logic [NumCores-1:0][NumClusters-1:0][CounterWidth-1:0]   suspect, confirm, late;
    logic [NumCores-1:0][NumClusters-1:0]                     late_adaptive, risk_valid, risk_use, adapt_confirm;

    // A threshold beyond the timer saturates it (the timer stops at all ones)
    assign late_global = (late_i > data_t'({CounterWidth{1'b1}})) ?
                         {CounterWidth{1'b1}} : late_i[CounterWidth-1:0];
    for (genvar c = 0; c < NumCores; c++) begin : gen_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_cluster
            localparam int Type = CoreTypeId[c][cl];
            logic [31:0] h_eff, c_eff;
            logic c_valid;
            assign h_eff = (type_h_i[Type] != 0 &&
                            type_h_i[Type] <= HeartbeatTimeoutCycles) ?
                            type_h_i[Type] : HeartbeatTimeoutCycles;
            assign c_valid = (ConfirmTimeoutCycles != 0 &&
                              type_c_i[Type] != 0 &&
                              type_c_i[Type] <= ConfirmTimeoutCycles &&
                              type_c_i[Type] > h_eff);
            assign c_eff = c_valid ? type_c_i[Type] : ConfirmTimeoutCycles;
            assign h_static[c][cl] = h_eff;
            assign c_static[c][cl] = c_eff;
            assign late_adaptive[c][cl] = learned_i.late_valid[c][cl] &&
                (late_global == '0 || learned_i.late[c][cl] <= 32'(late_global));
            assign late[c][cl] = late_adaptive[c][cl] ?
                CounterWidth'(learned_i.late[c][cl]) : late_global;
            assign suspect[c][cl] = CounterWidth'(
                learned_i.h_valid[c][cl] ? learned_i.h[c][cl] : h_eff);
            assign risk_valid[c][cl] = (ConfirmTimeoutCycles != 0 &&
                                        risk_confirm_i > h_eff &&
                                        risk_confirm_i < c_eff);
            // R keeps its A5 validity rule; C_ad wins only if strictly shorter.
            assign adapt_confirm[c][cl] = learned_i.c_valid[c][cl] &&
                learned_i.c[c][cl] < ((risk_i[c][cl] && risk_valid[c][cl]) ?
                                      risk_confirm_i : c_eff);
            assign risk_use[c][cl] = risk_valid[c][cl] && !adapt_confirm[c][cl];
            // Zero retains the parameter path and all existing logging.
            assign confirm[c][cl] = adapt_confirm[c][cl] ?
                CounterWidth'(learned_i.c[c][cl]) :
                c_valid ? CounterWidth'(c_eff) : '0;
        end
    end

    assign bound_o = '{late_global:   late_global,
                       risk_confirm:  CounterWidth'(risk_confirm_i),
                       h_static:      h_static,
                       c_static:      c_static,
                       suspect:       suspect,
                       suspect_valid: learned_i.h_valid,
                       confirm:       confirm,
                       late:          late,
                       late_adaptive: late_adaptive,
                       risk:          risk_i,
                       risk_valid:    risk_valid,
                       risk_use:      risk_use,
                       adapt_confirm: adapt_confirm,
                       wd_class:      class_i};
endmodule
