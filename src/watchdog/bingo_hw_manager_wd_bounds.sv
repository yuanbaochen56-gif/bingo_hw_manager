// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Per-slot watchdog threshold selector (combinational, in timer ticks).
//
// 1. Threshold sources
//   a) Parameters     HeartbeatTimeoutCycles and ConfirmTimeoutCycles (0: fencing
//                     off) give the default and maximum H and C.
//   b) Host static    Per-core-type H and C (type_h_i, type_c_i) and the global
//                     late-beat threshold (late_i -> late_global, 0: off).
//                     type_h_i applies if nonzero and <= HeartbeatTimeoutCycles,
//                     else the parameter -> h_static.
//                     type_c_i applies if fencing is on, nonzero, > h_static, and
//                     <= ConfirmTimeoutCycles, else the parameter -> c_static.
//                     h_static and c_static also cap bingo_hw_manager_wd_adapt.
//   c) At-risk CSR    Host confirm threshold R (risk_confirm_i) for an at-risk
//                     slot (risk_i, from bingo_hw_manager_ctrl); valid
//                     (risk_valid) when fencing is on and h_static < R < c_static.
//   d) Learned        Per-slot adaptive h, c, and late from
//                     bingo_hw_manager_wd_adapt (learned_i), each gated by its
//                     valid bit.
//
// 2. Per-slot selection (bound_o, compared by bingo_hw_manager_watchdog)
//   suspect   triggers dead_suspect 
//   confirm   triggers fenced
//   late      triggers a late-beat pulse on heartbeat/done (fault precursor that
//             increments the risk counter in bingo_hw_manager_risk):
//             learned_i.late when late_adaptive = 1 (late_valid and no longer
//             than late_global), else late_global (0: off)
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

    // Saturate the global late threshold to the counter width
    assign late_global = (late_i > data_t'({CounterWidth{1'b1}})) ?
                         {CounterWidth{1'b1}} : late_i[CounterWidth-1:0];

    for (genvar c = 0; c < NumCores; c++) begin : gen_core
        for (genvar cl = 0; cl < NumClusters; cl++) begin : gen_cluster
            localparam int Type = CoreTypeId[c][cl];
            logic        type_h_valid, type_c_valid;
            logic [31:0] non_learned_c;

            // 1. Static bounds: per-type CSR if valid, else parameter
            assign type_h_valid = (type_h_i[Type] != 0 &&
                                   type_h_i[Type] <= HeartbeatTimeoutCycles);
            assign h_static[c][cl] = type_h_valid ? type_h_i[Type] : HeartbeatTimeoutCycles;

            assign type_c_valid = (ConfirmTimeoutCycles != 0 &&
                                   type_c_i[Type] != 0 &&
                                   type_c_i[Type] > h_static[c][cl] &&
                                   type_c_i[Type] <= ConfirmTimeoutCycles);
            assign c_static[c][cl] = type_c_valid ? type_c_i[Type] : ConfirmTimeoutCycles;

            // 2. Suspect threshold: learned h (already capped at h_static) or h_static
            assign suspect[c][cl] = CounterWidth'(
                learned_i.h_valid[c][cl] ? learned_i.h[c][cl] : h_static[c][cl]);

            // 3. Confirm threshold: pick the shortest among learned c, risk R, and c_static
            assign risk_valid[c][cl] = (ConfirmTimeoutCycles != 0 &&
                                        risk_confirm_i > h_static[c][cl] &&
                                        risk_confirm_i < c_static[c][cl]);
            assign non_learned_c = (risk_i[c][cl] && risk_valid[c][cl]) ?
                                   risk_confirm_i : c_static[c][cl];
            assign adapt_confirm[c][cl] = learned_i.c_valid[c][cl] &&
                                          (learned_i.c[c][cl] < non_learned_c);
            assign risk_use[c][cl] = risk_valid[c][cl] && !adapt_confirm[c][cl];
            // 0 tells the watchdog to fall back to ConfirmTimeoutCycles (for logging)
            assign confirm[c][cl] = adapt_confirm[c][cl] ? CounterWidth'(learned_i.c[c][cl]) :
                                    type_c_valid         ? CounterWidth'(c_static[c][cl]) : '0;

            // 4. Late-beat threshold: learned late (if <= late_global) or late_global
            assign late_adaptive[c][cl] = learned_i.late_valid[c][cl] &&
                (late_global == '0 || learned_i.late[c][cl] <= 32'(late_global));
            assign late[c][cl] = late_adaptive[c][cl] ?
                CounterWidth'(learned_i.late[c][cl]) : late_global;
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
