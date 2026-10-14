// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Power-domain counters: for each counted power domain, the manager cycles it
// spends at each clock level, and the cycles in which one of its slots is
// active (bingo_hw_manager_pkg PmcIdle .. PmcActive). Observation only:
// nothing here feeds back into the PM or the control plane.
//
// The levels are clock dividers (bingo_hw_manager_pm). The level of a domain
// in a cycle is the one the DFS path last applied to it (pm_i.domain_level),
// or, in DVFS mode, the chip-wide level the host acknowledged, as for the
// frequency-aware watchdog (bingo_hw_manager_wd_tick). Level 0 (never set) is in
// no class. Classes, in this order when configured levels are equal: idle,
// normal, boost (if not 0), other (every other level: the access servo level,
// a derate level, or a level the host changed meanwhile).
//
// A slot is active while it is neither waiting for a task, nor exited, nor
// fenced (its core runs a task, its runtime or its boot code); a domain is
// active while any of its slots is.
//
// With the levels it configured, the host turns the counts into a relative
// energy estimate (the proxy of the evaluation, no technology library): at a
// fixed voltage the dynamic energy of a domain is proportional to its clock
// cycles, i.e. to the sum over the classes of cycles x normal / level, and its
// leakage to the cycles it is powered. Awake cycles (not at the idle level)
// without an active slot are the cost of keeping the domain ready: the idle
// entry delay, the external access wake, the recovery hold.
//
// The counters wrap at 2^CounterWidth; the host reads them before and after a
// window and subtracts.
module bingo_hw_manager_pm_counters
    import bingo_hw_manager_pkg::*;
#(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    // Domains FirstDomain .. FirstDomain + NumDomains - 1 are counted
    parameter int unsigned FirstDomain = 0,
    parameter int unsigned NumDomains = 1,
    parameter int unsigned CounterWidth = 32
) (
    input  logic                                      clk_i,
    input  logic                                      rst_ni,
    // Power state: the level of each domain (DFS) or of the chip (DVFS), and
    // the configured idle, normal and boost levels
    input  pm_view_t                                  pm_i,
    // Domain of each slot (>= 32: none) and its activity
    input  logic [NumCores-1:0][NumClusters-1:0][5:0] slot_domain_i,
    input  logic [NumCores-1:0][NumClusters-1:0]      slot_active_i,
    output logic [NumDomains-1:0][PmCounters-1:0][CounterWidth-1:0] count_o
);
    if ((NumDomains == 0) || (FirstDomain + NumDomains > PmMaxDomains)) begin : gen_invalid
        initial $fatal(1, "Power-domain counters: domains %0d .. %0d out of 0 .. %0d",
                       FirstDomain, FirstDomain + NumDomains - 1, PmMaxDomains - 1);
    end

    logic [NumDomains-1:0][PmCounters-1:0] inc;
    always_comb begin
        inc = '0;
        for (int unsigned d = 0; d < NumDomains; d++) begin
            automatic logic [7:0] level = pm_i.dvfs ? pm_i.dvfs_level : pm_i.domain_level[FirstDomain + d];
            automatic logic idle   = (level != '0) && (level == pm_i.idle_level);
            automatic logic normal = (level != '0) && !idle && (level == pm_i.normal_level);
            automatic logic boost  = (level != '0) && !idle && !normal &&
                                     (pm_i.boost_level != '0) && (level == pm_i.boost_level);
            inc[d][PmcIdle]   = idle;
            inc[d][PmcNormal] = normal;
            inc[d][PmcBoost]  = boost;
            inc[d][PmcOther]  = (level != '0) && !idle && !normal && !boost;
            for (int unsigned c = 0; c < NumCores; c++)
                for (int unsigned cl = 0; cl < NumClusters; cl++)
                    if (slot_active_i[c][cl] && (slot_domain_i[c][cl] == 6'(FirstDomain + d)))
                        inc[d][PmcActive] = 1'b1;
        end
    end

    logic [NumDomains-1:0][PmCounters-1:0][CounterWidth-1:0] count_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            count_q <= '0;
        end else begin
            for (int unsigned d = 0; d < NumDomains; d++)
                for (int unsigned k = 0; k < PmCounters; k++)
                    if (inc[d][k]) count_q[d][k] <= count_q[d][k] + 1'b1;
        end
    end
    assign count_o = count_q;
endmodule
