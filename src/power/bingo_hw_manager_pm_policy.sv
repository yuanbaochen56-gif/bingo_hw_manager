// Copyright 2025 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Fanchen Kong <fanchen.kong@kuleuven.be>
// - Xiaoling Yi  <xiaoling.yi@kuleuven.be>
// - Yunhao Deng  <yunhao.deng@kuleuven.be>

// Level policy of the power manager (bingo_hw_manager_pm), combinational:
//   DFS   the level every power domain should run at (idle, access servo,
//         boost, normal, at most the derate level), and the lowest domain
//         whose applied level (level_i) differs from it: the PM writes that
//         one next
//   DVFS  the one level of the chip (normal while any domain with a slot is
//         busy, else idle), and whether the host has not applied it yet
// A slot of no domain (slot_i.domain >= PmMaxDomains) is not counted.
module bingo_hw_manager_pm_policy #(
    parameter int unsigned NumCores = 8,
    parameter int unsigned NumClusters = 4,
    // Port type (bingo_hw_manager/typedef.svh)
    parameter type         pm_slot_t = logic,
    // Dependent parameters, DO NOT OVERRIDE!
    parameter int unsigned MAX_DOMAINS = bingo_hw_manager_pkg::PmMaxDomains,
    parameter type         domain_id_t = logic [$clog2(MAX_DOMAINS)-1:0]
) (
    input  bingo_hw_manager_pkg::pm_cfg_t cfg_i,
    input  pm_slot_t                      slot_i,
    input  logic [MAX_DOMAINS-1:0][7:0]   level_i,        // level applied to each domain (DFS)
    // DFS: a domain must change its level; the lowest one, and its new level
    output logic                          update_valid_o,
    output domain_id_t                    update_domain_o,
    output logic [7:0]                    update_level_o,
    // DVFS: a domain with a slot is busy; the chip level; the host has not applied it
    output logic                          chip_busy_o,
    output logic [7:0]                    chip_level_o,
    output logic                          dvfs_pending_o
);
    typedef logic [7:0] power_level_t;
    // -------------------------------------------------------------------------
    // 1. Status Aggregation
    // -------------------------------------------------------------------------
    logic [MAX_DOMAINS-1:0] domain_all_idle;    // 1 if ALL cores in domain are idle
    logic [MAX_DOMAINS-1:0] domain_access_only;
    logic [MAX_DOMAINS-1:0] domain_boost;       // 1 if a core of the domain is boosted
    logic [MAX_DOMAINS-1:0] domain_derate;      // 1 if a core of the domain is derated
    logic [MAX_DOMAINS-1:0] domain_active_mask; // 1 if at least one core is mapped to this domain
    domain_id_t  [NumCores-1:0][NumClusters-1:0] domain_id;

    // Example Scenario:
    // Setup:
    // - 2 Clusters, each with 2 Cores.
    // - Configuration:
    //   - C0_0 -> Domain 0
    //   - C0_1 -> Domain 0 (Shared)
    //   - C1_0 -> Domain 1
    //   - C1_1 -> Domain 2
    // - Status:
    //   - C0_0: Waiting (1)
    //   - C0_1: Active (0)
    //   - C1_0: Waiting (1)
    //   - C1_1: Waiting (1)
    //
    // Execution Trace:
    // 1. C0_0 (Domain 0): Idle. domain_all_idle[0] remains 1.
    // 2. C0_1 (Domain 0): Active. domain_all_idle[0] forced to 0. (Keeps Domain 0 awake).
    // 3. C1_0 (Domain 1): Idle. domain_all_idle[1] remains 1.
    // 4. C1_1 (Domain 2): Idle. domain_all_idle[2] remains 1.
    //
    // Result:
    // - domain_all_idle = ...11110 (Bit 0 is 0/Busy, Bits 1 & 2 are 1/Idle)
    // - Domain 0 -> Normal Power, Domains 1 & 2 -> Idle Power.
    always_comb begin
        domain_all_idle = '1; // Default to true, clear if any core busy
        domain_access_only = '1;
        domain_boost = '0;
        domain_derate = '0;
        domain_active_mask = '0;
        domain_id = '0;
        for (int core = 0; core < NumCores; core++) begin
            for (int cluster = 0; cluster < NumClusters; cluster++) begin
                // Some slots belong to no power domain (the host writes 99 for
                // them, the top passes MAX_DOMAINS)
                if (slot_i.domain[core][cluster] < MAX_DOMAINS) begin
                    domain_id[core][cluster] = slot_i.domain[core][cluster][4:0];
                    domain_active_mask[domain_id[core][cluster]] = 1'b1;
                    if (slot_i.idle[core][cluster] == 1'b0) begin
                        domain_all_idle[domain_id[core][cluster]] = 1'b0;
                        if (!slot_i.access_only[core][cluster])
                            domain_access_only[domain_id[core][cluster]] = 1'b0;
                    end
                    if (slot_i.boost[core][cluster]) begin
                        domain_boost[domain_id[core][cluster]] = 1'b1;
                    end
                    if (slot_i.derate[core][cluster]) begin
                        domain_derate[domain_id[core][cluster]] = 1'b1;
                    end
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 2. Change Detection & Arbitration
    // -------------------------------------------------------------------------
    power_level_t [MAX_DOMAINS-1:0] target_power_level;
    logic [MAX_DOMAINS-1:0]         pending_update;       // Domains that need update

    // Calculate target level and pending updates
    always_comb begin
        for (int d = 0; d < MAX_DOMAINS; d++) begin
            if (cfg_i.enable && domain_all_idle[d]) begin
                target_power_level[d] = cfg_i.idle_level;
            end else if (cfg_i.enable && !cfg_i.dvfs &&
                         domain_access_only[d] && cfg_i.access_level != 0) begin
                target_power_level[d] = cfg_i.access_level;
            end else if (cfg_i.enable && domain_boost[d] && (cfg_i.boost_level != '0)) begin
                target_power_level[d] = cfg_i.boost_level;
            end else begin
                target_power_level[d] = cfg_i.normal_level;
            end
            // Levels are clock dividers: the larger one is the slower clock
            if (domain_derate[d] && (cfg_i.derate_level > target_power_level[d])) begin
                target_power_level[d] = cfg_i.derate_level;
            end

            // We update if the domain exists AND the target differs from current
            if (domain_active_mask[d] && (target_power_level[d] != level_i[d])) begin
                pending_update[d] = 1'b1;
            end else begin
                pending_update[d] = 1'b0;
            end
        end
    end

    // Simple Priority Arbiter (Least Significant Bit First)
    lzc #(
        .WIDTH(MAX_DOMAINS),
        .MODE (1'b0) // 0 for Trailing Zeros (LSB)
    ) i_lzc_arbiter (
        .in_i    ( pending_update   ),
        .cnt_o   ( update_domain_o  ), // Returns the index of the first '1'
        .empty_o ( /* unused */     )
    );

    assign update_valid_o = |pending_update;
    assign update_level_o = target_power_level[update_domain_o];

    // -------------------------------------------------------------------------
    // 2b. DVFS chip-level aggregation
    // -------------------------------------------------------------------------
    // In DVFS mode the whole chiplet shares one PMIC, so we reduce the per-domain idle
    // status to a single chip-wide idle/busy signal. A domain with no core mapped to it
    // (domain_active_mask==0) does not keep the chip awake.
    logic         chip_all_idle;
    power_level_t acked_level;        // level the host reports it has applied (DVFS_ACK)

    assign chip_all_idle  = &(domain_all_idle | ~domain_active_mask);
    assign chip_busy_o    = ~chip_all_idle;
    assign chip_level_o   = chip_busy_o ? cfg_i.normal_level : cfg_i.idle_level;
    assign acked_level    = cfg_i.dvfs_ack;
    // Only meaningful when idle-PM is enabled and we are in DVFS mode.
    assign dvfs_pending_o = cfg_i.enable && cfg_i.dvfs &&
                            (chip_level_o != acked_level);

endmodule
