// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// CERF degradation of the control plane (bingo_hw_manager_ctrl): a host table,
// one entry per core type, default off (type_cfg_i). A type is faulted while
// any of its slots is stuck or rejected. The lowest enabled faulted type that
// has not fired yet requests one CERF update (clear one group, set another).
// The register file applies it only in a cycle the host does not write the
// whole mask; the request stays up until cerf_fb_done_i. cerf_fb_evt_o marks
// types that have fired and drops when that enable drops, which arms the type
// again if it is still faulted. One fire per enable, so a later host write of
// the mask is left alone.
module bingo_hw_manager_cerf_fallback #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    // Type of each (core, cluster) slot (see bingo_hw_manager_top)
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter logic [NumCores-1:0][NumClusters-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NumCores * NumClusters){CoreTypeIdWidth'(1)}},
    // Port types (bingo_hw_manager/typedef.svh)
    parameter type         slot_state_t = logic,
    parameter type         type_cfg_t = logic,
    parameter type         cerf_fb_t = logic
) (
    input  logic clk_i,
    input  logic rst_ni,

    // Stuck (fenced, nowhere to replay) or rejected by a remote chiplet
    input  slot_state_t                              slot_i,
    // Per core type: enable, CERF group to clear, CERF group to set. 0 = off.
    input  type_cfg_t                                type_cfg_i,
    // The CERF update committed this cycle (host did not write the mask)
    input  logic                                     cerf_fb_done_i,

    // One CERF degradation, held until cerf_fb_done_i
    output cerf_fb_t                                 cerf_fb_o,
    // Types whose degradation has fired; sticky until that enable drops
    output logic [2**CoreTypeIdWidth-1:0]            cerf_fb_evt_o
);

    // One type per cycle, lowest index first
    localparam int unsigned NumTypes = 2 ** CoreTypeIdWidth;

    logic [NumTypes-1:0]            type_fault;
    logic [NumTypes-1:0]            pending;
    logic [NumTypes-1:0]            armed_q, armed_d;
    logic                           fb_req;
    logic [CoreTypeIdWidth-1:0]     fb_sel;
    logic [4:0]                     fb_clear, fb_set;

    always_comb begin
        type_fault = '0;
        for (int unsigned c = 0; c < NumCores; c++) begin
            for (int unsigned cl = 0; cl < NumClusters; cl++) begin
                if (slot_i.stuck[c][cl] || slot_i.rejected[c][cl]) begin
                    type_fault[CoreTypeId[c][cl]] = 1'b1;
                end
            end
        end
    end
    assign pending = type_cfg_i.cerf_fb_en & type_fault & ~armed_q;

    always_comb begin
        fb_req   = 1'b0;
        fb_sel   = '0;
        fb_clear = '0;
        fb_set   = '0;
        for (int unsigned t = 0; t < NumTypes; t++) begin
            if (pending[t] && !fb_req) begin
                fb_req   = 1'b1;
                fb_sel   = CoreTypeIdWidth'(t);
                fb_clear = type_cfg_i.cerf_fb_clear[t];
                fb_set   = type_cfg_i.cerf_fb_set[t];
            end
        end
    end
    assign cerf_fb_o.req       = fb_req;
    assign cerf_fb_o.core_type = fb_sel;
    assign cerf_fb_o.clear     = fb_clear;
    assign cerf_fb_o.set       = fb_set;
    assign cerf_fb_evt_o       = armed_q;

    always_comb begin
        armed_d = armed_q & type_cfg_i.cerf_fb_en;
        if (cerf_fb_done_i && fb_req && type_cfg_i.cerf_fb_en[fb_sel]) begin
            armed_d[fb_sel] = 1'b1;
        end
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) armed_q <= '0;
        else         armed_q <= armed_d;
    end

endmodule
