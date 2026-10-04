// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Observation only. Event inputs are pulses on the same clock edge as the
// existing simulation prints. No output participates in scheduling or PM.
module bingo_hw_manager_evlog #(
    parameter int unsigned NumCores = 4,
    parameter int unsigned NumClusters = 2,
    parameter int unsigned NumTypes = 16,
    parameter int unsigned Depth = 32,
    parameter int unsigned AgeWidth = 6,
    localparam int unsigned NumSlots = NumCores * NumClusters,
    localparam int unsigned NumPending = 9 * NumSlots + NumTypes,
    localparam int unsigned PtrWidth = (Depth > 1) ? $clog2(Depth) : 1,
    localparam int unsigned CountWidth = $clog2(Depth + 1)
) (
    input logic clk_i,
    input logic rst_ni,
    input logic enable_i,
    input logic [31:0] clear_i,
    input logic [31:0] pop_i,
    // Slot groups in order: SUSPECT, CLEAR, FENCE, STUCK, BLOCKED,
    // RISK, PARKED, PARK_FAIL, RETIRED; then one CERF pulse per type.
    input logic [NumPending-1:0] event_i,
    input logic [NumPending-1:0][15:0] arg_i,
    input logic move_i,
    input logic [7:0] move_code_i,
    input logic [7:0] move_slot_i,
    input logic [15:0] move_arg_i,
    output logic [63:0] head_o,
    output logic [31:0] count_o,
    output logic [15:0] dropped_o
);
    if (Depth == 0 || (Depth & (Depth - 1)) != 0 ||
        NumSlots > 16 || NumCores > 16 || NumClusters > 16 ||
        NumTypes > 256 || AgeWidth == 0 || AgeWidth > 32) begin : gen_invalid
        initial $fatal(1, "Invalid bingo event log dimensions");
    end

    logic [31:0] timestamp_q, clear_q, pop_q;
    logic [NumPending-1:0] pending_q, pending_d;
    logic [NumPending-1:0][AgeWidth-1:0] age_q, age_d;
    logic [NumPending-1:0][15:0] arg_q, arg_d;
    logic [63:0] fifo_q [Depth];
    logic [PtrWidth-1:0] rd_q, wr_q;
    logic [CountWidth-1:0] count_q;
    logic [15:0] dropped_q, dropped_d;
    logic clear_fire, pop_fire, write_req, write_fire;
    logic [63:0] write_data;
    int selected, best_priority;
    int unsigned drops;

    function automatic logic [7:0] event_code(input int index);
        if (index >= 9 * NumSlots) return 8'h0a;
        case (index / NumSlots)
            0: return 8'h01;
            1: return 8'h02;
            2: return 8'h03;
            3: return 8'h05;
            4: return 8'h06;
            5: return 8'h07;
            6: return 8'h08;
            7: return 8'h09;
            default: return 8'h0b;
        endcase
    endfunction

    function automatic int priority_of(input int index);
        case (event_code(index))
            8'h03: return 0;
            8'h0a: return 1;
            8'h05: return 2;
            8'h06: return 3;
            default: return 4 + int'(event_code(index));
        endcase
    endfunction

    function automatic logic [7:0] event_slot(input int index);
        int slot;
        if (index >= 9 * NumSlots) return 8'(index - 9 * NumSlots);
        slot = index % NumSlots;
        return {4'(slot / NumCores), 4'(slot % NumCores)};
    endfunction

    assign clear_fire = clear_i != clear_q;
    assign pop_fire = (pop_i != pop_q) && count_q != 0 && !clear_fire;
    // A full FIFO drops the new item, even if a pop occurs on that edge.
    assign write_fire = write_req && count_q < Depth && !clear_fire;
    assign head_o = count_q != 0 ? fifo_q[rd_q] : 64'b0;
    assign count_o = 32'(count_q);
    assign dropped_o = dropped_q;

    always_comb begin
        pending_d = pending_q;
        age_d = age_q;
        arg_d = arg_q;
        drops = 0;
        selected = -1;
        best_priority = 1000;
        write_req = 1'b0;
        write_data = '0;
        for (int i = 0; i < NumPending; i++) begin
            // The age stored after the arrival edge is zero. Advance before
            // this edge's arbitration so a one-edge wait subtracts one.
            if (pending_q[i] && age_q[i] != '1) age_d[i] = age_q[i] + 1'b1;
            if (enable_i && event_i[i]) begin
                if (pending_q[i]) drops++;
                else begin
                    pending_d[i] = 1'b1;
                    age_d[i] = '0;
                    arg_d[i] = arg_i[i];
                end
            end
            if (pending_d[i] && priority_of(i) < best_priority) begin
                selected = i;
                best_priority = priority_of(i);
            end
        end
        if (enable_i) begin
            if (move_i) begin
                write_req = 1'b1;
                write_data = {timestamp_q, move_code_i, move_slot_i, move_arg_i};
            end else if (selected >= 0) begin
                write_req = 1'b1;
                write_data = {
                    timestamp_q - 32'(age_d[selected]),
                    event_code(selected) | ((age_d[selected] == '1) ? 8'h80 : 8'h00),
                    event_slot(selected), arg_d[selected]
                };
                pending_d[selected] = 1'b0;
                age_d[selected] = '0;
            end
            if (write_req && count_q == Depth) drops++;
        end
        dropped_d = (int'(dropped_q) + drops > 65535) ?
                    16'hffff : 16'(int'(dropped_q) + drops);
        if (clear_fire) begin
            pending_d = '0;
            age_d = '0;
            arg_d = '0;
            dropped_d = '0;
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            timestamp_q <= '0;
            clear_q <= '0;
            pop_q <= '0;
            pending_q <= '0;
            age_q <= '0;
            arg_q <= '0;
            rd_q <= '0;
            wr_q <= '0;
            count_q <= '0;
            dropped_q <= '0;
        end else begin
            timestamp_q <= timestamp_q + 32'd1;
            clear_q <= clear_i;
            pop_q <= pop_i;
            pending_q <= pending_d;
            age_q <= age_d;
            arg_q <= arg_d;
            dropped_q <= dropped_d;
            if (clear_fire) begin
                rd_q <= '0;
                wr_q <= '0;
                count_q <= '0;
            end else begin
                if (write_fire) begin
                    fifo_q[wr_q] <= write_data;
                    wr_q <= (wr_q == Depth - 1) ? '0 : wr_q + 1'b1;
                end
                if (pop_fire) rd_q <= (rd_q == Depth - 1) ? '0 : rd_q + 1'b1;
                case ({write_fire, pop_fire})
                    2'b10: count_q <= count_q + 1'b1;
                    2'b01: count_q <= count_q - 1'b1;
                    default: ;
                endcase
            end
        end
    end

`ifndef SYNTHESIS
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        count_q == Depth |-> !write_fire);
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        write_req && count_q == Depth && !clear_fire |-> dropped_d > dropped_q ||
                                                                  dropped_q == 16'hffff);
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        enable_i && !move_i && selected >= 0 && !clear_fire |->
            !pending_d[selected]);
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        !enable_i && !clear_fire && !pop_fire |=> $stable(count_q) && $stable(rd_q) &&
                                                  $stable(wr_q));
`endif
endmodule
