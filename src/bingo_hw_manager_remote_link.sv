// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Level 3 transport: carries the remote dispatch / remote done streams of
// bingo_hw_manager_top between chiplets over AXI-Lite posted writes (the same
// path as the chiplet dep set: one 64-bit write per message).
//
// TX: one AXI-Lite master. A done of an imported task (to its origin) has
//     priority over an export. Single outstanding write: AW + W, then B.
//     Export destination: static table RemoteTargetChip[core type]
//     ({valid, chip id}); an invalid entry is never exported (ready held low).
//     target_valid_o tells bingo which types it may export at all.
//     Done destination: done_out_chip_i (origin of the imported task). A
//     reject (done_out_reject_i: the task cannot run there) travels as a done.
// RX: one AXI-Lite slave spanning two 4 KiB pages at base_addr_i:
//     page 0 (+0x0000) dispatch mailbox, page 1 (+0x1000) done mailbox. Each is
//     a bingo_hw_manager_write_mailbox (write register at offset 0). Its other
//     registers are not writable from the link (SLVERR), so a peer cannot
//     flush a mailbox (CTRL) and lose packets and credits.
//
// Packets (64 bit, one per write):
//   [63:60] kind      DISPATCH = 4'h5, DONE = 4'hA, REJECT = 4'hC (done page)
//   [59:52] chip      DISPATCH: origin chip, DONE / REJECT: executing (sending) chip
//   [51:44] proxy slot on the origin chip
//   [43:40] core type (DISPATCH only)
//   [39:32] seq       per (sender, receiver, stream), wraps
//   [31:30] task type (DISPATCH only)
//   [29: 0] task id   (TaskIdWidth bits, zero extended)
//
// Flow control: an export to peer p takes one of DispatchCredits credits of p;
// the credit comes back when the done (or reject) of that task leaves the done
// FIFO here (delivered to bingo). A mailbox write to a full FIFO would be dropped with
// SLVERR, so the FIFOs are sized to never be full when a write arrives:
//   outstanding exports of a sender to one receiver       <= DispatchCredits
//   senders of a receiver, receivers of a sender           <= NumPeers
//   => DispatchFifoDepth >= NumPeers * DispatchCredits  (all peers export here)
//   => DoneFifoDepth     >= NumPeers * DispatchCredits  (dones of own exports)
// (every chiplet of the system uses the same DispatchCredits, and PeerChipId
// lists every chiplet this one exchanges messages with, in either direction).
//
// Errors (sticky, error_o):
//   [0] SLVERR / DECERR on a write           [1] wrong kind on a page (dropped)
//   [2] seq gap from a peer                  [3] unknown peer chip
//   [4] done from a peer without an outstanding export (credit overflow)
`include "common_cells/registers.svh"
`include "axi/typedef.svh"
module bingo_hw_manager_remote_link #(
    parameter int unsigned ChipIdWidth       = 8,
    parameter int unsigned CoreTypeIdWidth   = 4,
    parameter int unsigned RemoteSlotIdWidth = 2,
    parameter int unsigned TaskIdWidth       = 12,
    parameter int unsigned AxiAddrWidth      = 48,
    parameter int unsigned AxiDataWidth      = 64,
    // Chiplets this one exchanges messages with
    parameter int unsigned NumPeers          = 1,
    parameter logic [NumPeers-1:0][ChipIdWidth-1:0] PeerChipId = '0,
    // Export destination per core type: {valid, chip id}
    parameter int unsigned NumCoreTypes      = 2**CoreTypeIdWidth,
    parameter logic [NumCoreTypes-1:0][ChipIdWidth:0] RemoteTargetChip = '0,
    // Credits per peer, and the RX FIFO depths (see the sizing rule above)
    parameter int unsigned DispatchCredits   = 2,
    parameter int unsigned DispatchFifoDepth = (NumPeers * DispatchCredits < 2) ? 2 : NumPeers * DispatchCredits,
    parameter int unsigned DoneFifoDepth     = (NumPeers * DispatchCredits < 2) ? 2 : NumPeers * DispatchCredits,
    // Position of task_type / task_id in bingo_hw_manager_task_desc_full_t
    // (below them: cond_exec_en, cond_exec_group_id[4:0], cond_exec_invert).
    // bingo_hw_manager_top checks its layout against these defaults.
    parameter int unsigned DescTaskTypeLsb   = 7,
    parameter int unsigned DescTaskIdLsb     = 9,
    parameter type         req_t             = logic,
    parameter type         resp_t            = logic,
    // DEPENDENT PARAMETERS, DO NOT OVERRIDE!
    parameter type         addr_t            = logic [AxiAddrWidth-1:0],
    parameter type         data_t            = logic [AxiDataWidth-1:0],
    parameter type         chip_id_t         = logic [ChipIdWidth-1:0],
    parameter type         slot_t            = logic [RemoteSlotIdWidth-1:0],
    parameter type         task_id_t         = logic [TaskIdWidth-1:0],
    parameter type         core_type_t       = logic [CoreTypeIdWidth-1:0],
    parameter int unsigned CreditWidth       = $clog2(DispatchCredits + 1)
) (
    input  logic                                   clk_i,
    input  logic                                   rst_ni,
    input  chip_id_t                               chip_id_i,
    // Base of the 8 KiB link region (same offset on every chiplet; the chip id
    // bits are replaced by the destination / own chip id)
    input  addr_t                                  base_addr_i,
    // bingo -> link: export (remote_dispatch_*_o of bingo_hw_manager_top)
    input  logic                                   export_valid_i,
    output logic                                   export_ready_o,
    input  data_t                                  export_desc_i,
    input  core_type_t                             export_core_type_i,
    input  chip_id_t                               export_origin_chip_i,
    input  slot_t                                  export_proxy_slot_i,
    // bingo -> link: done of an imported task (remote_done_*_o)
    input  logic                                   done_out_valid_i,
    output logic                                   done_out_ready_o,
    input  chip_id_t                               done_out_chip_i,
    input  slot_t                                  done_out_proxy_slot_i,
    input  task_id_t                               done_out_task_id_i,
    input  logic                                   done_out_reject_i = 1'b0,
    // link -> bingo: import (remote_dispatch_*_i)
    output logic                                   import_valid_o,
    input  logic                                   import_ready_i,
    output data_t                                  import_desc_o,
    output core_type_t                             import_core_type_o,
    output chip_id_t                               import_origin_chip_o,
    output slot_t                                  import_proxy_slot_o,
    // link -> bingo: done of an exported task (remote_done_*_i)
    output logic                                   done_in_valid_o,
    input  logic                                   done_in_ready_i,
    output slot_t                                  done_in_proxy_slot_o,
    output task_id_t                               done_in_task_id_o,
    output logic                                   done_in_reject_o,
    // AXI-Lite
    output req_t                                   mst_req_o,
    input  resp_t                                  mst_resp_i,
    input  req_t                                   slv_req_i,
    output resp_t                                  slv_resp_o,
    // Core types with a valid RemoteTargetChip entry (to bingo_hw_manager_top
    // remote_export_type_en_i: the other types are never exported)
    output logic [NumCoreTypes-1:0]                target_valid_o,
    // Status
    output logic [4:0]                             error_o,
    output logic [NumPeers-1:0][CreditWidth-1:0]   credits_o
);
    localparam logic [3:0] KindDispatch = 4'h5;
    localparam logic [3:0] KindDone     = 4'hA;
    localparam logic [3:0] KindReject   = 4'hC;
    localparam int unsigned PageOffset  = 32'h1000;
    localparam int unsigned PeerIdxWidth = (NumPeers > 1) ? $clog2(NumPeers) : 1;
    typedef logic [PeerIdxWidth-1:0] peer_idx_t;
    typedef logic [7:0]              seq_t;
    typedef logic [AxiDataWidth/8-1:0] strb_t;

    typedef struct packed {
        logic [3:0]  kind;
        logic [7:0]  chip;
        logic [7:0]  slot;
        logic [3:0]  core_type;
        seq_t        seq;
        logic [1:0]  task_type;
        logic [29:0] task_id;
    } pkt_t;

    // ------------------------------------------------------------------
    // Elaboration checks
    // ------------------------------------------------------------------
    if (AxiDataWidth != 64) begin : gen_err_dw
        $fatal(1, "remote_link: AxiDataWidth must be 64 (got %0d)", AxiDataWidth);
    end
    if ((ChipIdWidth > 8) || (RemoteSlotIdWidth > 8) || (CoreTypeIdWidth > 4) || (TaskIdWidth > 30)) begin : gen_err_fields
        $fatal(1, "remote_link: a field does not fit the 64-bit packet");
    end
    if (DispatchCredits < 1) begin : gen_err_credits
        $fatal(1, "remote_link: DispatchCredits must be >= 1");
    end
    // Credit sizing rule: the RX FIFOs never see a write while full
    if (DispatchFifoDepth < NumPeers * DispatchCredits) begin : gen_err_disp_depth
        $fatal(1, "remote_link: DispatchFifoDepth (%0d) < NumPeers * DispatchCredits (%0d)",
               DispatchFifoDepth, NumPeers * DispatchCredits);
    end
    if (DoneFifoDepth < NumPeers * DispatchCredits) begin : gen_err_done_depth
        $fatal(1, "remote_link: DoneFifoDepth (%0d) < NumPeers * DispatchCredits (%0d)",
               DoneFifoDepth, NumPeers * DispatchCredits);
    end
    if ((DispatchFifoDepth < 2) || (DoneFifoDepth < 2)) begin : gen_err_min_depth
        $fatal(1, "remote_link: mailbox FIFO depths must be >= 2");
    end

    function automatic int unsigned peer_of(input chip_id_t c);
        for (int unsigned p = 0; p < NumPeers; p++) begin
            if (PeerChipId[p] == c) return p;
        end
        return NumPeers;
    endfunction

    for (genvar t = 0; t < NumCoreTypes; t++) begin : gen_check_target
        if (RemoteTargetChip[t][ChipIdWidth] &&
            (peer_of(chip_id_t'(RemoteTargetChip[t][ChipIdWidth-1:0])) >= NumPeers)) begin : gen_err_target
            $fatal(1, "remote_link: RemoteTargetChip[%0d] = %0d is not in PeerChipId", t,
                   RemoteTargetChip[t][ChipIdWidth-1:0]);
        end
    end

    for (genvar t = 0; t < NumCoreTypes; t++) begin : gen_target_valid
        assign target_valid_o[t] = RemoteTargetChip[t][ChipIdWidth];
    end

    // Runtime peer lookup
    logic      tx_done_peer_ok, tx_exp_valid;
    peer_idx_t tx_done_peer, tx_exp_peer;
    chip_id_t  tx_exp_chip;
    always_comb begin
        tx_done_peer_ok = 1'b0;
        tx_done_peer    = '0;
        for (int unsigned p = 0; p < NumPeers; p++) begin
            if (PeerChipId[p] == done_out_chip_i) begin
                tx_done_peer_ok = 1'b1;
                tx_done_peer    = peer_idx_t'(p);
            end
        end
        tx_exp_valid = RemoteTargetChip[export_core_type_i][ChipIdWidth];
        tx_exp_chip  = chip_id_t'(RemoteTargetChip[export_core_type_i][ChipIdWidth-1:0]);
        tx_exp_peer  = '0;
        for (int unsigned p = 0; p < NumPeers; p++) begin
            if (PeerChipId[p] == tx_exp_chip) tx_exp_peer = peer_idx_t'(p);
        end
    end

    // ------------------------------------------------------------------
    // Credits (per peer) and sequence numbers
    // ------------------------------------------------------------------
    logic [NumPeers-1:0][CreditWidth-1:0] credit_q, credit_d;
    seq_t [NumPeers-1:0] tx_seq_disp_q, tx_seq_disp_d, tx_seq_done_q, tx_seq_done_d;
    seq_t [NumPeers-1:0] rx_seq_disp_q, rx_seq_disp_d, rx_seq_done_q, rx_seq_done_d;
    logic [4:0] error_q, error_d;

    // ------------------------------------------------------------------
    // TX
    // ------------------------------------------------------------------
    typedef enum logic [1:0] { TxIdle, TxSend, TxWaitB } tx_state_e;
    tx_state_e tx_state_q, tx_state_d;
    pkt_t      tx_pkt_q, tx_pkt_d;
    chip_id_t  tx_dest_q, tx_dest_d;
    logic      tx_aw_done_q, tx_aw_done_d, tx_w_done_q, tx_w_done_d;
    logic      export_fire, credit_return;
    peer_idx_t credit_return_peer;

    // RX side signals used by the credit logic
    pkt_t      rx_done_pkt;
    logic      rx_done_pop, rx_done_deliver, rx_done_peer_ok;
    peer_idx_t rx_done_peer;

    assign export_fire = (tx_state_q == TxIdle) && !done_out_valid_i && export_valid_i &&
                         tx_exp_valid && (credit_q[tx_exp_peer] != '0);

    always_comb begin
        tx_state_d       = tx_state_q;
        tx_pkt_d         = tx_pkt_q;
        tx_dest_d        = tx_dest_q;
        tx_aw_done_d     = tx_aw_done_q;
        tx_w_done_d      = tx_w_done_q;
        tx_seq_disp_d    = tx_seq_disp_q;
        tx_seq_done_d    = tx_seq_done_q;
        done_out_ready_o = 1'b0;
        export_ready_o   = 1'b0;
        mst_req_o        = '0;
        unique case (tx_state_q)
            TxIdle: begin
                if (done_out_valid_i) begin
                    done_out_ready_o   = 1'b1;
                    tx_pkt_d           = '0;
                    tx_pkt_d.kind      = done_out_reject_i ? KindReject : KindDone;
                    tx_pkt_d.chip      = 8'(chip_id_i);
                    tx_pkt_d.slot      = 8'(done_out_proxy_slot_i);
                    tx_pkt_d.seq       = tx_done_peer_ok ? tx_seq_done_q[tx_done_peer] : '0;
                    tx_pkt_d.task_id   = 30'(done_out_task_id_i);
                    tx_dest_d          = done_out_chip_i;
                    if (tx_done_peer_ok) tx_seq_done_d[tx_done_peer] = tx_seq_done_q[tx_done_peer] + 8'd1;
                    tx_state_d         = TxSend;
                end else if (export_fire) begin
                    export_ready_o     = 1'b1;
                    tx_pkt_d           = '0;
                    tx_pkt_d.kind      = KindDispatch;
                    tx_pkt_d.chip      = 8'(export_origin_chip_i);
                    tx_pkt_d.slot      = 8'(export_proxy_slot_i);
                    tx_pkt_d.core_type = 4'(export_core_type_i);
                    tx_pkt_d.seq       = tx_seq_disp_q[tx_exp_peer];
                    tx_pkt_d.task_type = export_desc_i[DescTaskTypeLsb +: 2];
                    tx_pkt_d.task_id   = 30'(export_desc_i[DescTaskIdLsb +: TaskIdWidth]);
                    tx_dest_d          = tx_exp_chip;
                    tx_seq_disp_d[tx_exp_peer] = tx_seq_disp_q[tx_exp_peer] + 8'd1;
                    tx_state_d         = TxSend;
                end
                tx_aw_done_d = 1'b0;
                tx_w_done_d  = 1'b0;
            end
            TxSend: begin
                mst_req_o.aw_valid = !tx_aw_done_q;
                mst_req_o.aw.addr  = {tx_dest_q, base_addr_i[AxiAddrWidth-ChipIdWidth-1:0]} +
                                     ((tx_pkt_q.kind != KindDispatch) ? addr_t'(PageOffset) : addr_t'(0));
                mst_req_o.aw.prot  = '0;
                mst_req_o.w_valid  = !tx_w_done_q;
                mst_req_o.w.data   = data_t'(tx_pkt_q);
                mst_req_o.w.strb   = '1;
                if (mst_resp_i.aw_ready) tx_aw_done_d = 1'b1;
                if (mst_resp_i.w_ready)  tx_w_done_d  = 1'b1;
                if (tx_aw_done_d && tx_w_done_d) tx_state_d = TxWaitB;
            end
            TxWaitB: begin
                mst_req_o.b_ready = 1'b1;
                if (mst_resp_i.b_valid) tx_state_d = TxIdle;
            end
            default: tx_state_d = TxIdle;
        endcase
    end

    `FF(tx_state_q,    tx_state_d,    TxIdle, clk_i, rst_ni)
    `FF(tx_pkt_q,      tx_pkt_d,      '0,     clk_i, rst_ni)
    `FF(tx_dest_q,     tx_dest_d,     '0,     clk_i, rst_ni)
    `FF(tx_aw_done_q,  tx_aw_done_d,  1'b0,   clk_i, rst_ni)
    `FF(tx_w_done_q,   tx_w_done_d,   1'b0,   clk_i, rst_ni)
    `FF(tx_seq_disp_q, tx_seq_disp_d, '0,     clk_i, rst_ni)
    `FF(tx_seq_done_q, tx_seq_done_d, '0,     clk_i, rst_ni)

    // Credits: taken on an export, returned when the done is delivered to bingo
    assign credit_return      = rx_done_deliver && rx_done_peer_ok;
    assign credit_return_peer = rx_done_peer;
    always_comb begin
        credit_d = credit_q;
        if (export_fire) credit_d[tx_exp_peer] = credit_d[tx_exp_peer] - CreditWidth'(1);
        if (credit_return && (credit_d[credit_return_peer] != CreditWidth'(DispatchCredits))) begin
            credit_d[credit_return_peer] = credit_d[credit_return_peer] + CreditWidth'(1);
        end
    end
    logic [NumPeers-1:0][CreditWidth-1:0] credit_rst;
    for (genvar p = 0; p < NumPeers; p++) begin : gen_credit_rst
        assign credit_rst[p] = CreditWidth'(DispatchCredits);
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) credit_q <= credit_rst;
        else         credit_q <= credit_d;
    end
    assign credits_o = credit_q;

    // ------------------------------------------------------------------
    // RX: two write mailboxes behind a demux on address bit 12
    // ------------------------------------------------------------------
    `AXI_LITE_TYPEDEF_AW_CHAN_T(aw_chan_t, addr_t)
    `AXI_LITE_TYPEDEF_W_CHAN_T(w_chan_t, data_t, strb_t)
    `AXI_LITE_TYPEDEF_B_CHAN_T(b_chan_t)
    `AXI_LITE_TYPEDEF_AR_CHAN_T(ar_chan_t, addr_t)
    `AXI_LITE_TYPEDEF_R_CHAN_T(r_chan_t, data_t)

    req_t  [1:0] page_req;
    resp_t [1:0] page_resp;
    axi_lite_demux #(
        .aw_chan_t  ( aw_chan_t ),
        .w_chan_t   ( w_chan_t  ),
        .b_chan_t   ( b_chan_t  ),
        .ar_chan_t  ( ar_chan_t ),
        .r_chan_t   ( r_chan_t  ),
        .axi_req_t  ( req_t     ),
        .axi_resp_t ( resp_t    ),
        .NoMstPorts ( 2         ),
        .MaxTrans   ( 2         ),
        .FallThrough( 1'b0      ),
        .SpillAw    ( 1'b1      ),
        .SpillW     ( 1'b0      ),
        .SpillB     ( 1'b0      ),
        .SpillAr    ( 1'b1      ),
        .SpillR     ( 1'b0      )
    ) i_page_demux (
        .clk_i,
        .rst_ni,
        .test_i          ( 1'b0                  ),
        .slv_req_i       ( slv_req_i             ),
        .slv_aw_select_i ( slv_req_i.aw.addr[12] ),
        .slv_ar_select_i ( slv_req_i.ar.addr[12] ),
        .slv_resp_o      ( slv_resp_o            ),
        .mst_reqs_o      ( page_req              ),
        .mst_resps_i     ( page_resp             )
    );

    data_t rx_disp_data, rx_done_data;
    logic  rx_disp_empty, rx_done_empty, rx_disp_pop;
    bingo_hw_manager_write_mailbox #(
        .MailboxDepth ( DispatchFifoDepth ),
        .IrqEdgeTrig  ( 1'b0              ),
        .IrqActHigh   ( 1'b1              ),
        .AxiAddrWidth ( AxiAddrWidth      ),
        .AxiDataWidth ( AxiDataWidth      ),
        .ChipIdWidth  ( ChipIdWidth       ),
        .RegWriteEn   ( 1'b0              ),
        .req_lite_t   ( req_t             ),
        .resp_lite_t  ( resp_t            )
    ) i_dispatch_mailbox (
        .clk_i,
        .rst_ni,
        .test_i       ( 1'b0          ),
        .chip_id_i    ( chip_id_i     ),
        .req_i        ( page_req[0]   ),
        .resp_o       ( page_resp[0]  ),
        .irq_o        ( /* unused */  ),
        .base_addr_i  ( base_addr_i   ),
        .mbox_data_o  ( rx_disp_data  ),
        .mbox_pop_i   ( rx_disp_pop   ),
        .mbox_empty_o ( rx_disp_empty ),
        .mbox_flush_i ( 1'b0          )
    );
    bingo_hw_manager_write_mailbox #(
        .MailboxDepth ( DoneFifoDepth     ),
        .IrqEdgeTrig  ( 1'b0              ),
        .IrqActHigh   ( 1'b1              ),
        .AxiAddrWidth ( AxiAddrWidth      ),
        .AxiDataWidth ( AxiDataWidth      ),
        .ChipIdWidth  ( ChipIdWidth       ),
        .RegWriteEn   ( 1'b0              ),
        .req_lite_t   ( req_t             ),
        .resp_lite_t  ( resp_t            )
    ) i_done_mailbox (
        .clk_i,
        .rst_ni,
        .test_i       ( 1'b0                           ),
        .chip_id_i    ( chip_id_i                      ),
        .req_i        ( page_req[1]                    ),
        .resp_o       ( page_resp[1]                   ),
        .irq_o        ( /* unused */                   ),
        .base_addr_i  ( base_addr_i + addr_t'(PageOffset) ),
        .mbox_data_o  ( rx_done_data                   ),
        .mbox_pop_i   ( rx_done_pop                    ),
        .mbox_empty_o ( rx_done_empty                  ),
        .mbox_flush_i ( 1'b0                           )
    );

    // Dispatch page: unpack, a wrong kind is dropped
    pkt_t      rx_disp_pkt;
    logic      rx_disp_kind_ok, rx_disp_peer_ok, rx_disp_deliver;
    peer_idx_t rx_disp_peer;
    assign rx_disp_pkt     = pkt_t'(rx_disp_data);
    assign rx_disp_kind_ok = (rx_disp_pkt.kind == KindDispatch);
    always_comb begin
        rx_disp_peer_ok = 1'b0;
        rx_disp_peer    = '0;
        for (int unsigned p = 0; p < NumPeers; p++) begin
            if (PeerChipId[p] == chip_id_t'(rx_disp_pkt.chip)) begin
                rx_disp_peer_ok = 1'b1;
                rx_disp_peer    = peer_idx_t'(p);
            end
        end
    end
    assign import_valid_o       = !rx_disp_empty && rx_disp_kind_ok;
    assign rx_disp_deliver      = import_valid_o && import_ready_i;
    assign rx_disp_pop          = rx_disp_deliver || (!rx_disp_empty && !rx_disp_kind_ok);
    assign import_core_type_o   = core_type_t'(rx_disp_pkt.core_type);
    assign import_origin_chip_o = chip_id_t'(rx_disp_pkt.chip);
    assign import_proxy_slot_o  = slot_t'(rx_disp_pkt.slot);
    always_comb begin
        import_desc_o = '0;
        import_desc_o[DescTaskTypeLsb +: 2]         = rx_disp_pkt.task_type;
        import_desc_o[DescTaskIdLsb +: TaskIdWidth] = task_id_t'(rx_disp_pkt.task_id);
    end

    // Done page
    logic rx_done_kind_ok;
    assign rx_done_pkt     = pkt_t'(rx_done_data);
    assign rx_done_kind_ok = (rx_done_pkt.kind == KindDone) || (rx_done_pkt.kind == KindReject);
    always_comb begin
        rx_done_peer_ok = 1'b0;
        rx_done_peer    = '0;
        for (int unsigned p = 0; p < NumPeers; p++) begin
            if (PeerChipId[p] == chip_id_t'(rx_done_pkt.chip)) begin
                rx_done_peer_ok = 1'b1;
                rx_done_peer    = peer_idx_t'(p);
            end
        end
    end
    assign done_in_valid_o      = !rx_done_empty && rx_done_kind_ok;
    assign rx_done_deliver      = done_in_valid_o && done_in_ready_i;
    assign rx_done_pop          = rx_done_deliver || (!rx_done_empty && !rx_done_kind_ok);
    assign done_in_proxy_slot_o = slot_t'(rx_done_pkt.slot);
    assign done_in_task_id_o    = task_id_t'(rx_done_pkt.task_id);
    assign done_in_reject_o     = (rx_done_pkt.kind == KindReject);

    // Sequence checks (on delivery) and sticky errors
    always_comb begin
        rx_seq_disp_d = rx_seq_disp_q;
        rx_seq_done_d = rx_seq_done_q;
        error_d       = error_q;
        if ((tx_state_q == TxWaitB) && mst_resp_i.b_valid && (mst_resp_i.b.resp != axi_pkg::RESP_OKAY)) begin
            error_d[0] = 1'b1;
        end
        if ((!rx_disp_empty && !rx_disp_kind_ok) || (!rx_done_empty && !rx_done_kind_ok)) error_d[1] = 1'b1;
        if (rx_disp_deliver) begin
            if (!rx_disp_peer_ok) begin
                error_d[3] = 1'b1;
            end else begin
                if (rx_disp_pkt.seq != rx_seq_disp_q[rx_disp_peer]) error_d[2] = 1'b1;
                rx_seq_disp_d[rx_disp_peer] = rx_disp_pkt.seq + 8'd1;
            end
        end
        if (rx_done_deliver) begin
            if (!rx_done_peer_ok) begin
                error_d[3] = 1'b1;
            end else begin
                if (rx_done_pkt.seq != rx_seq_done_q[rx_done_peer]) error_d[2] = 1'b1;
                rx_seq_done_d[rx_done_peer] = rx_done_pkt.seq + 8'd1;
                // credit_d was already incremented by an export of this cycle
                if ((credit_q[rx_done_peer] == CreditWidth'(DispatchCredits)) &&
                    !(export_fire && (tx_exp_peer == rx_done_peer))) begin
                    error_d[4] = 1'b1;
                end
            end
        end
        if ((tx_state_q == TxIdle) && done_out_valid_i && !tx_done_peer_ok) error_d[3] = 1'b1;
    end
    `FF(rx_seq_disp_q, rx_seq_disp_d, '0, clk_i, rst_ni)
    `FF(rx_seq_done_q, rx_seq_done_d, '0, clk_i, rst_ni)
    `FF(error_q,       error_d,       '0, clk_i, rst_ni)
    assign error_o = error_q;

`ifndef SYNTHESIS
    initial begin
        @(posedge rst_ni);
        if (base_addr_i[12:0] != '0) $error("[REMOTE_LINK] base_addr_i %h is not 8 KiB aligned", base_addr_i);
    end
    always @(posedge clk_i) begin
        if (rst_ni && (error_d != error_q)) begin
            $display("[REMOTE_LINK] %0t chip=%0d error %b -> %b", $time, chip_id_i, error_q, error_d);
        end
    end
`endif
endmodule
