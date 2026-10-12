// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Constants that more than one bingo_hw_manager module, or the host software,
// relies on: the task types of a descriptor, the device CSRs of the ready and
// done queues, and the number of power domains.
package bingo_hw_manager_pkg;
    // Task type of a descriptor (two bits, written by the host compiler bingo_dfg.py)
    localparam logic [1:0] TaskNormal   = 2'b00; // runs on its core
    localparam logic [1:0] TaskDummy    = 2'b01; // dependency set / check only: retires in the manager, never dispatched
    localparam logic [1:0] TaskGating   = 2'b10; // runs, then writes the CERF groups it selected
    localparam logic [1:0] TaskNoReplay = 2'b11; // runs like a normal task, but is never replayed once it may
                                                 // have started (it is not idempotent)
    // Device CSRs of the ready and done queues (a core reads its next task / writes its done)
    localparam logic [11:0] CsrReady = 12'h5fe;
    localparam logic [11:0] CsrDone  = 12'h5ff;
    // Power domains 0 .. PmMaxDomains-1. A slot configured with a larger domain
    // (the host writes 99 for the host and tied-off slots) belongs to none.
    localparam int unsigned PmMaxDomains = 32;
endpackage
