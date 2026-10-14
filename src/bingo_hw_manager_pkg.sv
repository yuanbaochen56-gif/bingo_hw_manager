// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Constants that more than one bingo_hw_manager module, or the host software,
// relies on: the task types of a descriptor, the device CSRs of the ready and
// done queues, the number of power domains, and the port structs of fixed
// width.
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

    // Port structs of fixed width. The ones whose width depends on the
    // parameters of the top are macros in bingo_hw_manager/typedef.svh.

    // Host configuration of the control plane (bingo_hw_manager_ctrl)
    typedef struct packed {
        logic [31:0] idle_delay;     // cycles a slot must be idle before the PM may count it as idle (0: at once)
        logic [31:0] access_hold;    // cycles a cluster's slots stay awake after an external access (0: off)
        logic [31:0] recovery_hold;  // cycles the substitute of a fenced slot stays awake (0: off)
        logic [3:0]  risk_thresh;    // late beats that make a slot at risk (0: off)
        logic [1:0]  risk_action;    // what an at-risk slot does: [0] park, [1] derate its domain
        logic [31:0] risk_epoch;     // cycles between two halvings of the late-beat counts (0: never)
        logic        boost_policy;   // 0: boost the substitutes of fenced cores, 1: capacity
        logic [3:0]  boost_credit;   // boosted domains per lost slot (0: no limit)
        logic [7:0]  boost_load_min; // minimum load of a capacity boost candidate
    } ctrl_cfg_t;

    // Power state as the control plane reads it (bingo_hw_manager_pm)
    typedef struct packed {
        logic             enable;       // idle PM on
        logic             dvfs;         // DVFS mode: one chip level, acknowledged by the host
        logic [7:0]       normal_level; // clock divider of the normal level
        logic [7:0]       dvfs_level;   // DVFS: the level the host acknowledged
        logic [31:0][7:0] domain_level; // DFS: the level last applied to each domain
    } pm_view_t;

    // Retirement of the head of one slot's checkout queue
    // (bingo_hw_manager_checkout_queue): the head may leave (valid), and the
    // path it is offered to; the ready of that path
    typedef struct packed {
        logic valid;       // the head may retire this cycle
        logic local_set;   // its dep_set, into the local dependency matrix
        logic chiplet_set; // its dep_set, to another chiplet
        logic remote_done; // imported: its done, back to the origin chiplet
    } retire_req_t;
    typedef struct packed {
        logic local_set;
        logic chiplet_set;
        logic remote_done;
    } retire_rsp_t;

    // Event log (bingo_hw_manager_evlog). The top raises one pulse per (group,
    // slot) and one per core type for a CERF fallback; the log turns the pulse
    // into a record with the group's code. The host decodes the codes, so they
    // are an interface: add new ones, never renumber.
    typedef enum int unsigned {
        EvgSuspect,     // watchdog: suspect
        EvgClear,       //           suspect cleared before the fence
        EvgFence,       //           fenced (arg: which threshold confirmed)
        EvgStuck,       // replay: slot stuck
        EvgBlocked,     //         slot blocked
        EvgRisk,        // late heartbeat: at risk
        EvgParked,      // parked
        EvgParkFail,    // park failed
        EvgRetired,     // fenced slot retired (all its tasks moved)
        EvgRecHold,     // recovery hold entered / left (arg: new state)
        EvgExitAbsorb   // exit task absorbed (arg: logical slot, task)
    } evlog_group_e;
    localparam int unsigned EvlogGroups = 11;
    // The per-type pulses sit between the first EvlogGroupsBeforeTypes groups and the rest
    localparam int unsigned EvlogGroupsBeforeTypes = 9;

    localparam logic [7:0] EvSuspect    = 8'h01;
    localparam logic [7:0] EvClear      = 8'h02;
    localparam logic [7:0] EvFence      = 8'h03;
    localparam logic [7:0] EvMove       = 8'h04; // replay moved a task (arg: destination slot, task)
    localparam logic [7:0] EvStuck      = 8'h05;
    localparam logic [7:0] EvBlocked    = 8'h06;
    localparam logic [7:0] EvRisk       = 8'h07;
    localparam logic [7:0] EvParked     = 8'h08;
    localparam logic [7:0] EvParkFail   = 8'h09;
    localparam logic [7:0] EvCerfFb     = 8'h0a; // CERF fallback of a core type (slot field: the type)
    localparam logic [7:0] EvRetired    = 8'h0b;
    localparam logic [7:0] EvRecHold    = 8'h0c;
    localparam logic [7:0] EvMoveDummy  = 8'h0d; // replay moved a task that runs on no core
    localparam logic [7:0] EvBounce     = 8'h0e; // replay found no destination
    localparam logic [7:0] EvExitAbsorb = 8'h0f;

    function automatic logic [7:0] evlog_group_code(input evlog_group_e group);
        case (group)
            EvgSuspect:  return EvSuspect;
            EvgClear:    return EvClear;
            EvgFence:    return EvFence;
            EvgStuck:    return EvStuck;
            EvgBlocked:  return EvBlocked;
            EvgRisk:     return EvRisk;
            EvgParked:   return EvParked;
            EvgParkFail: return EvParkFail;
            EvgRetired:  return EvRetired;
            EvgRecHold:  return EvRecHold;
            default:     return EvExitAbsorb;
        endcase
    endfunction

    // Position of a pulse in the event vector of the log
    function automatic int unsigned evlog_idx(
        input evlog_group_e group, input int unsigned slot,
        input int unsigned num_slots, input int unsigned num_types);
        return int'(group) * num_slots + ((int'(group) >= EvlogGroupsBeforeTypes) ? num_types : 0) + slot;
    endfunction
    function automatic int unsigned evlog_type_idx(
        input int unsigned core_type, input int unsigned num_slots);
        return EvlogGroupsBeforeTypes * num_slots + core_type;
    endfunction
endpackage
