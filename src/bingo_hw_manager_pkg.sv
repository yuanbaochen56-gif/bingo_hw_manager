// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Constants that more than one bingo_hw_manager module, or the host software,
// relies on: the task types of a descriptor, the device CSRs of the ready and
// done queues, the power domains and their counters, the event log codes, and
// the port structs of fixed width.
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
    // belongs to none (the host writes BINGO_PM_NO_DOMAIN, 99, for the host and
    // tied-off slots; libbingo bingo_api.h). The top maps every such value to
    // PmMaxDomains.
    localparam int unsigned PmMaxDomains = 32;
    // Power-domain counters (bingo_hw_manager_pm_counters), per counted domain,
    // in this order. The host reads them by index: add new ones at the end.
    typedef enum int unsigned {
        PmcIdle,    // manager cycles at the idle level
        PmcNormal,  //                at the normal level
        PmcBoost,   //                at the boost level
        PmcOther,   //                at any other level (access servo, derate)
        PmcActive   // manager cycles in which a slot of the domain is active
    } pm_counter_e;
    localparam int unsigned PmCounters = int'(PmcActive) + 1;  // PmcActive is the last

    // Port structs of fixed width. The ones whose width depends on the
    // parameters of the top are macros in bingo_hw_manager/typedef.svh.

    // Host configuration of the control plane (bingo_hw_manager_ctrl)
    // Substitute choice of the slot mapping table (bingo_hw_manager_slot_table),
    // from the host:
    //   spare      clusters searched first for a substitute, one bit per cluster
    //              (e.g. a cluster the compiler leaves empty as a cold spare);
    //              0 = none (the order of before)
    //   partition  SubPartBits per cluster (cluster i at [SubPartBits * i +:
    //              SubPartBits]): the partitions it belongs to. A slot of another
    //              cluster may stand in for a slot only if their clusters share a
    //              partition; the same cluster always may (level 1). Clusters from
    //              32 / SubPartBits on belong to every partition. All ones = one
    //              partition (the choice of before)
    localparam int unsigned SubPartBits = 4;
    typedef struct packed {
        logic [31:0] spare;
        logic [31:0] partition;
    } sub_cfg_t;
    localparam sub_cfg_t SubCfgDefault = '{spare: '0, partition: '1};

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
        sub_cfg_t    sub;            // substitute choice (see sub_cfg_t)
    } ctrl_cfg_t;

    // Configuration of the PM (bingo_hw_manager_pm) from the host. The levels
    // are clock dividers: the larger one is the slower clock.
    typedef struct packed {
        logic       enable;       // idle PM on
        logic       dvfs;         // 1: DVFS (notify the host of one chip level), 0: DFS (per domain)
        logic [7:0] idle_level;   // a domain whose slots all wait for a task
        logic [7:0] normal_level; // a busy domain
        logic [7:0] access_level; // DFS: a domain busy only with accesses from outside (0: normal)
        logic [7:0] boost_level;  // a domain with a boosted slot, recovery or capacity boost (0: off)
        logic [7:0] derate_level; // a domain with an at-risk slot runs no faster (0: off)
        logic [7:0] dvfs_level;   // DVFS: the level the host has applied (its acknowledge register)
    } pm_cfg_t;

    // Power state and configured levels (clock dividers) of the PM
    // (bingo_hw_manager_pm), as the control plane and the power-domain
    // counters read them
    typedef struct packed {
        logic             enable;       // idle PM on
        logic             dvfs;         // DVFS mode: one chip level, acknowledged by the host
        logic [7:0]       idle_level;   // the idle level
        logic [7:0]       normal_level; // the normal level
        logic [7:0]       boost_level;  // the boost level (0: off)
        logic [7:0]       dvfs_level;   // DVFS: the level the host has applied
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
    localparam int unsigned EvlogGroups = int'(EvgExitAbsorb) + 1;  // EvgExitAbsorb is the last
    // The per-type pulses sit between the first EvlogGroupsBeforeTypes groups
    // (EvgSuspect .. EvgRetired) and the rest
    localparam int unsigned EvlogGroupsBeforeTypes = int'(EvgRecHold);

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

    // A move record of the log: the replay moved, bounced or rotated a task
    // (code EvMove, EvMoveDummy or EvBounce), from slot {cluster, core}, arg
    // {destination slot (4'hf: rotated, 0: bounced), task id}
    typedef struct packed {
        logic        valid;
        logic [7:0]  code;
        logic [7:0]  slot;
        logic [15:0] arg;
    } evlog_move_t;

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
