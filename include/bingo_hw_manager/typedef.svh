// Copyright 2026 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Port structs of the bingo_hw_manager submodules whose width depends on the
// parameters of the top (number of cores and clusters, id widths). A package
// cannot take parameters, so these are macros, as the AXI types of
// axi/typedef.svh: bingo_hw_manager_top defines each type once and hands it
// to its submodules as a type parameter. The structs of fixed width are in
// bingo_hw_manager_pkg.
//
// Most fields are slot vectors, indexed [core][cluster] like the rest of the
// top: one struct carries several per-slot signals that travel together, and
// a module reads a whole vector (slot_i.fenced) or one slot
// (slot_i.fenced[c][cl]) as before.

`ifndef BINGO_HW_MANAGER_TYPEDEF_SVH_
`define BINGO_HW_MANAGER_TYPEDEF_SVH_

// State of every slot that the control plane, the placement and the replay read:
//   fenced    confirmed dead, sticky (bingo_hw_manager_watchdog)
//   retired   fenced and all its outstanding tasks moved (bingo_hw_manager_replay_ctrl)
//   stuck     fenced with a task no live core may run (bingo_hw_manager_replay_ctrl)
//   rejected  level 3: its export was rejected or timed out (bingo_hw_manager_remote_export)
`define BINGO_HW_MANAGER_TYPEDEF_SLOT_STATE_T(slot_state_t, NumCores, NumClusters) \
  typedef struct packed {                                                          \
    logic [(NumCores)-1:0][(NumClusters)-1:0] fenced;                              \
    logic [(NumCores)-1:0][(NumClusters)-1:0] retired;                             \
    logic [(NumCores)-1:0][(NumClusters)-1:0] stuck;                               \
    logic [(NumCores)-1:0][(NumClusters)-1:0] rejected;                            \
  } slot_state_t;

// Slot mapping table (bingo_hw_manager_slot_table), indexed by logical slot:
// found: the slot has a substitute, at physical slot (core, cluster)
`define BINGO_HW_MANAGER_TYPEDEF_SMT_T(smt_t, NumCores, NumClusters, CoreIdWidth, ClusterIdWidth) \
  typedef struct packed {                                                                         \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                     found;                          \
    logic [(NumCores)-1:0][(NumClusters)-1:0][(CoreIdWidth)-1:0]    core;                         \
    logic [(NumCores)-1:0][(NumClusters)-1:0][(ClusterIdWidth)-1:0] cluster;                      \
  } smt_t;

// Core parking (bingo_hw_manager_slot_table), per slot:
//   hold    draining before the move (no new task of the slot is placed)
//   parked  its later executing tasks run on its substitute
//   unpark  moving back: new tasks wait until none of its tasks is elsewhere
//   fail    the park failed; sticky until the slot's request drops
`define BINGO_HW_MANAGER_TYPEDEF_PARK_T(park_t, NumCores, NumClusters) \
  typedef struct packed {                                              \
    logic [(NumCores)-1:0][(NumClusters)-1:0] hold;                    \
    logic [(NumCores)-1:0][(NumClusters)-1:0] parked;                  \
    logic [(NumCores)-1:0][(NumClusters)-1:0] unpark;                  \
    logic [(NumCores)-1:0][(NumClusters)-1:0] fail;                    \
  } park_t;

// The queues of every slot as the control plane reads them:
//   waiting         the core polls its ready queue (or has exited)
//   checkout_empty  its checkout queue is empty
//   ready_empty     its ready queue is empty
//   push            a task enters its checkout or ready queue this cycle
//   moved           tasks of this logical slot sit in another slot's checkout queue
//   load            checkout queue occupancy (full = highest)
`define BINGO_HW_MANAGER_TYPEDEF_CTRL_QUEUE_T(ctrl_queue_t, NumCores, NumClusters, LoadWidth) \
  typedef struct packed {                                                                     \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                waiting;                         \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                checkout_empty;                  \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                ready_empty;                     \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                push;                            \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                moved;                           \
    logic [(NumCores)-1:0][(NumClusters)-1:0][(LoadWidth)-1:0] load;                          \
  } ctrl_queue_t;

// What the host sets per slot for the control plane:
//   park_req    park the slot (see bingo_hw_manager_slot_table)
//   risk_clear  keep the slot's late-beat count at 0 (bingo_hw_manager_risk)
//   domain      power domain of the slot (>= 32: none, see bingo_hw_manager_pkg PmMaxDomains)
`define BINGO_HW_MANAGER_TYPEDEF_CTRL_HOST_T(ctrl_host_t, NumCores, NumClusters) \
  typedef struct packed {                                                        \
    logic [(NumCores)-1:0][(NumClusters)-1:0]      park_req;                     \
    logic [(NumCores)-1:0][(NumClusters)-1:0]      risk_clear;                   \
    logic [(NumCores)-1:0][(NumClusters)-1:0][5:0] domain;                       \
  } ctrl_host_t;

// What the control plane tells the rest of the manager about every slot:
//   idle         the PM may count the slot as idle (bingo_hw_manager_power_view)
//   access_only  otherwise idle, awake only because its cluster is accessed
//   boost        the PM runs its domain at the boost level (bingo_hw_manager_level_policy)
//   derate       the PM runs its domain at most at the derate level
//   rec_hold     recovery hold active (for the event log, not a PM input)
//   load_clear   the load monitor clears its pending count
//   wd_tick      its watchdog timer advances this cycle (bingo_hw_manager_wd_tick)
//   wd_fast      its domain runs faster than the normal level
//   risk         at risk (bingo_hw_manager_risk)
`define BINGO_HW_MANAGER_TYPEDEF_CTRL_VIEW_T(ctrl_view_t, NumCores, NumClusters) \
  typedef struct packed {                                                        \
    logic [(NumCores)-1:0][(NumClusters)-1:0] idle;                              \
    logic [(NumCores)-1:0][(NumClusters)-1:0] access_only;                       \
    logic [(NumCores)-1:0][(NumClusters)-1:0] boost;                             \
    logic [(NumCores)-1:0][(NumClusters)-1:0] derate;                            \
    logic [(NumCores)-1:0][(NumClusters)-1:0] rec_hold;                          \
    logic [(NumCores)-1:0][(NumClusters)-1:0] load_clear;                        \
    logic [(NumCores)-1:0][(NumClusters)-1:0] wd_tick;                           \
    logic [(NumCores)-1:0][(NumClusters)-1:0] wd_fast;                           \
    logic [(NumCores)-1:0][(NumClusters)-1:0] risk;                              \
  } ctrl_view_t;

// Host table per core type (0 = off):
//   cerf_fb_en     degrade the CERF once a slot of the type is stuck or rejected
//                  (bingo_hw_manager_cerf_fallback): clear group cerf_fb_clear,
//                  set group cerf_fb_set
//   fence_fb       fence fallback: a fenced or parked slot of the type takes no
//                  substitute at any level (bingo_hw_manager_slot_table)
`define BINGO_HW_MANAGER_TYPEDEF_TYPE_CFG_T(type_cfg_t, NumTypes) \
  typedef struct packed {                                         \
    logic [(NumTypes)-1:0]      cerf_fb_en;                       \
    logic [(NumTypes)-1:0][4:0] cerf_fb_clear;                    \
    logic [(NumTypes)-1:0][4:0] cerf_fb_set;                      \
    logic [(NumTypes)-1:0]      fence_fb;                         \
  } type_cfg_t;

// One CERF degradation (bingo_hw_manager_cerf_fallback), held until the CERF
// commits it: clear group clear, set group set, for core type core_type
`define BINGO_HW_MANAGER_TYPEDEF_CERF_FB_T(cerf_fb_t, CoreTypeIdWidth) \
  typedef struct packed {                                              \
    logic                       req;                                   \
    logic [(CoreTypeIdWidth)-1:0] core_type;                           \
    logic [4:0]                 clear;                                 \
    logic [4:0]                 set;                                   \
  } cerf_fb_t;

// Heads of the waiting queues, one per logical core, once checked:
//   desc   the head (task_desc_t: bingo_hw_manager_task_desc_t of the top)
//   empty  the waiting queue is empty
//   skip   the CERF skips the head (it runs on no core)
`define BINGO_HW_MANAGER_TYPEDEF_WAITING_T(waiting_t, task_desc_t, NumCores) \
  typedef struct packed {                                                    \
    task_desc_t [(NumCores)-1:0] desc;                                       \
    logic       [(NumCores)-1:0] empty;                                      \
    logic       [(NumCores)-1:0] skip;                                       \
  } waiting_t;

// The checkout queues of every slot (bingo_hw_manager_checkout_queue):
//   push, push_desc, push_tag  an entry is pushed this cycle, and the entry with its side tag
//   pop                        the head leaves (retired, or moved by the replay)
//   full                       no room for a new task (level 3 keeps a spare entry)
//   full_raw, usage, empty     occupancy of the FIFO
//   head, head_tag             the head and its side tag (remote_tag_t of the top)
//   head_imported              the head runs here for another chiplet
//   head_exec                  the head runs on a core: it retires with its done
`define BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_T(checkout_t, task_desc_t, remote_tag_t, NumCores, NumClusters, UsageWidth) \
  typedef struct packed {                                                                                             \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                 push;                                            \
    task_desc_t  [(NumCores)-1:0][(NumClusters)-1:0]                 push_desc;                                       \
    remote_tag_t [(NumCores)-1:0][(NumClusters)-1:0]                 push_tag;                                        \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                 pop;                                             \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                 full;                                            \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                 full_raw;                                        \
    logic        [(NumCores)-1:0][(NumClusters)-1:0][(UsageWidth)-1:0] usage;                                         \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                 empty;                                           \
    task_desc_t  [(NumCores)-1:0][(NumClusters)-1:0]                 head;                                            \
    remote_tag_t [(NumCores)-1:0][(NumClusters)-1:0]                 head_tag;                                        \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                 head_imported;                                   \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                 head_exec;                                       \
  } checkout_t;

// The ready queues of every slot (bingo_hw_manager_ready_queue):
//   push, push_data  an entry is pushed this cycle, and the word the core reads
//                    (ready_desc_t: bingo_hw_manager_ready_task_desc_full_t of the top)
//   full, empty      occupancy
//   pop, data        the core reads the head (CSR interface)
//   wd_class, moved  the class of the head and whether it left its logical slot
`define BINGO_HW_MANAGER_TYPEDEF_READY_T(ready_t, ready_desc_t, NumCores, NumClusters, WdClassWidth) \
  typedef struct packed {                                                                            \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                   push;                         \
    ready_desc_t [(NumCores)-1:0][(NumClusters)-1:0]                   push_data;                    \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                   full;                         \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                   empty;                        \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                   pop;                          \
    ready_desc_t [(NumCores)-1:0][(NumClusters)-1:0]                   data;                         \
    logic        [(NumCores)-1:0][(NumClusters)-1:0][(WdClassWidth)-1:0] wd_class;                   \
    logic        [(NumCores)-1:0][(NumClusters)-1:0]                   moved;                        \
  } ready_t;

// Placement of the waiting queue heads (bingo_hw_manager_router), per logical
// core: the head may go (valid) to physical slot (core, cluster)
`define BINGO_HW_MANAGER_TYPEDEF_SELECT_T(select_t, NumCores, CoreIdWidth, ClusterIdWidth) \
  typedef struct packed {                                                                  \
    logic [(NumCores)-1:0]                       valid;                                    \
    logic [(NumCores)-1:0][(CoreIdWidth)-1:0]    core;                                     \
    logic [(NumCores)-1:0][(ClusterIdWidth)-1:0] cluster;                                  \
  } select_t;

// What the router does for every destination slot:
//   valid        a waiting queue head is routed to the slot, from logical core src_core
//   exit_absorb  it is an exit absorbed into a dummy (it runs on no core)
//   remote       level 3: it is exported, the slot is its proxy
//   fire         it enters the slot's checkout queue this cycle
//   ready_push   ... and its ready queue (it runs on a core)
`define BINGO_HW_MANAGER_TYPEDEF_ROUTE_T(route_t, NumCores, NumClusters, CoreIdWidth) \
  typedef struct packed {                                                             \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                    valid;               \
    logic [(NumCores)-1:0][(NumClusters)-1:0][(CoreIdWidth)-1:0] src_core;            \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                    exit_absorb;         \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                    remote;              \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                    fire;                \
    logic [(NumCores)-1:0][(NumClusters)-1:0]                    ready_push;          \
  } route_t;

// All port types of the control plane (bingo_hw_manager_ctrl), named as its
// type parameters, for a module or testbench that instantiates it
`define BINGO_HW_MANAGER_TYPEDEF_CTRL_ALL(NumCores, NumClusters, CoreIdWidth, ClusterIdWidth, CoreTypeIdWidth, LoadWidth) \
  `BINGO_HW_MANAGER_TYPEDEF_SLOT_STATE_T(slot_state_t, NumCores, NumClusters)                                             \
  `BINGO_HW_MANAGER_TYPEDEF_CTRL_QUEUE_T(ctrl_queue_t, NumCores, NumClusters, LoadWidth)                                  \
  `BINGO_HW_MANAGER_TYPEDEF_CTRL_HOST_T(ctrl_host_t, NumCores, NumClusters)                                               \
  `BINGO_HW_MANAGER_TYPEDEF_CTRL_VIEW_T(ctrl_view_t, NumCores, NumClusters)                                               \
  `BINGO_HW_MANAGER_TYPEDEF_TYPE_CFG_T(type_cfg_t, 2**(CoreTypeIdWidth))                                                  \
  `BINGO_HW_MANAGER_TYPEDEF_SMT_T(smt_t, NumCores, NumClusters, CoreIdWidth, ClusterIdWidth)                              \
  `BINGO_HW_MANAGER_TYPEDEF_PARK_T(park_t, NumCores, NumClusters)                                                         \
  `BINGO_HW_MANAGER_TYPEDEF_CERF_FB_T(cerf_fb_t, CoreTypeIdWidth)

`endif
