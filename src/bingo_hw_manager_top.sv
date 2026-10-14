// Copyright 2025 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Fanchen Kong <fanchen.kong@kuleuven.be>
// - Xiaoling Yi  <xiaoling.yi@kuleuven.be>
// - Yunhao Deng  <yunhao.deng@kuleuven.be>
`include "bingo_hw_manager/typedef.svh"
module bingo_hw_manager_top #(
    parameter int unsigned READY_AND_DONE_QUEUE_INTERFACE_TYPE = 1, // 1: CSR Req/Resp 0: Default AXi Lite Slave
    parameter int unsigned TASK_QUEUE_TYPE = 1,                     // 1: AXI Lite Master 0: Default AXI Lite Slave
    parameter int unsigned NUM_CORES_PER_CLUSTER = 4,
    parameter int unsigned NUM_CLUSTERS_PER_CHIPLET = 2,
    // Dedicated host DVFS doorbell bit inside the shared CLINT MSIP word. Injected from
    // the HeMAiA level (occamygen hw_manager_ipi_idx) and forwarded to the PM so it is
    // never hardcoded; must match HW_MANAGER_DVFS_MSIP_BIT / occamy_soc.sv ipi_i.
    parameter int unsigned HOST_DVFS_MSIP_BIT = 3,
    parameter int unsigned ChipIdWidth = 8,
    parameter int unsigned TaskIdWidth = 12,
    // Identity-aware dependency tracking (per-edge tags). The mini-compiler's
    // per-edge tags are plumbed to the tagged dep-matrix scoreboard so a
    // consumer drains only ITS producer's increment (no counter-sharing hazard).
    parameter int unsigned DepTagWidth = 4,
    parameter int unsigned WatchdogHeartbeatTimeoutCycles = 100000, // The number of cycles for the watchdog to time out
    // Cycles a busy core may go without heartbeat before it is fenced (confirmed
    // dead, sticky until reset) and its outstanding tasks are replayed on other
    // cores. Must exceed WatchdogHeartbeatTimeoutCycles. 0 disables fencing,
    // replay and remap (detection only). Needs the CSR ready/done interface.
    parameter int unsigned WatchdogConfirmTimeoutCycles = 0,
    // Watchdog enable per (core, cluster) slot. A masked slot is never reported
    // dead_suspect (e.g. a host slot that sends no heartbeats, or a tied-off slot).
    parameter logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] WatchdogCoreMask = '1,
    // Core remap / replay: type id of each (core, cluster) slot, chiplet-wide. Two
    // cores with the same non-zero type can run each other's tasks, also from
    // another cluster (level 2, see SubstituteLevelMask), so one may take over the
    // tasks of the other once that one is fenced; a substitute in the dead core's
    // own cluster is preferred. Give
    // cores whose tasks only work in their own cluster (e.g. operands in local
    // L1 at local addresses) a cleared SubstituteL2TypeEn bit. Type 0: the slot
    // neither hands over its tasks nor takes over others' (e.g. a host slot).
    // Default: all cores of the chiplet are interchangeable.
    parameter int unsigned CoreTypeIdWidth = 4,
    parameter int unsigned WdAdaptTypeBits = 2,
    parameter int unsigned WdClassWidth = 3,
    parameter logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][CoreTypeIdWidth-1:0] CoreTypeId =
        {(NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET){CoreTypeIdWidth'(1)}},
    // Where a fenced core's tasks (outstanding: replay, new: remap) may go, per
    // level; the lowest enabled level with a live same-type core wins:
    //   [0] level 1: a core of the same cluster
    //   [1] level 2: a core of another cluster of this chiplet
    //   [2] level 3: another chiplet, through the remote dispatch interface
    // Default: level 1 only. Without WatchdogConfirmTimeoutCycles nothing is
    // fenced and this mask has no effect (detection only).
    parameter logic [2:0] SubstituteLevelMask = 3'b001,
    // A cleared bit keeps that core type within its assigned cluster.
    // Same-cluster substitutes remain enabled by SubstituteLevelMask[0].
    parameter logic [2**CoreTypeIdWidth-1:0] SubstituteL2TypeEn = '1,
    // Levels an imported task (level 3, from another chiplet) may use to find a
    // live stand-in for its home slot here (bit 2 is ignored: never re-exported).
    // Default: the local levels of SubstituteLevelMask. Setting it apart from
    // SubstituteLevelMask is a debug / loopback aid: e.g. SubstituteLevelMask =
    // 3'b100 with ImportSubstituteLevelMask = 3'b001 exports every task of a
    // fenced core (force remote) and still lets it run on a same-cluster core
    // when it comes back through a loopback link.
    parameter logic [2:0] ImportSubstituteLevelMask = SubstituteLevelMask & 3'b011,
    // Substitute choice when a core dies (bingo_hw_manager_slot_table): 0 = lowest
    // live core of the type, 1 = least loaded (checkout queue occupancy), both
    // in the logical cluster first. Fixed until that substitute dies.
    parameter int unsigned SubstitutePolicy = 0,
    // A stuck or rejected slot may also drain heads of another logical core
    // that died before it (see stuck_drain): one counter per (slot, logical
    // core) pair. 0 = only its own logical core (no counters).
    parameter bit          ForeignStuckDrain = 1'b1,
    // CSR number of the heartbeat write (see bingo_hw_manager_csr_to_fifo).
    parameter logic [11:0] CsrHeartbeatAddr = 12'h5fd,
    // AXI interface types
    // The task queue holds tasks to be scheduled to the devices
    // Host writes the task queue via 64bit AXI Lite
    parameter int unsigned HostAxiLiteAddrWidth = 48,
    parameter int unsigned HostAxiLiteDataWidth = 64,
    // Device writes the done queue via 32bit AXI Lite
    parameter int unsigned DeviceAxiLiteAddrWidth = 48,
    parameter int unsigned DeviceAxiLiteDataWidth = 32,
    // AXI Lite Interface types for host and device
    parameter type host_axi_lite_req_t = logic,
    parameter type host_axi_lite_resp_t = logic,
    parameter type device_axi_lite_req_t = logic,
    parameter type device_axi_lite_resp_t = logic,
    parameter type csr_req_t = logic,
    parameter type csr_rsp_t = logic,
    // FIFO Depths
    parameter int unsigned TaskQueueDepth = 32,
    parameter int unsigned ChipletDoneQueueDepth = 32,
    parameter int unsigned DoneQueueDepth = 32,
    parameter int unsigned CheckoutQueueDepth = 8,
    parameter int unsigned ReadyQueueDepth = 8,
    // Waiting queue of each core index, shared by the clusters: a task waits
    // there for its dependency check before it is placed on a slot
    parameter int unsigned WaitingQueueDepth = 8,
    parameter int unsigned EventLogDepth = 0,
    parameter int unsigned EventLogAgeWidth = 6,
    // Power-domain counters (bingo_hw_manager_pm_counters, pm_count_o) of the
    // domains PmCountBase .. PmCountBase + PmCountDomains - 1; 0 = none
    parameter int unsigned PmCountBase = 0,
    parameter int unsigned PmCountDomains = 0,
    // Level 3 export queues: one per peer of the transport (e.g.
    // bingo_hw_manager_remote_link NumPeers), each deep enough for a whole
    // checkout queue, so a peer without credits only holds its own exports
    parameter int unsigned RemoteNumPeers = 1,
    parameter int unsigned RemoteExportFifoDepth = CheckoutQueueDepth + 1,
    // Address Offsets
    parameter int unsigned ReadyQueueAddrOffset = 4096,
    // Dependent parameters, DO NOT OVERRIDE!
    parameter type chip_id_t = logic [ChipIdWidth-1:0],
    parameter type host_axi_lite_addr_t = logic [HostAxiLiteAddrWidth-1:0],
    parameter type host_axi_lite_data_t = logic [HostAxiLiteDataWidth-1:0],
    parameter type device_axi_lite_addr_t = logic [DeviceAxiLiteAddrWidth-1:0],
    parameter type device_axi_lite_data_t = logic [DeviceAxiLiteDataWidth-1:0],
    // Flat slot id (core + cluster * NUM_CORES_PER_CLUSTER) of the remote interface
    parameter int unsigned RemoteSlotIdWidth = cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET),
    parameter int unsigned RemotePeerIdWidth = (RemoteNumPeers > 1) ? $clog2(RemoteNumPeers) : 1,
    parameter int unsigned PmCountPorts = (PmCountDomains != 0) ? PmCountDomains : 1
) (
    /// Clock
    input logic clk_i,
    /// Asynchronous reset, active low
    input logic rst_ni,
    /// Chip ID for multi-chip addressing
    input chip_id_t chip_id_i,
    /// Interface to the system
    // For the task queue, we have two interfaces:
    // 1. Host writes to the task queue via 64bit AXI Lite interface
    // Host -----> Task Queue
    // Here this queue holds all the tasks to be scheduled to the devices
    // Hence this is a slave AXI Lite interface
    input  host_axi_lite_addr_t                 task_queue_base_addr_i,
    input  host_axi_lite_req_t                  task_queue_axi_lite_req_i,
    output host_axi_lite_resp_t                 task_queue_axi_lite_resp_o,
    // 2. The Hw Manager issues the read request to the address specified by the host via the following inputs
    // Hence this is a master AXI Lite interface
    input host_axi_lite_addr_t                  task_list_base_addr_i, // The task list base address specified by the host
    input device_axi_lite_data_t                num_task_i,            // The number of tasks specified by the host
    // Control signals to start the HW Manager (from the reg gen modules)
    input  device_axi_lite_data_t               bingo_hw_manager_start_i,
    output device_axi_lite_data_t               bingo_hw_manager_reset_start_o,
    output logic                                bingo_hw_manager_reset_start_en_o,
    output host_axi_lite_req_t                  task_queue_axi_lite_req_o,
    input  host_axi_lite_resp_t                 task_queue_axi_lite_resp_i,
    /// The chiplet set interface to other chiplets
    // HW Manager -----> Other chiplets
    input  host_axi_lite_addr_t                 chiplet_mailbox_base_addr_i,
    output host_axi_lite_req_t                  to_remote_chiplet_axi_lite_req_o,
    input  host_axi_lite_resp_t                 to_remote_chiplet_axi_lite_resp_i,
    /// The chiplet done interface from other chiplets
    input  host_axi_lite_req_t                  from_remote_axi_lite_req_i,
    output host_axi_lite_resp_t                 from_remote_axi_lite_resp_o,
    /// The done queue interface to the devices
    // Devices -----> Done Queue
    // Here this queue holds all the completed tasks info from the devices
    // The device cores will write completed tasks into this queue via 32bit AXI Lite
    input  device_axi_lite_addr_t               done_queue_base_addr_i,
    input  device_axi_lite_req_t                done_queue_axi_lite_req_i,
    output device_axi_lite_resp_t               done_queue_axi_lite_resp_o,
    /// The ready queue interface to the devices
    // HW scheduler -----> Ready Queue
    // Here the ready queue holds the tasks that are ready to be executed by the devices
    // The device cores will read tasks from this queue via 32bit AXI Lite
    // Each core has its own ready queue interface
    input  device_axi_lite_addr_t               ready_queue_base_addr_i,
    input  device_axi_lite_req_t                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    ready_queue_axi_lite_req_i,
    output device_axi_lite_resp_t               [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    ready_queue_axi_lite_resp_o,
    /// CSR Req/Resp Interface: the cores read the ready queue and write the done queue
    input  csr_req_t                            [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_req_i,
    input  logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_req_valid_i,
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_req_ready_o,
    output csr_rsp_t                            [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_rsp_o,
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_rsp_valid_o,
    input  logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_rsp_ready_i,
    /// The interface to the Power Management Module
    // Host configuration interface
    input device_axi_lite_data_t                bingo_hw_manager_enable_idle_pm_i,
    // Recovery boost level (a faster clock divider than the normal level) of a
    // domain whose core runs a dead core's tasks; 0 disables the boost
    input device_axi_lite_data_t                bingo_hw_manager_boost_power_level_i = '0,
    // Which cores get the boost level (bingo_hw_manager_level_policy): [0] 0 = the
    // substitutes of fenced cores, 1 = capacity (busy cores of a type that lost
    // a core); [11:8] boosted domains per lost core (0 = no limit); [23:16]
    // minimum load (checkout occupancy) of a boosted core
    input device_axi_lite_data_t                bingo_hw_manager_boost_policy_i = '0,
    // Cycles a core must be idle before it lets its domain drop to the idle
    // level (bingo_hw_manager_power_view); 0 = at once
    input device_axi_lite_data_t                bingo_hw_manager_idle_entry_delay_i = '0,
    // External access wake (bingo_hw_manager_power_view): a request from outside into
    // a cluster's memory this cycle (e.g. the host reading its L1), and the
    // cycles the cluster's domain stays awake after the last one; 0 = off
    input logic [NUM_CLUSTERS_PER_CHIPLET-1:0]  bingo_hw_manager_cluster_access_i = '0,
    input device_axi_lite_data_t                bingo_hw_manager_access_wake_hold_i = '0,
    input logic [31:0]                         bingo_hw_manager_recovery_hold_i = '0,
    input logic [31:0]                         bingo_hw_manager_pm_access_level_i = '0,
    // Core parking (bingo_hw_manager_slot_table). Bit (core + cluster*NUM_CORES) asks
    // that logical slot to drain, then send later executing tasks to a live
    // core of its type. 0 leaves placement unchanged. park_fail_o is sticky for
    // a slot until its request bit drops (no substitute, or the slot runs
    // another slot's tasks, or the substitute died and none was left). Clearing
    // the request of a parked slot moves it back: its new tasks wait until none
    // of its tasks is left on another core, then run on the slot again.
    input  device_axi_lite_data_t               bingo_hw_manager_park_req_i = '0,
    output device_axi_lite_data_t               bingo_hw_manager_park_fail_o,
    // Fault precursors (bingo_hw_manager_risk). A heartbeat or done of a busy
    // core whose watchdog timer reached risk_late is late (0 = off). policy:
    // [3:0] late beats that make a slot at risk (0 = off), [4] park it,
    // [5] derate its domain, [15:8] derate level. epoch: the counts are
    // halved every that many cycles (0 = never). clear and risk_o use the
    // park_req bit order: a set clear bit keeps that slot's count at 0.
    input  device_axi_lite_data_t               bingo_hw_manager_risk_late_i = '0,
    input  device_axi_lite_data_t               bingo_hw_manager_risk_policy_i = '0,
    input  device_axi_lite_data_t               bingo_hw_manager_risk_epoch_i = '0,
    input  device_axi_lite_data_t               bingo_hw_manager_risk_clear_i = '0,
    input  logic [31:0]                        bingo_hw_manager_risk_confirm_i = '0,
    // Runtime type thresholds may only shorten the parameter timeouts.
    // Zero/invalid entries fall back to the parameters; validate before narrowing.
    input  logic [2**CoreTypeIdWidth-1:0][31:0] wd_type_h_i = '0,
    input  logic [2**CoreTypeIdWidth-1:0][31:0] wd_type_c_i = '0,
    input  logic [31:0]                        wd_adapt_i = '0,
    input  logic [31:0]                        bingo_hw_manager_wd_late_i = '0,
    output device_axi_lite_data_t               bingo_hw_manager_risk_o,
    input device_axi_lite_data_t                bingo_hw_manager_idle_power_level_i,
    input device_axi_lite_data_t                bingo_hw_manager_normal_power_level_i,
    input device_axi_lite_addr_t                bingo_hw_manager_pm_base_addr_i,
    input device_axi_lite_data_t                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    bingo_hw_manager_core_power_domain_i,
    // DVFS: mode select, CLINT doorbell address, host ack, and published request
    input  device_axi_lite_data_t               bingo_hw_manager_pm_mode_i,
    input  device_axi_lite_addr_t               bingo_hw_manager_dvfs_clint_msip_addr_i,
    input  device_axi_lite_data_t               bingo_hw_manager_dvfs_ack_i,
    output device_axi_lite_data_t               bingo_hw_manager_dvfs_request_o,
    // AXI Lite Master Interface
    output host_axi_lite_req_t                  pm_axi_lite_req_o,
    input  host_axi_lite_resp_t                 pm_axi_lite_resp_i,
    // DARTS: CERF (Conditional Execution Register File) interface
    input  logic                                cerf_write_en_i,
    input  logic [31:0]                         cerf_write_data_i,
    output logic [31:0]                         cerf_state_o,
    // Degradation (bingo_hw_manager_cerf_fallback): per core type, once any slot of that
    // type is stuck or rejected, clear one CERF group and set another.
    // Enable 0 leaves the CERF unchanged. evt is sticky until that enable drops.
    input  logic [2**CoreTypeIdWidth-1:0]       cerf_fb_en_i = '0,
    input  logic [2**CoreTypeIdWidth-1:0][4:0]  cerf_fb_clear_i = '0,
    input  logic [2**CoreTypeIdWidth-1:0][4:0]  cerf_fb_set_i = '0,
    output logic [2**CoreTypeIdWidth-1:0]       cerf_fb_evt_o,
    // Fence fallback (bingo_hw_manager_slot_table): per core type, a fenced or parked
    // slot of that type takes no substitute at any level (it is not exported
    // either), so a fence makes it stuck and the degradation above applies.
    // 0 = off. Set it before the first fence or park.
    input  logic [2**CoreTypeIdWidth-1:0]       bingo_hw_manager_fence_fb_i = '0,
    // Observation-only event log. CLEAR/POP trigger once per value change.
    input logic                                evlog_enable_i = 1'b0,
    input logic [31:0]                         evlog_clear_i = '0,
    input logic [31:0]                         evlog_pop_i = '0,
    output logic [63:0]                        evlog_head_o,
    output logic [31:0]                        evlog_count_o,
    output logic [15:0]                        evlog_dropped_o,
    // DARTS: Load Monitor output (CSR readable)
    output logic [10:0]                         load_total_pending_o,
    // Power-domain counters: per counted domain, bingo_hw_manager_pkg PmcIdle .. PmcActive
    output logic [PmCountPorts-1:0][bingo_hw_manager_pkg::PmCounters-1:0][31:0] pm_count_o,
    // Watchdog: confirmed dead cores (sticky). The system may reset / isolate them.
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    core_fenced_o,
    // Replay: a fenced core holds a task that no live core may run (level 3:
    // on this chiplet, or on the remote chiplet that rejected its export)
    output logic                                replay_stuck_o,
    // Sticky no-replay blocks, indexed like core_fenced_o; a subset of stuck.
    output logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_blocked_o,
    // Watchdog: busy cores without heartbeat for WatchdogHeartbeatTimeoutCycles
    // (not sticky, cleared by a heartbeat or a done)
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    core_dead_suspect_o,
    // A ready task waits in the slot's ready queue: lets a reader of the CSR
    // ready queue (which stalls while it is empty) poll without blocking.
    // Always 0 with the AXI-Lite mailbox queues, which have their own status.
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    ready_queue_pending_o,
    // Level 3 remote dispatch (SubstituteLevelMask[2]; unused and tied off
    // otherwise, all inputs have defaults). valid/ready streams.
    // Export: a task of this chiplet that no live core here may run. Its
    // logical slot (proxy_slot, flat id) stays its proxy until the done returns.
    output logic                                remote_dispatch_valid_o,
    input  logic                                remote_dispatch_ready_i = 1'b0,
    output host_axi_lite_data_t                 remote_dispatch_desc_o,        // bingo_hw_manager_task_desc_full_t
    output logic [CoreTypeIdWidth-1:0]          remote_dispatch_core_type_o,   // CoreTypeId of the logical slot
    output chip_id_t                            remote_dispatch_origin_chip_o,
    output logic [RemoteSlotIdWidth-1:0]        remote_dispatch_proxy_slot_o,
    // Import: a task exported by another chiplet, run on a core of the same type here
    input  logic                                remote_dispatch_valid_i = 1'b0,
    output logic                                remote_dispatch_ready_o,
    input  host_axi_lite_data_t                 remote_dispatch_desc_i = '0,
    input  logic [CoreTypeIdWidth-1:0]          remote_dispatch_core_type_i = '0,
    input  chip_id_t                            remote_dispatch_origin_chip_i = '0,
    input  logic [RemoteSlotIdWidth-1:0]        remote_dispatch_proxy_slot_i = '0,
    // Done of an imported task, to be delivered to chiplet remote_done_chip_o
    output logic                                remote_done_valid_o,
    input  logic                                remote_done_ready_i = 1'b0,
    output chip_id_t                            remote_done_chip_o,
    output logic [RemoteSlotIdWidth-1:0]        remote_done_proxy_slot_o,
    output logic [TaskIdWidth-1:0]              remote_done_task_id_o,
    // ... or a reject: no live core here may run the imported task (none of its
    // type was live on arrival, or its core died and none is left)
    output logic                                remote_done_reject_o,
    // Done of an exported task (from the chiplet that ran it)
    input  logic                                remote_done_valid_i = 1'b0,
    output logic                                remote_done_ready_o,
    input  logic [RemoteSlotIdWidth-1:0]        remote_done_proxy_slot_i = '0,
    input  logic [TaskIdWidth-1:0]              remote_done_task_id_i = '0,
    // ... or its reject: the proxy slot becomes stuck (replay_stuck_o), after
    // the dones that arrived before it retired its earlier entries
    input  logic                                remote_done_reject_i = 1'b0,
    // Core types the transport can export (it has a target chiplet for them),
    // e.g. bingo_hw_manager_remote_link target_valid_o. A fenced core of another
    // type without a local substitute is stuck, as without level 3, instead of
    // waiting forever in an export queue. Default: every type.
    input  logic [2**CoreTypeIdWidth-1:0]       remote_export_type_en_i = '1,
    // Peer (export queue) of each core type, and the peers whose next export
    // the transport takes at once (a free credit), e.g. remote_link
    // target_peer_o / peer_ready_o. Exports only leave a queue whose peer is
    // ready, round robin, so a peer without credits does not hold the others.
    input  logic [2**CoreTypeIdWidth-1:0][RemotePeerIdWidth-1:0] remote_export_type_peer_i = '0,
    input  logic [RemoteNumPeers-1:0]           remote_export_peer_ready_i = '1,
    // Origin side: cycles a proxy slot may wait for the remote done of its
    // exported head before it gives up as on a reject (stops, replay_stuck_o)
    // and sets remote_timeout_o; 0 = wait forever
    input  device_axi_lite_data_t               remote_proxy_timeout_i = '0,
    // Sticky: a proxy slot gave up waiting for a remote done (see above)
    output logic                                remote_timeout_o,
    // Sticky: the done at the head of a proxy slot's done queue does not
    // belong to its exported head task (the slot then stops retiring), or a
    // remote done / reject arrived for a slot that is no proxy (not fenced, or
    // out of range; dropped)
    output logic                                remote_done_mismatch_o,
    // Sticky bus errors (SLVERR / DECERR) of the manager's AXI-Lite masters:
    // [0] a task descriptor read (the descriptor is dropped and the fetch
    // stops, bingo_hw_manager_task_queue_master), [1] a write of the PM, [2] the
    // DVFS read of the PM (bingo_hw_manager_pm). The chiplet dep-set master
    // gets its B response from the D2D link, which answers every write.
    output logic [2:0]                          bus_error_o
);
    import bingo_hw_manager_pkg::*;
    // --------Type definitions and signal declarations--------------------//
    // Task type (two bits, DARTS gating included): values and meaning in bingo_hw_manager_pkg
    typedef logic [1:0]                                  bingo_hw_manager_task_type_t;
    // Task ID
    typedef logic [TaskIdWidth-1:0                     ] bingo_hw_manager_task_id_t;
    // Assigned Chiplet ID
    typedef logic [ChipIdWidth-1:0                     ] bingo_hw_manager_assigned_chiplet_id_t;
    // Assigned Cluster ID
    typedef logic [cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET)-1:0] bingo_hw_manager_assigned_cluster_id_t;
    // Assigned Core ID
    typedef logic [cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)-1:0   ] bingo_hw_manager_assigned_core_id_t;
    // Dependency check info struct
    typedef logic [NUM_CORES_PER_CLUSTER-1:0]            bingo_hw_manager_dep_code_t;
    // Per-edge identity tag. Carried alongside the dep code so
    // it flows through every existing dep_check_info / dep_set_info copy unchanged.
    typedef logic [DepTagWidth-1:0]                      bingo_hw_manager_dep_tag_t;
    typedef struct packed{
        bingo_hw_manager_dep_tag_t                   dep_check_tag;
        bingo_hw_manager_dep_code_t                  dep_check_code;
        logic                                        dep_check_en;
    } bingo_hw_manager_dep_check_info_t;
    // Dependency set info struct
    typedef struct packed{
        bingo_hw_manager_dep_tag_t                   dep_set_tag;
        bingo_hw_manager_dep_code_t                  dep_set_code;
        bingo_hw_manager_assigned_cluster_id_t       dep_set_cluster_id;
        bingo_hw_manager_assigned_chiplet_id_t       dep_set_chiplet_id;
        logic                                        dep_set_all_chiplet;
        logic                                        dep_set_en;
    } bingo_hw_manager_dep_set_info_t;

    // Task info struct (DARTS: includes conditional execution fields)
    typedef struct packed{
        logic [WdClassWidth-1:0]                     wd_class;
        logic                                        is_exit;
        bingo_hw_manager_dep_set_info_t              dep_set_info;
        bingo_hw_manager_dep_check_info_t            dep_check_info;
        bingo_hw_manager_assigned_core_id_t          assigned_core_id;
        bingo_hw_manager_assigned_cluster_id_t       assigned_cluster_id;
        bingo_hw_manager_assigned_chiplet_id_t       assigned_chiplet_id;
        bingo_hw_manager_task_id_t                   task_id;
        bingo_hw_manager_task_type_t                 task_type;
        // DARTS Tier 1: Conditional Execution
        logic                                        cond_exec_en;
        logic [4:0]                                  cond_exec_group_id;
        logic                                        cond_exec_invert;
    } bingo_hw_manager_task_desc_t;

    // The watchdog only sees dispatches (ready queue pops) in the CSR interface mode.
    if ((WatchdogConfirmTimeoutCycles != 0) && (READY_AND_DONE_QUEUE_INTERFACE_TYPE != 1)) begin : gen_replay_intf_check
        initial begin
        $error("WatchdogConfirmTimeoutCycles (task replay) needs READY_AND_DONE_QUEUE_INTERFACE_TYPE = 1 (CSR)");
        $finish;
        end
    end

    // Retain the legacy width/offset of is_exit. Class is optional metadata
    // in the low reserved bits, never an extension of the host descriptor.
    localparam int unsigned WdTaskDescWidth = $bits(bingo_hw_manager_task_desc_t);
    localparam int unsigned TaskDescWidth = WdTaskDescWidth - WdClassWidth;
    localparam int unsigned ReservedBitsForTaskDesc = HostAxiLiteDataWidth - TaskDescWidth;
    localparam bit WdClassAvail = (ReservedBitsForTaskDesc >= WdClassWidth);
    if (TaskDescWidth>HostAxiLiteDataWidth) begin : gen_task_desc_width_check
        initial begin
        $error("Task Decriptor width (%0d) exceeds Host AXI Lite Data Width (%0d)! Please adjust the parameters accordingly.", TaskDescWidth, HostAxiLiteDataWidth);
        $finish;
        end
    end
    // 64bit Task Descriptor with reserved bits
    typedef struct packed{
        logic [ReservedBitsForTaskDesc-1:0]          reserved_bits;
        logic                                        is_exit;
        bingo_hw_manager_dep_set_info_t              dep_set_info;
        bingo_hw_manager_dep_check_info_t            dep_check_info;
        bingo_hw_manager_assigned_core_id_t          assigned_core_id;
        bingo_hw_manager_assigned_cluster_id_t       assigned_cluster_id;
        bingo_hw_manager_assigned_chiplet_id_t       assigned_chiplet_id;
        bingo_hw_manager_task_id_t                   task_id;
        bingo_hw_manager_task_type_t                 task_type;
        // DARTS Tier 1: Conditional Execution
        logic                                        cond_exec_en;
        logic [4:0]                                  cond_exec_group_id;
        logic                                        cond_exec_invert;
    } bingo_hw_manager_task_desc_full_t;

    function automatic logic [WdClassWidth-1:0] get_class(
        input bingo_hw_manager_task_desc_full_t f);
        return WdClassAvail ? WdClassWidth'(f.reserved_bits) : '0;
    endfunction

    function automatic bingo_hw_manager_task_desc_full_t set_class(
        input bingo_hw_manager_task_desc_full_t f,
        input logic [WdClassWidth-1:0] wd_class);
        bingo_hw_manager_task_desc_full_t result;
        result = f;
        if (WdClassAvail)
            result.reserved_bits =
                ((f.reserved_bits >> WdClassWidth) << WdClassWidth) | wd_class;
        return result;
    endfunction

    if (!WdClassAvail) begin : gen_wd_class
`ifndef SYNTHESIS
        initial $info("A7 class unavailable: %0d reserved bits", ReservedBitsForTaskDesc);
`endif
    end

    // Level 3 remote dispatch
    localparam bit RemoteEn = SubstituteLevelMask[2];
    if (RemoteEn && (READY_AND_DONE_QUEUE_INTERFACE_TYPE != 1)) begin : gen_remote_intf_check
        initial begin
        $error("SubstituteLevelMask[2] (remote dispatch) needs READY_AND_DONE_QUEUE_INTERFACE_TYPE = 1 (CSR)");
        $finish;
        end
    end
    // bingo_hw_manager_remote_link only carries task_type and task_id of the
    // descriptor and finds them at fixed bit positions (its DescTaskTypeLsb /
    // DescTaskIdLsb defaults): keep them in sync with the layout above
    localparam int unsigned RemoteLinkDescTaskTypeLsb = 7;
    localparam int unsigned RemoteLinkDescTaskIdLsb   = 9;
    if (RemoteEn) begin : gen_remote_desc_check
        initial begin
            automatic bingo_hw_manager_task_desc_full_t probe = '0;
            automatic host_axi_lite_data_t expected = '0;
            probe.task_type = '1;
            probe.task_id   = '1;
            expected[RemoteLinkDescTaskTypeLsb +: 2]         = '1;
            expected[RemoteLinkDescTaskIdLsb +: TaskIdWidth] = '1;
            if (($bits(bingo_hw_manager_task_type_t) != 2) || (host_axi_lite_data_t'(probe) != expected)) begin
                $error("Descriptor layout does not match bingo_hw_manager_remote_link (task_type at %0d, task_id at %0d)",
                       RemoteLinkDescTaskTypeLsb, RemoteLinkDescTaskIdLsb);
                $finish;
            end
        end
    end
    typedef logic [RemoteSlotIdWidth-1:0] remote_slot_t;
    // Side tag of each checkout entry (only stored with RemoteEn)
    typedef struct packed{
        logic                                    exported;    // proxy entry: runs on another chiplet
        logic                                    imported;    // runs here for another chiplet
        bingo_hw_manager_assigned_chiplet_id_t   origin_chip; // imported: origin chiplet
        remote_slot_t                            proxy_slot;  // imported: proxy slot on the origin chiplet
    } remote_tag_t;
    typedef struct packed{
        bingo_hw_manager_task_desc_full_t        desc;
        logic [CoreTypeIdWidth-1:0]              core_type;
        remote_slot_t                            proxy_slot;
    } remote_export_t;
    typedef struct packed{
        bingo_hw_manager_assigned_chiplet_id_t   chip;
        remote_slot_t                            proxy_slot;
        bingo_hw_manager_task_id_t               task_id;
        logic                                    reject;
    } remote_done_t;

    function automatic bingo_hw_manager_task_desc_full_t desc_to_full(input bingo_hw_manager_task_desc_t d);
        bingo_hw_manager_task_desc_full_t f;
        f = '0;
        f.is_exit             = d.is_exit;
        f.dep_set_info        = d.dep_set_info;
        f.dep_check_info      = d.dep_check_info;
        f.assigned_core_id    = d.assigned_core_id;
        f.assigned_cluster_id = d.assigned_cluster_id;
        f.assigned_chiplet_id = d.assigned_chiplet_id;
        f.task_id             = d.task_id;
        f.task_type           = d.task_type;
        f.cond_exec_en        = d.cond_exec_en;
        f.cond_exec_group_id  = d.cond_exec_group_id;
        f.cond_exec_invert    = d.cond_exec_invert;
        return set_class(f, d.wd_class);
    endfunction

    function automatic bingo_hw_manager_task_desc_t full_to_desc(input bingo_hw_manager_task_desc_full_t f);
        bingo_hw_manager_task_desc_t d;
        d.wd_class            = get_class(f);
        d.is_exit             = f.is_exit;
        d.dep_set_info        = f.dep_set_info;
        d.dep_check_info      = f.dep_check_info;
        d.assigned_core_id    = f.assigned_core_id;
        d.assigned_cluster_id = f.assigned_cluster_id;
        d.assigned_chiplet_id = f.assigned_chiplet_id;
        d.task_id             = f.task_id;
        d.task_type           = f.task_type;
        d.cond_exec_en        = f.cond_exec_en;
        d.cond_exec_group_id  = f.cond_exec_group_id;
        d.cond_exec_invert    = f.cond_exec_invert;
        return d;
    endfunction

    // Done info struct
    typedef struct packed{
        bingo_hw_manager_assigned_cluster_id_t     assigned_cluster_id;
        bingo_hw_manager_assigned_core_id_t        assigned_core_id;
        bingo_hw_manager_task_id_t                 task_id;
    } bingo_hw_manager_done_info_t;

    localparam int unsigned DoneInfoWidth = $bits(bingo_hw_manager_done_info_t);
    localparam int unsigned ReservedBitsForDoneInfo = DeviceAxiLiteDataWidth - DoneInfoWidth;
    if (DoneInfoWidth>DeviceAxiLiteDataWidth) begin : gen_done_info_width_check
        initial begin
        $error("Task Decriptor width (%0d) exceeds Device AXI Lite Data Width (%0d)! Please adjust the parameters accordingly.", DoneInfoWidth, DeviceAxiLiteDataWidth);
        $finish;
        end
    end

    typedef struct packed{
        logic [ReservedBitsForDoneInfo-1:0]        reserved_bits;
        bingo_hw_manager_assigned_cluster_id_t     assigned_cluster_id;
        bingo_hw_manager_assigned_core_id_t        assigned_core_id;
        bingo_hw_manager_task_id_t                 task_id;
    } bingo_hw_manager_done_info_full_t;

    typedef struct packed{
        bingo_hw_manager_assigned_cluster_id_t     dep_matrix_id;
        bingo_hw_manager_assigned_core_id_t        dep_matrix_col;
        bingo_hw_manager_dep_tag_t                 dep_matrix_set_tag;
        bingo_hw_manager_dep_code_t                dep_set_code;
    } bingo_hw_manager_dep_matrix_set_meta_t;

    typedef struct packed{
        bingo_hw_manager_task_id_t           task_id;
    } bingo_hw_manager_ready_task_desc_t;
    // Check the width
    localparam int unsigned ReadyTaskDescWidth = $bits(bingo_hw_manager_ready_task_desc_t);
    localparam int unsigned ReservedBitsForReadyTaskDesc = DeviceAxiLiteDataWidth - ReadyTaskDescWidth;
    if (ReadyTaskDescWidth>DeviceAxiLiteDataWidth) begin : gen_ready_task_desc_width_check
        initial begin
        $error("Ready Task Decriptor width (%0d) exceeds Device AXI Lite Data Width (%0d)! Please adjust the parameters accordingly.", ReadyTaskDescWidth, DeviceAxiLiteDataWidth);
        $finish;
        end
    end
    typedef struct packed{
        logic [ReservedBitsForReadyTaskDesc-1:0] reserved_bits;
        bingo_hw_manager_task_id_t           task_id;
    } bingo_hw_manager_ready_task_desc_full_t;
    //----- End of Type definitions ------------------------------------//

    //----- Start of Signal declarations -------------------------------//

    /////////////////////////////////////////////////////////
    // Task Queue Signals
    /////////////////////////////////////////////////////////
    // The task queue holds the tasks to be scheduled to the devices
    bingo_hw_manager_task_desc_full_t  cur_task_desc_full;
    bingo_hw_manager_task_desc_t       cur_task_desc;
    logic [HostAxiLiteDataWidth-1:0]   task_queue_mbox_data;
    logic                              task_queue_mbox_empty;
    logic                              task_queue_mbox_pop;
    logic                              fetch_bus_error;   // master mode: a descriptor read failed
    logic [1:0]                        pm_bus_error;


    /////////////////////////////////////////////////////////
    // Chiplet Dep Set Issue
    /////////////////////////////////////////////////////////
    // This module is to send the chiplet dep set signal to other chiplets
    // It will receive the chiplet dep set task from the wait dep check queues
    bingo_hw_manager_task_desc_full_t chiplet_dep_set_task_desc;
    logic                             chiplet_dep_set_task_desc_valid;
    logic                             chiplet_dep_set_task_desc_ready;

    //////////////////////////////////////////////////////////
    // Stream Arbiter Chiplet Dep Set Issue Signals
    //////////////////////////////////////////////////////////
    // The inputs are from the checkout queues of all cores in the chiplet
    bingo_hw_manager_task_desc_full_t [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] stream_arbiter_chiplet_dep_set_inp_task_desc;
    logic                             [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] stream_arbiter_chiplet_dep_set_inp_valid;
    logic                             [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] stream_arbiter_chiplet_dep_set_inp_ready;
    bingo_hw_manager_task_desc_full_t                                                      stream_arbiter_chiplet_dep_set_oup_task_desc;
    logic                                                                                  stream_arbiter_chiplet_dep_set_oup_valid;
    logic                                                                                  stream_arbiter_chiplet_dep_set_oup_ready;


    //////////////////////////////////////////////////////////
    // Chiplet Done Queue
    //////////////////////////////////////////////////////////
    logic [HostAxiLiteDataWidth-1:0]   chiplet_done_queue_mbox_data;
    logic                              chiplet_done_queue_mbox_empty;
    logic                              chiplet_done_queue_mbox_pop;
    bingo_hw_manager_task_desc_full_t  cur_chiplet_done_queue_task_desc;
    /////////////////////////////////////////////////////////
    // Stream demux core type
    /////////////////////////////////////////////////////////
    logic                                           stream_demux_core_type_inp_valid;
    logic                                           stream_demux_core_type_inp_ready;
    logic [cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)-1:0]       stream_demux_core_type_oup_sel;
    logic [NUM_CORES_PER_CLUSTER-1:0]               stream_demux_core_type_oup_valid;
    logic [NUM_CORES_PER_CLUSTER-1:0]               stream_demux_core_type_oup_ready;

    ///////////////////////////////////
    // Waiting dep check queue signals
    ///////////////////////////////////
    bingo_hw_manager_task_desc_t      [NUM_CORES_PER_CLUSTER-1:0] waiting_dep_check_task_desc;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] waiting_dep_check_queue_push;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] waiting_dep_check_queue_full;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] waiting_dep_check_queue_empty;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] waiting_dep_check_queue_pop;

    ////////////////////////////////
    // Dep Check Manager Signals
    ////////////////////////////////
    logic                             [NUM_CORES_PER_CLUSTER-1:0] dep_check_manager_inp_wait_dep_check_queue_valid;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] dep_check_manager_inp_wait_dep_check_queue_ready;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] dep_check_manager_oup_dep_check_valid;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] dep_check_manager_oup_dep_check_ready;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] dep_check_manager_oup_ready_and_checkout_queue_valid;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] dep_check_manager_oup_ready_and_checkout_queue_ready;
    ////////////////////////////////
    // Dep matrix demux signals
    ////////////////////////////////
    typedef logic [NUM_CLUSTERS_PER_CHIPLET-1:0] dep_matrix_demux_oup_t;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] demux_dep_matrix_inp_valid;
    logic                             [NUM_CORES_PER_CLUSTER-1:0] demux_dep_matrix_inp_ready;
    dep_matrix_demux_oup_t            [NUM_CORES_PER_CLUSTER-1:0] demux_dep_matrix_oup_valid;
    dep_matrix_demux_oup_t            [NUM_CORES_PER_CLUSTER-1:0] demux_dep_matrix_oup_ready;

    ////////////////////////////////
    // Ready and Checkout queue demux signals
    ////////////////////////////////
    typedef logic [NUM_CLUSTERS_PER_CHIPLET-1:0] ready_and_checkout_queue_demux_oup_t;
    logic                                          [NUM_CORES_PER_CLUSTER-1:0] demux_ready_and_checkout_queue_inp_valid;
    logic                                          [NUM_CORES_PER_CLUSTER-1:0] demux_ready_and_checkout_queue_inp_ready;
    ready_and_checkout_queue_demux_oup_t           [NUM_CORES_PER_CLUSTER-1:0] demux_ready_and_checkout_queue_oup_valid;
    ready_and_checkout_queue_demux_oup_t           [NUM_CORES_PER_CLUSTER-1:0] demux_ready_and_checkout_queue_oup_ready;

    //////////////////////
    // Dep matrix signals
    //////////////////////
    typedef logic [NUM_CORES_PER_CLUSTER-1:0] dep_check_code_t;
    typedef logic [NUM_CORES_PER_CLUSTER-1:0] dep_set_code_t;

    logic [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0]            dep_check_valid;
    logic [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0]            dep_check_result;
    dep_check_code_t [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0] dep_check_code;
    bingo_hw_manager_dep_tag_t [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0] dep_check_tag;
    logic [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0]            dep_set_valid;
    logic [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0]            dep_set_ready;
    dep_set_code_t [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0]   dep_set_code;
    bingo_hw_manager_dep_tag_t [NUM_CLUSTERS_PER_CHIPLET-1:0][NUM_CORES_PER_CLUSTER-1:0] dep_set_tag;

    ///////////////////////////////////////
    // Stream Arbiter Dep Matrix Set
    ///////////////////////////////////////
    // There are two types input streams to set the dep matrix
    // Type 1: From Checkout queues (NUM_CORE * NUM_Cluster) for normal and dummy set dep
    // Type 2: From Chiplet Dep Set Recv Queue for chiplet dep set queues
    // In total we have (NUM_CORE * NUM_Cluster) + 1 inputs for the dep matrix set
    localparam int unsigned STREAM_ARBITER_DEP_MATRIX_SET_NUM_INP = NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET + 1;
    bingo_hw_manager_dep_matrix_set_meta_t    [STREAM_ARBITER_DEP_MATRIX_SET_NUM_INP-1:0] stream_arbiter_dep_matrix_set_inp_data;
    logic                                     [STREAM_ARBITER_DEP_MATRIX_SET_NUM_INP-1:0] stream_arbiter_dep_matrix_set_inp_valid;
    logic                                     [STREAM_ARBITER_DEP_MATRIX_SET_NUM_INP-1:0] stream_arbiter_dep_matrix_set_inp_ready;
    bingo_hw_manager_dep_matrix_set_meta_t                                                stream_arbiter_dep_matrix_set_oup_data;
    logic                                                                                 stream_arbiter_dep_matrix_set_oup_valid;
    logic                                                                                 stream_arbiter_dep_matrix_set_oup_ready;
 
    ///////////////////////////////////////
    // Stream Demux Set Dep Matrix Cluster ID
    ///////////////////////////////////////
    // Possbile to move the demux before the arbiter to support more parallelism
    logic                                                          stream_demux_set_dep_matrix_cluster_id_inp_valid;
    logic                                                          stream_demux_set_dep_matrix_cluster_id_inp_ready;
    logic  [cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET)-1:0]  stream_demux_set_dep_matrix_cluster_id_oup_sel;
    logic  [NUM_CLUSTERS_PER_CHIPLET-1:0]                          stream_demux_set_dep_matrix_cluster_id_oup_valid;
    logic  [NUM_CLUSTERS_PER_CHIPLET-1:0]                          stream_demux_set_dep_matrix_cluster_id_oup_ready;
    ///////////////////////////////////////
    // Stream Demux Set Dep Matrix Core ID
    ///////////////////////////////////////
    typedef logic [cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)-1:0]             stream_demux_set_dep_matrix_core_id_oup_sel_t;
    typedef logic [NUM_CORES_PER_CLUSTER-1:0]                                     stream_demux_set_dep_matrix_core_id_oup_t;
    logic                                          [NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_set_dep_matrix_core_id_inp_valid;
    logic                                          [NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_set_dep_matrix_core_id_inp_ready;
    stream_demux_set_dep_matrix_core_id_oup_sel_t  [NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_set_dep_matrix_core_id_oup_sel;
    stream_demux_set_dep_matrix_core_id_oup_t      [NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_set_dep_matrix_core_id_oup_valid;
    stream_demux_set_dep_matrix_core_id_oup_t      [NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_set_dep_matrix_core_id_oup_ready;


    //////////////////////
    // Ready queue signals
    //////////////////////
    // Ready task info
    bingo_hw_manager_ready_task_desc_full_t [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_data_in;
    logic                                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_push;
    logic                                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_full;
    // ready queue data_o/empty_o/pop_i signals are only for CSR interface
    logic                                    [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_pop;
    bingo_hw_manager_ready_task_desc_full_t  [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_data_out;
    logic                                    [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_empty;


    //////////////////////
    // Checkout queue signals
    //////////////////////
    bingo_hw_manager_task_desc_t   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_queue_data_out;
    bingo_hw_manager_task_desc_t   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_queue_data_in;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_queue_push;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_queue_pop;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_queue_full;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_queue_empty;
    // Level 3: one spare entry lets a replay rotation (pop + push) run on a full queue
    localparam int unsigned CheckoutFifoDepth = CheckoutQueueDepth + (RemoteEn ? 1 : 0);
    localparam int unsigned CheckoutUsageWidth = (CheckoutFifoDepth > 1) ? $clog2(CheckoutFifoDepth) : 1;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_queue_full_raw;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][CheckoutUsageWidth-1:0] checkout_queue_usage;
    remote_tag_t                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_remote_tag_in;
    remote_tag_t                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_remote_tag_out;
    // Checkout head imported from another chiplet: retires into the remote done stream
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_head_imported;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] remote_head_mismatch;
    // The head may retire this cycle (bingo_hw_manager_checkout_queue), and the
    // retiring head offered to the local dependency matrix or to another chiplet
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_retire_valid;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_local_set_valid;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_chiplet_set_valid;
    // Level 3 export (origin side)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]         route_remote_grant;
    logic                                  replay_rotate;
    logic                                  replay_export;
    logic                                  replay_bounce;
    remote_export_t                        export_in;
    logic                                  export_push;
    // Per core type: its export queue has room
    logic [2**CoreTypeIdWidth-1:0]         export_type_ready;
    // Level 3 import (executor side)
    // Level 3 remote dones
    remote_done_t [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] remote_done_arb_data;
    logic         [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] remote_done_arb_valid;
    logic         [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] remote_done_arb_ready;
    remote_done_t                                                      remote_done_out;
    // Level 3 rejects: executor side (one pending reject), origin side (sticky
    // per proxy slot)
    logic                                                              reject_pending;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    remote_rejected_q;

    ///////////////////////////////////////
    // Per (Core, Cluster) Done Queue signals
    // Each (core, cluster) pair has its own done queue FIFO.
    // This fully eliminates HOL blocking: completions for different
    // cores AND different clusters drain independently.
    ///////////////////////////////////////
    bingo_hw_manager_done_info_full_t [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] done_q_info;
    logic                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] done_q_pop;
    logic                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] done_q_empty;
    bingo_hw_manager_done_info_full_t [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] done_q_data_in;
    logic                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] done_q_push;
    ///////////////////////////////////////
    // DARTS Tier 1: CERF state and per-core conditional skip signals
    logic [31:0] cerf_state;
    assign cerf_state_o = cerf_state;  // read-back for SW
    logic [NUM_CORES_PER_CLUSTER-1:0] cond_exec_skip;

    // DARTS CERF: per-core conditional skip evaluation.
    // Only valid when there IS a task being processed (queue not empty).
    // When cond_exec_en==0 (default), this is always 0 regardless of CERF state.
    for (genvar c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin: gen_cerf_skip
        logic cerf_group_active_for_core;
        assign cerf_group_active_for_core = cerf_state[waiting_dep_check_task_desc[c].cond_exec_group_id];
        assign cond_exec_skip[c] = !waiting_dep_check_queue_empty[c] &&
                                    waiting_dep_check_task_desc[c].cond_exec_en &&
                                    (waiting_dep_check_task_desc[c].cond_exec_invert ?
                                        cerf_group_active_for_core : !cerf_group_active_for_core);
    end

    // Core status signals
    ///////////////////////////////////////
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] heartbeat_valid;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_status_waiting_task;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] exited_q;
    // Port structs of slot vectors (bingo_hw_manager/typedef.svh), indexed
    // [core][cluster] like the nets of this module
    `BINGO_HW_MANAGER_TYPEDEF_SLOT_STATE_T(slot_state_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_SMT_T(smt_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET,
        cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER), cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET))
    `BINGO_HW_MANAGER_TYPEDEF_PARK_T(park_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_CTRL_QUEUE_T(ctrl_queue_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET,
        CheckoutUsageWidth + 1)
    `BINGO_HW_MANAGER_TYPEDEF_CTRL_HOST_T(ctrl_host_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_CTRL_VIEW_T(ctrl_view_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_TYPE_CFG_T(type_cfg_t, 2**CoreTypeIdWidth)
    `BINGO_HW_MANAGER_TYPEDEF_CERF_FB_T(cerf_fb_t, CoreTypeIdWidth)
    `BINGO_HW_MANAGER_TYPEDEF_WAITING_T(waiting_t, bingo_hw_manager_task_desc_t, NUM_CORES_PER_CLUSTER)
    `BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_T(checkout_t, bingo_hw_manager_task_desc_t, remote_tag_t,
        NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET, CheckoutUsageWidth)
    `BINGO_HW_MANAGER_TYPEDEF_READY_T(ready_t, bingo_hw_manager_ready_task_desc_full_t,
        NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET, WdClassWidth)
    `BINGO_HW_MANAGER_TYPEDEF_SELECT_T(select_t, NUM_CORES_PER_CLUSTER,
        cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER), cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET))
    `BINGO_HW_MANAGER_TYPEDEF_ROUTE_T(route_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET,
        cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER))
    `BINGO_HW_MANAGER_TYPEDEF_DONE_T(done_t, bingo_hw_manager_done_info_full_t,
        NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_DONE_REMOTE_T(done_remote_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_CSR_DONE_T(csr_done_t, bingo_hw_manager_done_info_full_t)
    `BINGO_HW_MANAGER_TYPEDEF_CORE_STATUS_T(core_status_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_REPLAY_HEAD_T(replay_head_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET,
        cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER), cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET))
    `BINGO_HW_MANAGER_TYPEDEF_REPLAY_STEP_T(replay_step_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET,
        cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER), cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET))
    `BINGO_HW_MANAGER_TYPEDEF_REPLAY_OP_T(replay_op_t, bingo_hw_manager_task_desc_t,
        NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_REMOTE_DISPATCH_T(remote_dispatch_out_t, bingo_hw_manager_task_desc_full_t,
        chip_id_t, remote_slot_t, CoreTypeIdWidth)
    `BINGO_HW_MANAGER_TYPEDEF_REMOTE_DISPATCH_T(remote_dispatch_in_t, bingo_hw_manager_task_desc_t,
        chip_id_t, remote_slot_t, CoreTypeIdWidth)
    `BINGO_HW_MANAGER_TYPEDEF_PROXY_T(proxy_t, NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET)
    `BINGO_HW_MANAGER_TYPEDEF_IMPORT_T(import_t, bingo_hw_manager_task_desc_t, remote_tag_t,
        NUM_CORES_PER_CLUSTER, NUM_CLUSTERS_PER_CHIPLET,
        cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER), cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET))
    // ... and of one slot's queues
    `BINGO_HW_MANAGER_TYPEDEF_TASK_PUSH_T(task_push_t, bingo_hw_manager_task_desc_t)
    `BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_ROUTE_T(checkout_route_t, bingo_hw_manager_task_desc_t)
    `BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_REPLAY_T(checkout_replay_t, bingo_hw_manager_task_desc_t, remote_tag_t)
    `BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_IMPORT_T(checkout_import_t, bingo_hw_manager_task_desc_t, remote_tag_t)
    `BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_ENTRY_T(checkout_entry_t, bingo_hw_manager_task_desc_t, remote_tag_t)
    `BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_OCC_T(checkout_occ_t, CheckoutUsageWidth)
    `BINGO_HW_MANAGER_TYPEDEF_CHECKOUT_HEAD_T(checkout_head_t, bingo_hw_manager_task_desc_t, remote_tag_t)
    `BINGO_HW_MANAGER_TYPEDEF_READY_PUSH_T(ready_push_t, bingo_hw_manager_ready_task_desc_full_t)
    `BINGO_HW_MANAGER_TYPEDEF_READY_HEAD_T(ready_head_t, bingo_hw_manager_ready_task_desc_full_t, WdClassWidth)
    // Control plane (bingo_hw_manager_ctrl): what it reads of every slot, and
    // its view of every slot (PM idle, boost, derate, watchdog tick, risk, ...)
    slot_state_t                                                    slot_state;
    ctrl_view_t                                                     ctrl_view;
    // Late beats (bingo_hw_manager_watchdog)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] wd_late;
    logic [31:0][7:0]                                               pm_domain_level;
    // Power state and configured levels, as the control plane and the
    // power-domain counters read them
    pm_view_t                                                       pm_view;
    assign pm_view = '{enable:       bingo_hw_manager_enable_idle_pm_i[0],
                       dvfs:         bingo_hw_manager_pm_mode_i[0],
                       idle_level:   bingo_hw_manager_idle_power_level_i[7:0],
                       normal_level: bingo_hw_manager_normal_power_level_i[7:0],
                       boost_level:  bingo_hw_manager_boost_power_level_i[7:0],
                       dvfs_level:   bingo_hw_manager_dvfs_ack_i[7:0],
                       domain_level: pm_domain_level};
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][5:0] ctrl_slot_domain;
    // Power domain of each slot (PmMaxDomains: none), decoded once for the PM,
    // the control plane and the power-domain counters
    for (genvar c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin : gen_slot_domain_core
        for (genvar cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin : gen_slot_domain_cluster
            assign ctrl_slot_domain[c][cl] = (bingo_hw_manager_core_power_domain_i[c][cl] < PmMaxDomains) ?
                6'(bingo_hw_manager_core_power_domain_i[c][cl]) : 6'(PmMaxDomains);
        end
    end
    logic [31:0] pm_access_level;
    // Validate all 32 bits before the PM narrows the divider to eight bits.
    assign pm_access_level =
        (bingo_hw_manager_pm_access_level_i > 32'(bingo_hw_manager_normal_power_level_i[7:0]) &&
         bingo_hw_manager_pm_access_level_i < 32'(bingo_hw_manager_idle_power_level_i[7:0])) ?
        bingo_hw_manager_pm_access_level_i : 32'd0;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_busy;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_available;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_dead_suspect;
    // Confirmed dead (sticky): the slot's ready reads, dones and heartbeats are ignored
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_fenced;
    // Fenced and all its outstanding tasks moved to other cores (sticky)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_retired;
    // Checkout queue head executes on a core (normal / gating): retires with its done
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_head_exec;
    // Stuck or rejected slot whose head may still retire without running: its
    // own logical core, nothing of that core elsewhere, not imported
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stuck_drain;
    // ... and the head is a task the current CERF skips (retires as skipped)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stuck_head_skip;

    // Replay (see bingo_hw_manager_replay_ctrl)
    logic                                replay_pending;      // a slot is fenced, neither retired nor stuck
    logic [NUM_CLUSTERS_PER_CHIPLET-1:0] replay_move_cluster; // a MOVE step may push into the cluster
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_ready_flush;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_move;
    // Checkout output held by the replay controller (being moved or partly moved)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_hold_slot;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_pop;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_push;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_push_ready;
    // The checkout heads as the replay controller sees them (bingo_hw_manager_replay_path)
    replay_head_t                          replay_head;
    logic                                  replay_exit_absorb;
    smt_t                                  smt;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] exit_absorb;
    logic                                  replay_move_fire;
    logic                                  replay_push_ready_q;
    bingo_hw_manager_assigned_core_id_t    replay_src_core;
    bingo_hw_manager_assigned_cluster_id_t replay_src_cluster;
    bingo_hw_manager_assigned_core_id_t    replay_dst_core;
    bingo_hw_manager_assigned_cluster_id_t replay_dst_cluster;
    // Fenced slot that no live core can take over (sticky); replay_stuck: any
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_stuck_slot;
    logic                                  replay_stuck;
    bingo_hw_manager_task_desc_t           replay_data;
    // Fenced slots whose first executing entry was moved (for top_assert)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_exec_moved;

    // Placement of the waiting queue heads (bingo_hw_manager_router): per
    // logical core its physical slot, per physical slot the routed task
    select_t                               remap_select;
    route_t                                route;

    // Number of tasks of logical core [core] (cluster [cluster]) that were remapped
    // to another physical core and are still in that core's checkout queue.
    localparam int unsigned RemapOutstandingWidth =
        $clog2(NUM_CORES_PER_CLUSTER * CheckoutQueueDepth + 1);
    logic [RemapOutstandingWidth-1:0] remap_outstanding_q [NUM_CORES_PER_CLUSTER][NUM_CLUSTERS_PER_CHIPLET];
    // Of those, the ones in the checkout queue of physical slot [p][pcl]:
    // moved_in_q[p][pcl][core][cluster] (ForeignStuckDrain only)
    logic [RemapOutstandingWidth-1:0] moved_in_q [NUM_CORES_PER_CLUSTER][NUM_CLUSTERS_PER_CHIPLET][NUM_CORES_PER_CLUSTER][NUM_CLUSTERS_PER_CHIPLET];

    // Watchdog counter wide enough for the configured timeouts (saturating timer).
    localparam int unsigned WatchdogMaxTimeoutCycles =
        (WatchdogConfirmTimeoutCycles > WatchdogHeartbeatTimeoutCycles) ?
        WatchdogConfirmTimeoutCycles : WatchdogHeartbeatTimeoutCycles;
    localparam int unsigned WatchdogCounterWidth = $clog2(WatchdogMaxTimeoutCycles + 1) + 1;
    logic [WatchdogCounterWidth-1:0] wd_late_cycles;  // late-beat threshold of the watchdog
    // Not read by the design (each slot uses wd_risk_confirm_valid below): the value
    // tb_bingo_hw_manager_watchdog_risk_confirm compares the effective threshold with
    logic [WatchdogCounterWidth-1:0] wd_risk_confirm;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][WatchdogCounterWidth-1:0] wd_suspect_thr;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][WatchdogCounterWidth-1:0] wd_confirm_thr;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][31:0] wd_h_eff, wd_c_eff;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][31:0] wd_h_ad, wd_c_ad;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] wd_h_ad_valid, wd_c_ad_valid;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] wd_adapt_confirm, wd_risk_use;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] wd_progress;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][WatchdogCounterWidth-1:0] wd_interval;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][31:0] wd_late_ad;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] wd_late_ad_valid, wd_late_ad_use;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][WatchdogCounterWidth-1:0] wd_late_slot;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][WdClassWidth-1:0] wd_class_q, ready_class;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] wd_moved_q, ready_moved;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] wd_risk_confirm_valid;
    // The waiting queue heads, and the checkout, ready and done queues of every slot
    waiting_t  waiting_heads;
    checkout_t checkout_queue;
    ready_t    ready_queue;
    done_t     done_queue;
    assign waiting_heads  = '{desc:  waiting_dep_check_task_desc,
                              empty: waiting_dep_check_queue_empty,
                              skip:  cond_exec_skip};
    assign checkout_queue = '{push:          checkout_queue_push,
                              push_desc:     checkout_queue_data_in,
                              push_tag:      checkout_remote_tag_in,
                              exit_absorb:   exit_absorb,
                              pop:           checkout_queue_pop,
                              full:          checkout_queue_full,
                              full_raw:      checkout_queue_full_raw,
                              usage:         checkout_queue_usage,
                              empty:         checkout_queue_empty,
                              head:          checkout_queue_data_out,
                              head_tag:      checkout_remote_tag_out,
                              head_imported: checkout_head_imported,
                              head_exec:     checkout_head_exec};
    assign ready_queue    = '{push:      ready_queue_push,
                              push_data: ready_queue_data_in,
                              full:      ready_queue_full,
                              empty:     ready_queue_empty,
                              pop:       ready_queue_pop,
                              data:      ready_queue_data_out,
                              wd_class:  ready_class,
                              moved:     ready_moved};
    // The done queues drive done_queue; the nets of each field keep their names
    assign done_q_push    = done_queue.push;
    assign done_q_data_in = done_queue.push_data;
    assign done_q_info    = done_queue.head;
    assign done_q_empty   = done_queue.empty;
    // Level 3: what the remote dones and rejects of exported tasks do to the
    // done queues, and the remote done input as a remote_done_t (its chip is
    // this one)
    done_remote_t done_remote;
    // Level 3, executor side: the import taken this cycle (bingo_hw_manager_remote_import)
    import_t      imported;
    remote_done_t remote_done_in;
    assign remote_done_in = '{chip:       '0,
                              proxy_slot: remote_done_proxy_slot_i,
                              task_id:    remote_done_task_id_i,
                              reject:     remote_done_reject_i};
    // --------Finish Type definitions and signal declarations--------------------//

    // --------Module initializations---------------------------------------------//

    //////////////////////////////////////////////////////////////////////
    // Task Queue
    /////////////////////////////////////////////////////////////////////
    if (TASK_QUEUE_TYPE == 0 ) begin : gen_bingo_hw_manager_task_queue_default_slave
        // Default AXI Lite Slave Task Queue
        bingo_hw_manager_write_mailbox #(
            .MailboxDepth(TaskQueueDepth               ),
            .IrqEdgeTrig (1'b0                         ),
            .IrqActHigh  (1'b1                         ),
            .AxiAddrWidth(HostAxiLiteAddrWidth         ),
            .AxiDataWidth(HostAxiLiteDataWidth         ),
            .ChipIdWidth (ChipIdWidth                  ),
            .req_lite_t  (host_axi_lite_req_t          ),
            .resp_lite_t (host_axi_lite_resp_t         )
        ) i_bingo_hw_manager_task_queue_slave (
            .clk_i       (clk_i                     ),
            .rst_ni      (rst_ni                    ),
            .chip_id_i   (chip_id_i                 ),
            .test_i      (1'b0                      ),
            .req_i       (task_queue_axi_lite_req_i ),
            .resp_o      (task_queue_axi_lite_resp_o),
            .irq_o       (/*not used*/              ),
            .base_addr_i (task_queue_base_addr_i    ),
            .mbox_data_o (task_queue_mbox_data      ),
            .mbox_pop_i  (task_queue_mbox_pop       ),
            .mbox_empty_o(task_queue_mbox_empty     ),
            .mbox_flush_i('0                        )
        );
        // Tie off the unused master interface signals
        assign task_queue_axi_lite_req_o = '0;
        assign bingo_hw_manager_reset_start_o    = '0;
        assign bingo_hw_manager_reset_start_en_o = 1'b0;
        assign fetch_bus_error                   = 1'b0;
    end
    else begin : gen_bingo_hw_manager_task_queue_master
        // AXI Lite Master Task Queue
        // The Hw Manager issues the read request to the address specified by the host via the following inputs
        // Hence this is a master AXI Lite interface
        bingo_hw_manager_task_queue_master #(
            .TaskQueueDepth               (TaskQueueDepth               ),
            .TaskIdWidth                  (TaskIdWidth                  ),
            .req_lite_t                   (host_axi_lite_req_t          ),
            .resp_lite_t                  (host_axi_lite_resp_t         ),
            .addr_t                       (host_axi_lite_addr_t         ),
            .data_t                       (host_axi_lite_data_t         )
        ) i_bingo_hw_manager_task_queue_master (
            .clk_i                     (clk_i                                ),
            .rst_ni                    (rst_ni                               ),
            .task_list_base_addr_i     (task_list_base_addr_i                ),
            .num_task_i                (num_task_i                           ),
            .start_i                   (bingo_hw_manager_start_i             ),
            .reset_start_o             (bingo_hw_manager_reset_start_o       ),
            .reset_start_en_o          (bingo_hw_manager_reset_start_en_o    ),
            .task_queue_axi_lite_req_o (task_queue_axi_lite_req_o            ),
            .task_queue_axi_lite_resp_i(task_queue_axi_lite_resp_i           ),
            .task_queue_data_o         (task_queue_mbox_data                 ),
            .task_queue_pop_i          (task_queue_mbox_pop                  ),
            .task_queue_empty_o        (task_queue_mbox_empty                ),
            .bus_error_o               (fetch_bus_error                      )
        );
        // Tie off the unused slave interface signals
        assign task_queue_axi_lite_resp_o = '0;
    end
    //////////////////////////////////////////////////////////////////////
    // Task queue → demux (direct connection, no mux needed)
    //////////////////////////////////////////////////////////////////////
    host_axi_lite_data_t muxed_task_data;
    logic                muxed_task_valid;

    assign muxed_task_data  = task_queue_mbox_data;
    assign muxed_task_valid = !task_queue_mbox_empty;
    assign task_queue_mbox_pop = stream_demux_core_type_inp_ready && !task_queue_mbox_empty;

    // Compose the current task descriptor from the muxed source
    assign cur_task_desc_full = bingo_hw_manager_task_desc_full_t'(muxed_task_data);
    assign cur_task_desc = full_to_desc(cur_task_desc_full);


    /////////////////////////////////////////////////////////
    // H2H Dep Set Interface
    /////////////////////////////////////////////////////////       
    bingo_hw_manager_chiplet_dep_set #(
        .ChipIdWidth                                  (ChipIdWidth            ),
        .HostAxiLiteAddrWidth                         (HostAxiLiteAddrWidth   ),
        .HostAxiLiteDataWidth                         (HostAxiLiteDataWidth   ),
        .host_axi_lite_req_t                          (host_axi_lite_req_t    ),
        .host_axi_lite_resp_t                         (host_axi_lite_resp_t   ),
        .bingo_hw_manager_task_desc_full_t            (bingo_hw_manager_task_desc_full_t)
    ) i_bingo_hw_manager_chiplet_dep_set (
        .clk_i                             (clk_i                              ),
        .rst_ni                            (rst_ni                             ),
        .chiplet_mailbox_base_addr_i       (chiplet_mailbox_base_addr_i        ),
        .to_remote_chiplet_axi_lite_req_o  (to_remote_chiplet_axi_lite_req_o   ),
        .to_remote_chiplet_axi_lite_resp_i (to_remote_chiplet_axi_lite_resp_i  ),
        .chiplet_dep_set_task_desc_i       (chiplet_dep_set_task_desc          ),
        .chiplet_dep_set_task_desc_valid_i (chiplet_dep_set_task_desc_valid    ),
        .chiplet_dep_set_task_desc_ready_o (chiplet_dep_set_task_desc_ready    )
    );
    assign chiplet_dep_set_task_desc = stream_arbiter_chiplet_dep_set_oup_task_desc;
    assign chiplet_dep_set_task_desc_valid = stream_arbiter_chiplet_dep_set_oup_valid;

    /////////////////////////////////////////////////////////
    // Stream Arbiter for Chiplet Dep Set
    /////////////////////////////////////////////////////////     
    stream_arbiter #(
        .DATA_T (bingo_hw_manager_task_desc_full_t                             ),
        .N_INP  (NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET              )
    ) i_stream_arbiter_chiplet_dep_set (
        .clk_i      ( clk_i                                        ),
        .rst_ni     ( rst_ni                                       ),
        .inp_data_i ( stream_arbiter_chiplet_dep_set_inp_task_desc ),
        .inp_valid_i( stream_arbiter_chiplet_dep_set_inp_valid     ),
        .inp_ready_o( stream_arbiter_chiplet_dep_set_inp_ready     ),
        .oup_data_o ( stream_arbiter_chiplet_dep_set_oup_task_desc ),
        .oup_valid_o( stream_arbiter_chiplet_dep_set_oup_valid     ),
        .oup_ready_i( stream_arbiter_chiplet_dep_set_oup_ready     )
    );
    assign stream_arbiter_chiplet_dep_set_oup_ready = chiplet_dep_set_task_desc_ready;
    always_comb begin : compose_stream_arbiter_chiplet_dep_set_signals
        for (int unsigned cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin
            for (int unsigned core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin
                // The whole head descriptor, without the class in the reserved
                // bits: the chiplet dep-set master sends it as its write data, and
                // the other chiplet reads only the dep-set fields and the core id
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER] = desc_to_full(checkout_queue_data_out[core][cluster]);
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].reserved_bits = '0;
                stream_arbiter_chiplet_dep_set_inp_valid[core + cluster * NUM_CORES_PER_CLUSTER] = checkout_chiplet_set_valid[core][cluster];
            end           
        end
    end


    //////////////////////////////////////////////////////////////////////
    // Chiplet from remote Done Queue
    //////////////////////////////////////////////////////////////////////
    // Other chiplets write this port: only the mailbox write register is
    // writable, so a remote write cannot flush the FIFO (CTRL) or change its
    // thresholds and lose dependency signals (SLVERR, no effect)
    bingo_hw_manager_write_mailbox #(
        .MailboxDepth(ChipletDoneQueueDepth                    ),
        .IrqEdgeTrig (1'b0                                     ),
        .IrqActHigh  (1'b1                                     ),
        .AxiAddrWidth(HostAxiLiteAddrWidth                     ),
        .AxiDataWidth(HostAxiLiteDataWidth                     ),
        .ChipIdWidth (ChipIdWidth                              ),
        .RegWriteEn  (1'b0                                     ),
        .req_lite_t  (host_axi_lite_req_t                      ),
        .resp_lite_t (host_axi_lite_resp_t                     )
    ) i_bingo_hw_manager_chiplet_done_queue (
        .clk_i       (clk_i                             ),
        .rst_ni      (rst_ni                            ),
        .chip_id_i   (chip_id_i                         ),
        .test_i      (1'b0                              ),
        .req_i       (from_remote_axi_lite_req_i        ),
        .resp_o      (from_remote_axi_lite_resp_o       ),
        .irq_o       (/*not used*/                      ),
        .base_addr_i (chiplet_mailbox_base_addr_i       ),
        .mbox_data_o (chiplet_done_queue_mbox_data      ),
        .mbox_pop_i  (chiplet_done_queue_mbox_pop       ),
        .mbox_empty_o(chiplet_done_queue_mbox_empty     ),
        .mbox_flush_i('0                                )
    );
    assign cur_chiplet_done_queue_task_desc = bingo_hw_manager_task_desc_full_t'(chiplet_done_queue_mbox_data);
    assign chiplet_done_queue_mbox_pop =  stream_arbiter_dep_matrix_set_inp_ready[NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET] && !chiplet_done_queue_mbox_empty;
    //////////////////////////////////////////////////////////////////////
    // Stream demux core type
    //////////////////////////////////////////////////////////////////////
    stream_demux #(
        .N_OUP ( NUM_CORES_PER_CLUSTER           )
    ) i_stream_demux_core_type (
        .inp_valid_i ( stream_demux_core_type_inp_valid ),
        .inp_ready_o ( stream_demux_core_type_inp_ready ),
        .oup_sel_i   ( stream_demux_core_type_oup_sel   ),
        .oup_valid_o ( stream_demux_core_type_oup_valid ),
        .oup_ready_i ( stream_demux_core_type_oup_ready )
    );
    always_comb begin: compose_stream_demux_core_type_signals
        stream_demux_core_type_inp_valid = muxed_task_valid;
        stream_demux_core_type_oup_sel = cur_task_desc.assigned_core_id;
        for (int unsigned core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin
            stream_demux_core_type_oup_ready[core] = !waiting_dep_check_queue_full[core];
        end
    end


    for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin: gen_waiting_dep_check_queue
        fifo_v3 #(
            .FALL_THROUGH ( 1'b0                               ),
            .DEPTH        ( WaitingQueueDepth                  ),
            .dtype        ( bingo_hw_manager_task_desc_t       )
        ) i_waiting_dep_check_queue (
            .clk_i       ( clk_i                               ),
            .rst_ni      ( rst_ni                              ),
            .testmode_i  ( 1'b0                                ),
            .flush_i     ( 1'b0                                ),
            .full_o      ( waiting_dep_check_queue_full[core]  ),
            .empty_o     ( waiting_dep_check_queue_empty[core] ),
            .usage_o     ( /*not used*/                        ),
            .data_i      ( cur_task_desc                       ),
            .push_i      ( waiting_dep_check_queue_push[core]  ),
            .data_o      ( waiting_dep_check_task_desc[core]   ),
            .pop_i       ( waiting_dep_check_queue_pop[core]   )
        );
        assign waiting_dep_check_queue_push[core] = stream_demux_core_type_oup_valid[core] && !waiting_dep_check_queue_full[core];
        assign waiting_dep_check_queue_pop[core] = dep_check_manager_inp_wait_dep_check_queue_ready[core] && !waiting_dep_check_queue_empty[core];

        bingo_hw_manager_dep_check_manager i_dep_check_manager(
            .clk_i                       ( clk_i                        ),
            .rst_ni                      ( rst_ni                       ),
            .wait_dep_check_queue_valid_i(dep_check_manager_inp_wait_dep_check_queue_valid[core]),
            .wait_dep_check_queue_ready_o(dep_check_manager_inp_wait_dep_check_queue_ready[core]),
            .dep_check_valid_o           (dep_check_manager_oup_dep_check_valid[core]),
            .dep_check_ready_i           (dep_check_manager_oup_dep_check_ready[core]),
            .ready_and_checkout_queue_valid_o(dep_check_manager_oup_ready_and_checkout_queue_valid[core]),
            .ready_and_checkout_queue_ready_i(dep_check_manager_oup_ready_and_checkout_queue_ready[core])
        );
        assign dep_check_manager_inp_wait_dep_check_queue_valid[core] = ~waiting_dep_check_queue_empty[core];
        // To Dep Matrix
        // For the dep matrix, if the dep check is disable, we do not need to send the task to dep matrix
        stream_filter i_stream_filter_dep_check_en_to_dep_matrix (
            .valid_i ( dep_check_manager_oup_dep_check_valid[core]    ),
            .ready_o ( dep_check_manager_oup_dep_check_ready[core]    ),
            .drop_i  ( (!waiting_dep_check_task_desc[core].dep_check_info.dep_check_en) ),
            .valid_o ( demux_dep_matrix_inp_valid[core]  ),
            .ready_i ( demux_dep_matrix_inp_ready[core]  )
        );
        stream_demux #(
            .N_OUP ( NUM_CLUSTERS_PER_CHIPLET           )
        ) i_stream_demux_from_waiting_dep_check_queue_to_dep_matrix (
            .inp_valid_i ( demux_dep_matrix_inp_valid[core]    ),
            .inp_ready_o ( demux_dep_matrix_inp_ready[core]    ),
            .oup_sel_i   ( waiting_dep_check_task_desc[core].assigned_cluster_id ),
            .oup_valid_o ( demux_dep_matrix_oup_valid[core]    ),
            .oup_ready_i ( demux_dep_matrix_oup_ready[core]    )
        );
        // To Ready Queue and Checkout Queue
        // We need a filter to drop the dummy check tasks
        // The dummy check does not need to go to the ready and checkout queue
        stream_filter i_stream_filter_dummy_check_task_to_ready_and_checkout_queue (
            .valid_i ( dep_check_manager_oup_ready_and_checkout_queue_valid[core]    ),
            .ready_o ( dep_check_manager_oup_ready_and_checkout_queue_ready[core]    ),
            .drop_i  ( (waiting_dep_check_task_desc[core].task_type == TaskDummy) && (waiting_dep_check_task_desc[core].dep_check_info.dep_check_en) ), // Drop if it is a dummy check task
            .valid_o ( demux_ready_and_checkout_queue_inp_valid[core]  ),
            .ready_i ( demux_ready_and_checkout_queue_inp_ready[core]  )
        );
        stream_demux #(
            .N_OUP ( NUM_CLUSTERS_PER_CHIPLET           )
        ) i_stream_demux_from_waiting_dep_check_queue_to_ready_and_checkout_queue (
            .inp_valid_i ( demux_ready_and_checkout_queue_inp_valid[core]    ),
            .inp_ready_o ( demux_ready_and_checkout_queue_inp_ready[core]    ),
            // Physical cluster: the logical one unless the task goes to a substitute
            .oup_sel_i   ( remap_select.cluster[core]                          ),
            .oup_valid_o ( demux_ready_and_checkout_queue_oup_valid[core]    ),
            .oup_ready_i ( demux_ready_and_checkout_queue_oup_ready[core]    )
        );

    end


    ////////////////////////////////////////////////////////////////////////
    // Dep Matrix
    //////////////////////////////////////////////////////////////////////

    for (genvar cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin: gen_dep_matrix
        bingo_hw_manager_dep_matrix #(
            .DEP_MATRIX_ROWS(NUM_CORES_PER_CLUSTER),
            .DEP_MATRIX_COLS(NUM_CORES_PER_CLUSTER),
            .TagWidth(DepTagWidth)
        ) i_dep_matrix (
            .clk_i             (clk_i                    ),
            .rst_ni            (rst_ni                   ),
            .dep_check_valid_i (dep_check_valid[cluster] ),
            .dep_check_code_i  (dep_check_code[cluster]  ),
            .dep_check_tag_i   (dep_check_tag[cluster]   ),
            .dep_check_result_o(dep_check_result[cluster]),
            .dep_set_valid_i   (dep_set_valid[cluster]   ),
            .dep_set_ready_o   (dep_set_ready[cluster]   ),
            .dep_set_code_i    (dep_set_code[cluster]    ),
            .dep_set_tag_i     (dep_set_tag[cluster]     )
        );
    end

    always_comb begin : connect_dep_check_for_dep_matrix
        for ( int cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin
            for ( int core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin
                dep_check_valid[cluster][core] = demux_dep_matrix_oup_valid[core][cluster];
                demux_dep_matrix_oup_ready[core][cluster] = dep_check_result[cluster][core];
                dep_check_code[cluster][core] = waiting_dep_check_task_desc[core].dep_check_info.dep_check_code;
                dep_check_tag[cluster][core] = waiting_dep_check_task_desc[core].dep_check_info.dep_check_tag;
            end
        end
    end

    //////////////////////////////////////////////////////////////////////
    // Stream Arbiter Dep Matrix Set
    //////////////////////////////////////////////////////////////////////
    stream_arbiter #(
        .DATA_T(bingo_hw_manager_dep_matrix_set_meta_t),
        .N_INP (STREAM_ARBITER_DEP_MATRIX_SET_NUM_INP)
    ) i_stream_arbiter_dep_matrix_set(
        .clk_i      (clk_i),
        .rst_ni     (rst_ni),
        .inp_data_i (stream_arbiter_dep_matrix_set_inp_data ),
        .inp_valid_i(stream_arbiter_dep_matrix_set_inp_valid),
        .inp_ready_o(stream_arbiter_dep_matrix_set_inp_ready),
        .oup_data_o (stream_arbiter_dep_matrix_set_oup_data ),
        .oup_valid_o(stream_arbiter_dep_matrix_set_oup_valid),
        .oup_ready_i(stream_arbiter_dep_matrix_set_oup_ready)
    );
    always_comb begin : compose_stream_arbiter_dep_matrix_set_inputs
        // For Checkout Queue
        int stream_arbiter_inp_idx;
        for ( int core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin
            for ( int cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin
                    stream_arbiter_inp_idx = core + cluster * NUM_CORES_PER_CLUSTER;
                    stream_arbiter_dep_matrix_set_inp_data[stream_arbiter_inp_idx].dep_matrix_id = checkout_queue_data_out[core][cluster].dep_set_info.dep_set_cluster_id;
                    stream_arbiter_dep_matrix_set_inp_data[stream_arbiter_inp_idx].dep_matrix_col = checkout_queue_data_out[core][cluster].assigned_core_id;
                    stream_arbiter_dep_matrix_set_inp_data[stream_arbiter_inp_idx].dep_matrix_set_tag = checkout_queue_data_out[core][cluster].dep_set_info.dep_set_tag;
                    stream_arbiter_dep_matrix_set_inp_data[stream_arbiter_inp_idx].dep_set_code  = checkout_queue_data_out[core][cluster].dep_set_info.dep_set_code;
                    // Valid only for a head that may retire, so a task that ran
                    // already has its done here (bingo_hw_manager_checkout_queue)
                    stream_arbiter_dep_matrix_set_inp_valid[stream_arbiter_inp_idx] = checkout_local_set_valid[core][cluster];
            end
        end
        // For Chiplet Set Queue
        stream_arbiter_dep_matrix_set_inp_data[NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET].dep_matrix_id  = cur_chiplet_done_queue_task_desc.dep_set_info.dep_set_cluster_id;
        stream_arbiter_dep_matrix_set_inp_data[NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET].dep_matrix_col = cur_chiplet_done_queue_task_desc.assigned_core_id;
        stream_arbiter_dep_matrix_set_inp_data[NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET].dep_matrix_set_tag = cur_chiplet_done_queue_task_desc.dep_set_info.dep_set_tag;
        stream_arbiter_dep_matrix_set_inp_data[NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET].dep_set_code   = cur_chiplet_done_queue_task_desc.dep_set_info.dep_set_code;
        stream_arbiter_dep_matrix_set_inp_valid[NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET] = !chiplet_done_queue_mbox_empty;
        stream_arbiter_dep_matrix_set_oup_ready = stream_demux_set_dep_matrix_cluster_id_inp_ready;
    end 
    //////////////////////////////////////////////////////////////////////
    // Stream Demux Set Dep Matrix Cluster ID
    //////////////////////////////////////////////////////////////////////
    stream_demux #(
        .N_OUP(NUM_CLUSTERS_PER_CHIPLET)
    ) i_stream_demux_set_dep_matrix_cluster_id (
        .inp_valid_i(stream_demux_set_dep_matrix_cluster_id_inp_valid),
        .inp_ready_o(stream_demux_set_dep_matrix_cluster_id_inp_ready),
        .oup_sel_i  (stream_demux_set_dep_matrix_cluster_id_oup_sel),
        .oup_valid_o(stream_demux_set_dep_matrix_cluster_id_oup_valid),
        .oup_ready_i(stream_demux_set_dep_matrix_cluster_id_oup_ready)
    );
    assign stream_demux_set_dep_matrix_cluster_id_inp_valid = stream_arbiter_dep_matrix_set_oup_valid;
    assign stream_demux_set_dep_matrix_cluster_id_oup_sel = stream_arbiter_dep_matrix_set_oup_data.dep_matrix_id;

    //////////////////////////////////////////////////////////////////////
    // Stream Demux Set Dep Matrix Core ID
    //////////////////////////////////////////////////////////////////////
    for (genvar cluster= 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin: gen_set_dep_matrix_core_id
        stream_demux #(
            .N_OUP(NUM_CORES_PER_CLUSTER)
        ) i_stream_demux_set_dep_matrix_core_id (
            .inp_valid_i(stream_demux_set_dep_matrix_core_id_inp_valid[cluster]),
            .inp_ready_o(stream_demux_set_dep_matrix_core_id_inp_ready[cluster]),
            .oup_sel_i  (stream_demux_set_dep_matrix_core_id_oup_sel[cluster]  ),
            .oup_valid_o(stream_demux_set_dep_matrix_core_id_oup_valid[cluster]),
            .oup_ready_i(stream_demux_set_dep_matrix_core_id_oup_ready[cluster])
        );
        assign stream_demux_set_dep_matrix_cluster_id_oup_ready[cluster] = stream_demux_set_dep_matrix_core_id_inp_ready[cluster];
        assign stream_demux_set_dep_matrix_core_id_inp_valid[cluster] = stream_demux_set_dep_matrix_cluster_id_oup_valid[cluster];
        assign stream_demux_set_dep_matrix_core_id_oup_sel[cluster] = stream_arbiter_dep_matrix_set_oup_data.dep_matrix_col;
    end

    always_comb begin : connect_dep_set_for_dep_matrix
        for ( int cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin
            for ( int core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin
                dep_set_valid[cluster][core] = stream_demux_set_dep_matrix_core_id_oup_valid[cluster][core];
                stream_demux_set_dep_matrix_core_id_oup_ready[cluster][core] = dep_set_ready[cluster][core];
                dep_set_code[cluster][core] = stream_arbiter_dep_matrix_set_oup_data.dep_set_code;
                dep_set_tag[cluster][core] = stream_arbiter_dep_matrix_set_oup_data.dep_matrix_set_tag;
            end
        end        
    end

    //////////////////////////////////////////////////////////////////////
    // Ready Queue
    //////////////////////////////////////////////////////////////////////
    // A task routed to a slot enters its checkout queue and, if it runs on a
    // core (route.ready_push, see bingo_hw_manager_router), its ready queue
    // (bingo_hw_manager_ready_queue), where the core reads its id.
    for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin: gen_ready_queue_per_core
        for (genvar cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin: gen_ready_queue_per_core_per_cluster
            task_push_t  route_in, replay_in, import_in;
            ready_push_t push;
            ready_head_t head;
            assign route_in  = '{valid: route.ready_push[core][cluster],
                                 desc:  waiting_dep_check_task_desc[route.src_core[core][cluster]]};
            assign replay_in = '{valid: replay_push_ready[core][cluster], desc: replay_data};
            assign import_in = '{valid: imported.push[core][cluster], desc: imported.desc};
            bingo_hw_manager_ready_queue #(
                .InterfaceType ( READY_AND_DONE_QUEUE_INTERFACE_TYPE     ),
                .Depth         ( ReadyQueueDepth                         ),
                .WdClassWidth  ( WdClassWidth                            ),
                .ChipIdWidth   ( ChipIdWidth                             ),
                .CoreIdWidth   ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
                .ClusterIdWidth( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
                .AxiAddrWidth  ( DeviceAxiLiteAddrWidth                  ),
                .AxiDataWidth  ( DeviceAxiLiteDataWidth                  ),
                .task_desc_t   ( bingo_hw_manager_task_desc_t            ),
                .ready_desc_t  ( bingo_hw_manager_ready_task_desc_full_t ),
                .req_lite_t    ( device_axi_lite_req_t                   ),
                .resp_lite_t   ( device_axi_lite_resp_t                  ),
                .addr_t        ( device_axi_lite_addr_t                  ),
                .task_push_t   ( task_push_t                             ),
                .ready_push_t  ( ready_push_t                            ),
                .ready_head_t  ( ready_head_t                            )
            ) i_ready (
                .clk_i           ( clk_i                                                           ),
                .rst_ni          ( rst_ni                                                          ),
                .chip_id_i       ( chip_id_i                                                       ),
                .core_id_i       ( bingo_hw_manager_assigned_core_id_t'(core)                      ),
                .cluster_id_i    ( bingo_hw_manager_assigned_cluster_id_t'(cluster)                ),
                .route_i         ( route_in                                                        ),
                .replay_i        ( replay_in                                                       ),
                .import_i        ( import_in                                                       ),
                .push_o          ( push                                                            ),
                .full_o          ( ready_queue_full[core][cluster]                                 ),
                .flush_i         ( replay_ready_flush[core][cluster]                               ),
                .pop_i           ( ready_queue_pop[core][cluster]                                  ),
                .head_o          ( head                                                            ),
                .base_addr_i     ( ready_queue_base_addr_i +
                                   (core + cluster * NUM_CORES_PER_CLUSTER) * ReadyQueueAddrOffset ),
                .axi_lite_req_i  ( ready_queue_axi_lite_req_i[core][cluster]                       ),
                .axi_lite_resp_o ( ready_queue_axi_lite_resp_o[core][cluster]                      )
            );
            assign ready_queue_push[core][cluster]     = push.valid;
            assign ready_queue_data_in[core][cluster]  = push.data;
            assign ready_queue_empty[core][cluster]    = head.empty;
            assign ready_queue_data_out[core][cluster] = head.data;
            assign ready_class[core][cluster]          = head.wd_class;
            assign ready_moved[core][cluster]          = head.moved;
        end
    end


    //////////////////////////////////////////////////////////////////////
    // Checkout Queue
    //////////////////////////////////////////////////////////////////////
    // Every task placed on a physical slot waits in its checkout queue until it
    // retires (see bingo_hw_manager_checkout_queue): with its own done if it ran
    // on a core, at once if it runs on no core, never from a stuck slot unless
    // it runs on no core. The retiring head sets its dependencies here or on
    // another chiplet; an imported head goes back to its origin instead.
    for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin: gen_checkout_queue_per_core
        for (genvar cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin: gen_checkout_queue_per_core_per_cluster
            localparam int unsigned Slot = core + cluster * NUM_CORES_PER_CLUSTER;
            checkout_route_t  route_in;
            checkout_replay_t replay_in;
            checkout_import_t import_in;
            checkout_entry_t  push;
            checkout_occ_t    occ;
            checkout_head_t   head;
            retire_req_t      retire;
            retire_rsp_t      retire_rsp;
            assign route_in  = '{valid:       route.fire[core][cluster],
                                 desc:        waiting_dep_check_task_desc[route.src_core[core][cluster]],
                                 skip:        cond_exec_skip[route.src_core[core][cluster]],
                                 exit_absorb: route.exit_absorb[core][cluster],
                                 remote:      route.remote[core][cluster]};
            assign replay_in = '{push:        replay_push[core][cluster],
                                 desc:        replay_data,
                                 tag:         checkout_remote_tag_out[replay_src_core][replay_src_cluster],
                                 rotate:      replay_rotate,
                                 exit_absorb: replay_exit_absorb,
                                 pop:         replay_pop[core][cluster],
                                 hold:        replay_hold_slot[core][cluster]};
            assign import_in = '{valid: imported.push[core][cluster], desc: imported.desc, tag: imported.tag};
            assign retire_rsp = '{local_set:   stream_arbiter_dep_matrix_set_inp_ready[Slot],
                                  chiplet_set: stream_arbiter_chiplet_dep_set_inp_ready[Slot],
                                  remote_done: remote_done_arb_ready[Slot]};
            bingo_hw_manager_checkout_queue #(
                .NumCores          ( NUM_CORES_PER_CLUSTER                            ),
                .NumClusters       ( NUM_CLUSTERS_PER_CHIPLET                         ),
                .Core              ( core                                             ),
                .Cluster           ( cluster                                          ),
                .Depth             ( CheckoutQueueDepth                               ),
                .TaskIdWidth       ( TaskIdWidth                                      ),
                .CoreIdWidth       ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
                .ClusterIdWidth    ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
                .RemoteEn          ( RemoteEn                                         ),
                .ForeignStuckDrain ( ForeignStuckDrain                                ),
                .OutstandingWidth  ( RemapOutstandingWidth                            ),
                .task_desc_t       ( bingo_hw_manager_task_desc_t                     ),
                .remote_tag_t      ( remote_tag_t                                     ),
                .chip_id_t         ( chip_id_t                                        ),
                .checkout_route_t  ( checkout_route_t                                 ),
                .checkout_replay_t ( checkout_replay_t                                ),
                .checkout_import_t ( checkout_import_t                                ),
                .checkout_entry_t  ( checkout_entry_t                                 ),
                .checkout_occ_t    ( checkout_occ_t                                   ),
                .checkout_head_t   ( checkout_head_t                                  ),
                .slot_state_t      ( slot_state_t                                     ),
                .done_t            ( done_t                                           )
            ) i_checkout (
                .clk_i             ( clk_i                                          ),
                .rst_ni            ( rst_ni                                         ),
                .chip_id_i         ( chip_id_i                                      ),
                .replay_i          ( replay_in                                      ),
                .import_i          ( import_in                                      ),
                .route_i           ( route_in                                       ),
                .push_o            ( push                                           ),
                .pop_o             ( checkout_queue_pop[core][cluster]              ),
                .occ_o             ( occ                                            ),
                .head_o            ( head                                           ),
                .slot_i            ( slot_state                                     ),
                .cerf_state_i      ( cerf_state                                     ),
                .outstanding_i     ( remap_outstanding_q                            ),
                .moved_in_i        ( moved_in_q[core][cluster]                      ),
                .stuck_drain_o     ( stuck_drain[core][cluster]                     ),
                .stuck_head_skip_o ( stuck_head_skip[core][cluster]                 ),
                .done_i            ( done_queue                                     ),
                .done_pop_o        ( done_q_pop[core][cluster]                      ),
                .remote_mismatch_o ( remote_head_mismatch[core][cluster]            ),
                .retire_o          ( retire                                         ),
                .retire_i          ( retire_rsp                                     )
            );
            assign checkout_queue_push[core][cluster]         = push.valid;
            assign checkout_queue_data_in[core][cluster]      = push.desc;
            assign checkout_remote_tag_in[core][cluster]      = push.tag;
            assign exit_absorb[core][cluster]                 = push.exit_absorb;
            assign checkout_queue_full[core][cluster]         = occ.full;
            assign checkout_queue_full_raw[core][cluster]     = occ.full_raw;
            assign checkout_queue_empty[core][cluster]        = occ.empty;
            assign checkout_queue_usage[core][cluster]        = occ.usage;
            assign checkout_queue_data_out[core][cluster]     = head.desc;
            assign checkout_remote_tag_out[core][cluster]     = head.tag;
            assign checkout_head_imported[core][cluster]      = head.imported;
            assign checkout_head_exec[core][cluster]          = head.exec;
            assign checkout_retire_valid[core][cluster]       = retire.valid;
            assign checkout_local_set_valid[core][cluster]    = retire.local_set;
            assign checkout_chiplet_set_valid[core][cluster]  = retire.chiplet_set;
            assign remote_done_arb_valid[Slot]                = retire.remote_done;
            assign remote_done_arb_data[Slot] = '{
                chip:       checkout_remote_tag_out[core][cluster].origin_chip,
                proxy_slot: checkout_remote_tag_out[core][cluster].proxy_slot,
                task_id:    checkout_queue_data_out[core][cluster].task_id,
                reject:     1'b0
            };
        end
    end

    //////////////////////////////////////////////////////////////////////
    // Core interface and done queues
    //////////////////////////////////////////////////////////////////////
    // With the CSR interface the cores read their ready queues, write their
    // dones and send heartbeats through CSR requests (bingo_hw_manager_csr_port),
    // which cut a fenced core off. With the AXI-Lite interface they use the
    // mailboxes of the ready queues and of the done queues instead.
    // Each (core, cluster) slot has its own done queue
    // (bingo_hw_manager_done_queues), so the completions of different cores
    // and clusters drain independently; a done of an exported task comes back
    // into the done queue of its proxy slot.
    csr_done_t                        csr_done;
    logic                             csr_done_ready;
    if (READY_AND_DONE_QUEUE_INTERFACE_TYPE==1) begin: gen_csr_to_fifo_intf
        core_status_t core_status;
        bingo_hw_manager_csr_port #(
            .NumCores         ( NUM_CORES_PER_CLUSTER             ),
            .NumClusters      ( NUM_CLUSTERS_PER_CHIPLET          ),
            .TaskIdWidth      ( TaskIdWidth                       ),
            .CsrHeartbeatAddr ( CsrHeartbeatAddr                  ),
            .csr_req_t        ( csr_req_t                         ),
            .csr_rsp_t        ( csr_rsp_t                         ),
            .data_t           ( device_axi_lite_data_t            ),
            .done_info_t      ( bingo_hw_manager_done_info_full_t ),
            .chip_id_t        ( chip_id_t                         ),
            .slot_state_t     ( slot_state_t                      ),
            .ready_t          ( ready_t                           ),
            .core_status_t    ( core_status_t                     ),
            .csr_done_t       ( csr_done_t                        )
        ) i_csr_port (
            .clk_i           ( clk_i           ),
            .rst_ni          ( rst_ni          ),
            .chip_id_i       ( chip_id_i       ),
            .csr_req_i       ( csr_req_i       ),
            .csr_req_valid_i ( csr_req_valid_i ),
            .csr_req_ready_o ( csr_req_ready_o ),
            .csr_rsp_o       ( csr_rsp_o       ),
            .csr_rsp_valid_o ( csr_rsp_valid_o ),
            .csr_rsp_ready_i ( csr_rsp_ready_i ),
            .slot_i          ( slot_state      ),
            .ready_i         ( ready_queue     ),
            .ready_pop_o     ( ready_queue_pop ),
            .core_o          ( core_status     ),
            .done_o          ( csr_done        ),
            .done_ready_i    ( csr_done_ready  )
        );
        assign heartbeat_valid          = core_status.heartbeat;
        assign core_status_waiting_task = core_status.waiting;
        assign exited_q                 = core_status.exited;
    end else begin: gen_no_csr_to_fifo_intf
        // A core waits for a task while its read of the ready mailbox is
        // pending (ar_valid high, r_ready low); the mailbox pops itself
        for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core++) begin : gen_waiting_core
            for (genvar cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster++) begin : gen_waiting_cluster
                assign core_status_waiting_task[core][cluster] = ready_queue_axi_lite_req_i[core][cluster].ar_valid &&
                                                                 !ready_queue_axi_lite_req_i[core][cluster].r_ready;
            end
        end
        assign ready_queue_pop = '0;
        // Tie the csr signals to zero
        assign csr_req_ready_o = '0;
        assign csr_rsp_o       = '0;
        assign csr_rsp_valid_o = '0;
        assign heartbeat_valid = '0;
        assign exited_q        = '0;
        assign csr_done        = '0;
    end

    bingo_hw_manager_done_queues #(
        .NumCores      ( NUM_CORES_PER_CLUSTER                            ),
        .NumClusters   ( NUM_CLUSTERS_PER_CHIPLET                         ),
        .InterfaceType ( READY_AND_DONE_QUEUE_INTERFACE_TYPE              ),
        .Depth         ( DoneQueueDepth                                   ),
        .RemoteEn      ( RemoteEn                                         ),
        .TaskIdWidth   ( TaskIdWidth                                      ),
        .CoreIdWidth   ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
        .ClusterIdWidth( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
        .ChipIdWidth   ( ChipIdWidth                                      ),
        .AxiAddrWidth  ( DeviceAxiLiteAddrWidth                           ),
        .AxiDataWidth  ( DeviceAxiLiteDataWidth                           ),
        .done_info_t   ( bingo_hw_manager_done_info_full_t                ),
        .remote_done_t ( remote_done_t                                    ),
        .req_lite_t    ( device_axi_lite_req_t                            ),
        .resp_lite_t   ( device_axi_lite_resp_t                           ),
        .slot_state_t  ( slot_state_t                                     ),
        .csr_done_t    ( csr_done_t                                       ),
        .done_remote_t ( done_remote_t                                    ),
        .done_t        ( done_t                                           )
    ) i_done_queues (
        .clk_i               ( clk_i                      ),
        .rst_ni              ( rst_ni                     ),
        .chip_id_i           ( chip_id_i                  ),
        .csr_done_i          ( csr_done                   ),
        .csr_done_ready_o    ( csr_done_ready             ),
        .base_addr_i         ( done_queue_base_addr_i     ),
        .axi_lite_req_i      ( done_queue_axi_lite_req_i  ),
        .axi_lite_resp_o     ( done_queue_axi_lite_resp_o ),
        .slot_i              ( slot_state                 ),
        .remote_done_valid_i ( remote_done_valid_i        ),
        .remote_done_ready_o ( remote_done_ready_o        ),
        .remote_done_i       ( remote_done_in             ),
        .remote_o            ( done_remote                ),
        .done_o              ( done_queue                 ),
        .pop_i               ( done_q_pop                 )
    );

    //////////////////////////////////////////////////////////////////////
    // Power Manager
    //////////////////////////////////////////////////////////////////////
    bingo_hw_manager_pm #(
        .NUM_CLUSTERS_PER_CHIPLET ( NUM_CLUSTERS_PER_CHIPLET          ),
        .NUM_CORES_PER_CLUSTER    ( NUM_CORES_PER_CLUSTER             ),
        .CfgBusWidth              ( DeviceAxiLiteDataWidth            ),
        .HOST_DVFS_MSIP_BIT       ( HOST_DVFS_MSIP_BIT                ),
        .req_lite_t               ( host_axi_lite_req_t               ),
        .resp_lite_t              ( host_axi_lite_resp_t              ),
        .addr_t                   ( host_axi_lite_addr_t              ),
        .data_t                   ( host_axi_lite_data_t              )
    ) i_bingo_hw_manager_pm (
        .clk_i                 ( clk_i                                 ),
        .rst_ni                ( rst_ni                                ),
        // Configuration from the host
        .enable_idle_pm_i      ( bingo_hw_manager_enable_idle_pm_i      ),
        .idle_power_level_i    ( bingo_hw_manager_idle_power_level_i    ),
        .normal_power_level_i  ( bingo_hw_manager_normal_power_level_i  ),
        .boost_power_level_i   ( bingo_hw_manager_boost_power_level_i   ),
        .core_boost_i          ( ctrl_view.boost                        ),
        .derate_power_level_i  ( device_axi_lite_data_t'(bingo_hw_manager_risk_policy_i[15:8]) ),
        .core_derate_i         ( ctrl_view.derate                       ),
        .pm_base_addr_i        ( bingo_hw_manager_pm_base_addr_i        ),
        .slot_domain_i         ( ctrl_slot_domain                       ),
        // Internal Core status: polling or fenced (bingo_hw_manager_power_view), so a
        // dead core does not keep its domain at the normal level
        .core_status_waiting_task_i ( ctrl_view.idle                   ),
        .access_power_level_i  ( device_axi_lite_data_t'(pm_access_level) ),
        .core_access_only_i    ( ctrl_view.access_only                  ),
        // DVFS mode: monitor + notify host
        .pm_mode_i             ( bingo_hw_manager_pm_mode_i             ),
        .dvfs_clint_msip_addr_i( bingo_hw_manager_dvfs_clint_msip_addr_i),
        .dvfs_ack_i            ( bingo_hw_manager_dvfs_ack_i            ),
        .dvfs_request_o        ( bingo_hw_manager_dvfs_request_o        ),
        .domain_level_o        ( pm_domain_level                        ),
        // Interface to Host AXI Lite
        .pm_axi_lite_req_o     (pm_axi_lite_req_o                      ),
        .pm_axi_lite_resp_i    (pm_axi_lite_resp_i                     ),
        .bus_error_o           (pm_bus_error                           )
    );
    assign bus_error_o = {pm_bus_error, fetch_bus_error};
    // Power-domain counters (observation only): cycles of each counted domain
    // per clock level the PM applied, and cycles with an active slot (one that
    // is neither waiting for a task, nor exited, nor fenced: the PM's idle
    // view before the idle entry delay and the holds of bingo_hw_manager_ctrl)
    if (PmCountDomains != 0) begin : gen_pm_counters
        bingo_hw_manager_pm_counters #(
            .NumCores    ( NUM_CORES_PER_CLUSTER    ),
            .NumClusters ( NUM_CLUSTERS_PER_CHIPLET ),
            .FirstDomain ( PmCountBase              ),
            .NumDomains  ( PmCountDomains           )
        ) i_pm_counters (
            .clk_i          ( clk_i                                                 ),
            .rst_ni         ( rst_ni                                                ),
            .pm_i           ( pm_view                                               ),
            .slot_domain_i  ( ctrl_slot_domain                                      ),
            .slot_active_i  ( ~(core_status_waiting_task | exited_q | core_fenced) ),
            .count_o        ( pm_count_o                                            )
        );
    end else begin : gen_no_pm_counters
        assign pm_count_o = '0;
    end

    //////////////////////////////////////////////////////////////////////
    // Watchdog for core heartbeat monitoring
    //////////////////////////////////////////////////////////////////////
    // core_available is kept for observability only; routing no longer uses it
    // (a healthy core keeps its tasks whether it is busy or polling).
    bingo_hw_manager_watchdog #(
        .NumCores(NUM_CORES_PER_CLUSTER),
        .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
        .CounterWidth(WatchdogCounterWidth),
        .HeartbeatTimeoutCycles(WatchdogHeartbeatTimeoutCycles),
        .ConfirmTimeoutCycles(WatchdogConfirmTimeoutCycles),
        .CoreMask(WatchdogCoreMask),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .WdClassWidth(WdClassWidth),
        .CoreTypeId(CoreTypeId)
    ) i_watchdog (
        .clk_i                 ( clk_i                          ),
        .rst_ni                ( rst_ni                         ),
        .task_dispatched_i     ( ready_queue_pop                 ), // ready pop: busy, timer cleared
        .task_done_i           ( done_q_push                     ), // done reached the done FIFO: idle
        .heartbeat_i           ( heartbeat_valid                 ), // clears the timer (ignored once fenced)
        .waiting_task_i        ( core_status_waiting_task        ), // only feeds core_available
        .tick_i                ( ctrl_view.wd_tick               ), // slowed down in a slow power domain
        .late_cycles_i         ( wd_late_cycles                  ), // fault precursor threshold
        .late_thr_i            ( wd_late_slot                    ),
        .risk_i                ( ctrl_view.risk                  ), // registered ctrl risk bitmap
        .risk_confirm_i        ( WatchdogCounterWidth'(bingo_hw_manager_risk_confirm_i) ),
        .risk_confirm_valid_i  ( wd_risk_use                     ),
        .suspect_thr_i         ( wd_suspect_thr                  ),
        .confirm_thr_i         ( wd_confirm_thr                  ),
        .suspect_thr_valid_i   ( wd_h_ad_valid                   ),
        .adapt_confirm_i       ( wd_adapt_confirm                ),
        .wd_class_i            ( wd_class_q                      ),
        .progress_o            ( wd_progress                     ),
        .interval_o            ( wd_interval                     ),
        .late_o                ( wd_late                         ),
        .core_busy_o           ( core_busy                       ),
        .core_available_o      ( core_available                  ),
        .core_dead_suspect_o   ( core_dead_suspect               ),
        .core_fenced_o         ( core_fenced                     )
    );
    assign core_fenced_o = core_fenced;
    // A threshold beyond the timer saturates it (the timer stops at all ones)
    assign wd_late_cycles = (bingo_hw_manager_risk_late_i > device_axi_lite_data_t'({WatchdogCounterWidth{1'b1}})) ?
                            {WatchdogCounterWidth{1'b1}} : bingo_hw_manager_risk_late_i[WatchdogCounterWidth-1:0];
    assign core_dead_suspect_o = core_dead_suspect;
    assign ready_queue_pending_o = (READY_AND_DONE_QUEUE_INTERFACE_TYPE == 0) ? '0 : ~ready_queue_empty;
    // Validate the full CSR before narrowing it. A valid R is below C and fits.
    assign wd_risk_confirm = (WatchdogConfirmTimeoutCycles != 0 &&
                             bingo_hw_manager_risk_confirm_i > WatchdogHeartbeatTimeoutCycles &&
                             bingo_hw_manager_risk_confirm_i < WatchdogConfirmTimeoutCycles) ?
                             WatchdogCounterWidth'(bingo_hw_manager_risk_confirm_i) : '0;
    for (genvar c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin : gen_wd_type_core
        for (genvar cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin : gen_wd_type_cluster
            localparam int Type = CoreTypeId[c][cl];
            logic [31:0] h_eff, c_eff;
            logic c_valid;
            assign h_eff = (wd_type_h_i[Type] != 0 &&
                            wd_type_h_i[Type] <= WatchdogHeartbeatTimeoutCycles) ?
                            wd_type_h_i[Type] : WatchdogHeartbeatTimeoutCycles;
            assign c_valid = (WatchdogConfirmTimeoutCycles != 0 &&
                              wd_type_c_i[Type] != 0 &&
                              wd_type_c_i[Type] <= WatchdogConfirmTimeoutCycles &&
                              wd_type_c_i[Type] > h_eff);
            assign c_eff = c_valid ? wd_type_c_i[Type] : WatchdogConfirmTimeoutCycles;
            assign wd_h_eff[c][cl] = h_eff;
            assign wd_c_eff[c][cl] = c_eff;
            assign wd_late_ad_use[c][cl] = wd_late_ad_valid[c][cl] &&
                (wd_late_cycles == '0 || wd_late_ad[c][cl] <= 32'(wd_late_cycles));
            assign wd_late_slot[c][cl] = wd_late_ad_use[c][cl] ?
                WatchdogCounterWidth'(wd_late_ad[c][cl]) : wd_late_cycles;
            assign wd_suspect_thr[c][cl] = WatchdogCounterWidth'(
                wd_h_ad_valid[c][cl] ? wd_h_ad[c][cl] : h_eff);
            assign wd_risk_confirm_valid[c][cl] = (WatchdogConfirmTimeoutCycles != 0 &&
                                                  bingo_hw_manager_risk_confirm_i > h_eff &&
                                                  bingo_hw_manager_risk_confirm_i < c_eff);
            // R keeps its A5 validity rule; C_ad wins only if strictly shorter.
            assign wd_adapt_confirm[c][cl] = wd_c_ad_valid[c][cl] &&
                wd_c_ad[c][cl] < ((ctrl_view.risk[c][cl] && wd_risk_confirm_valid[c][cl]) ?
                                  bingo_hw_manager_risk_confirm_i : c_eff);
            assign wd_risk_use[c][cl] = wd_risk_confirm_valid[c][cl] &&
                                        !wd_adapt_confirm[c][cl];
            // Zero retains the parameter path and all existing logging.
            assign wd_confirm_thr[c][cl] = wd_adapt_confirm[c][cl] ?
                WatchdogCounterWidth'(wd_c_ad[c][cl]) :
                c_valid ? WatchdogCounterWidth'(c_eff) : '0;
        end
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) wd_class_q <= '0;
        else begin
            for (int c = 0; c < NUM_CORES_PER_CLUSTER; c++)
                for (int cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++)
                    if (ready_queue_pop[c][cl] && !done_q_push[c][cl] && !core_fenced[c][cl])
                        wd_class_q[c][cl] <= ready_class[c][cl];
        end
    end
    // Whether the slot's current task runs away from its own logical slot,
    // latched with its class
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) wd_moved_q <= '0;
        else begin
            for (int c = 0; c < NUM_CORES_PER_CLUSTER; c++)
                for (int cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++)
                    if (ready_queue_pop[c][cl] && !done_q_push[c][cl] && !core_fenced[c][cl])
                        wd_moved_q[c][cl] <= ready_moved[c][cl];
        end
    end
    bingo_hw_manager_wd_adapt #(
        .NumCores(NUM_CORES_PER_CLUSTER), .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
        .CounterWidth(WatchdogCounterWidth), .WdAdaptTypeBits(WdAdaptTypeBits),
        .WdClassWidth(WdClassWidth), .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreMask(WatchdogCoreMask), .CoreTypeId(CoreTypeId)
    ) i_wd_adapt (
        .clk_i(clk_i), .rst_ni(rst_ni), .ctrl_i(wd_adapt_i),
        .late_ctrl_i(bingo_hw_manager_wd_late_i),
        .progress_i(wd_progress), .interval_i(wd_interval), .dispatch_i(ready_queue_pop),
        .done_i(done_q_push), .fenced_i(core_fenced), .busy_i(core_busy),
        .class_i(wd_class_q), .moved_i(wd_moved_q), .fast_i(ctrl_view.wd_fast),
        .h_eff_i(wd_h_eff), .c_eff_i(wd_c_eff),
        .h_ad_o(wd_h_ad), .c_ad_o(wd_c_ad), .h_valid_o(wd_h_ad_valid),
        .c_valid_o(wd_c_ad_valid), .late_ad_o(wd_late_ad), .late_valid_o(wd_late_ad_valid)
    );

`ifndef SYNTHESIS
    bit wd_late_log;
    initial wd_late_log = $test$plusargs("bingo_adapt_log");
    always @(posedge clk_i) begin
        if (rst_ni && wd_late_log && bingo_hw_manager_wd_late_i[0]) begin
            for (int c = 0; c < NUM_CORES_PER_CLUSTER; c++)
                for (int cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++)
                    if (wd_late[c][cl])
                        $display("[BINGO_ADAPT_LATE] %0t slot=%0d core=%0d cluster=%0d type=%0d class=%0d interval=%0d threshold=%0d source=%s",
                                 $time, c + cl * NUM_CORES_PER_CLUSTER, c, cl,
                                 CoreTypeId[c][cl], wd_class_q[c][cl], wd_interval[c][cl],
                                 wd_late_slot[c][cl], wd_late_ad_use[c][cl] ? "adaptive" : "static");
        end
    end
`endif

    //////////////////////////////////////////////////////////////////////
    // Task Replay
    //////////////////////////////////////////////////////////////////////
    // A fenced core's checkout queue holds all its dispatched, unfinished tasks
    // in order (the head is the one it was running). The replay controller
    // (bingo_hw_manager_replay_ctrl) moves them to live cores, one MOVE step at
    // a time; its data path (bingo_hw_manager_replay_path) presents each
    // checkout head to it and carries out the steps on the queues.
    // Level 3: a fence-fallback type is not exported, as a type the transport
    // has no target for
    logic [2**CoreTypeIdWidth-1:0] remote_export_type_en;
    assign remote_export_type_en = remote_export_type_en_i & ~bingo_hw_manager_fence_fb_i;

    //////////////////////////////////////////////////////////////////////
    // Control plane: slot mapping table (substitute of every logical slot)
    //////////////////////////////////////////////////////////////////////
    logic                                                           smt_update;
    // Parking. The request mask is bit (core + cluster * NUM_CORES).
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_req;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] risk_clear;
    park_t                                                          park;
    // Logical slots with tasks in another core's checkout queue
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_moved;
    // HOLD or UNPARK: no new task of the slot may be placed
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_block;
    always_comb begin
        for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
            for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                park_moved[c][cl] = (remap_outstanding_q[c][cl] != '0);
            end
        end
    end
    assign park_block = park.hold | park.unpark;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_slot_push;
    // CERF degradation request (ctrl) and the cycle the register file commits it
    cerf_fb_t                    cerf_fb;
    logic                        cerf_fb_done;
    always_comb begin
        bingo_hw_manager_park_fail_o = '0;
        bingo_hw_manager_risk_o = '0;
        for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
            for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                int unsigned idx;
                idx = c + cl * NUM_CORES_PER_CLUSTER;
                // Parking needs the CSR ready queues (their empty flag ends the
                // drain); with AXI-Lite mailboxes a request fails at once
                park_req[c][cl] = (READY_AND_DONE_QUEUE_INTERFACE_TYPE == 1) &&
                                  (idx < $bits(bingo_hw_manager_park_req_i)) &&
                                  bingo_hw_manager_park_req_i[idx];
                park_slot_push[c][cl] = checkout_queue_push[c][cl] || ready_queue_push[c][cl];
                risk_clear[c][cl] = (idx < $bits(bingo_hw_manager_risk_clear_i)) &&
                                    bingo_hw_manager_risk_clear_i[idx];
                if (idx < $bits(bingo_hw_manager_risk_o)) bingo_hw_manager_risk_o[idx] = ctrl_view.risk[c][cl];
                if (idx < $bits(bingo_hw_manager_park_fail_o)) begin
                    bingo_hw_manager_park_fail_o[idx] = park.fail[c][cl] ||
                        ((READY_AND_DONE_QUEUE_INTERFACE_TYPE != 1) && bingo_hw_manager_park_req_i[idx]);
                end
            end
        end
    end
    // Load of a slot: its checkout queue occupancy (full = highest)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][CheckoutUsageWidth:0] ctrl_load;
    for (genvar c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin : gen_ctrl_load_core
        for (genvar cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin : gen_ctrl_load_cluster
            assign ctrl_load[c][cl] = {checkout_queue_full_raw[c][cl], checkout_queue_usage[c][cl]};
        end
    end
    // Inputs of the control plane: slot state, queues, host settings, power state
    ctrl_queue_t ctrl_queue;
    ctrl_host_t  ctrl_host;
    type_cfg_t   type_cfg;
    ctrl_cfg_t   ctrl_cfg;
    assign slot_state = '{fenced:   core_fenced,
                          retired:  core_retired,
                          stuck:    replay_stuck_slot,
                          rejected: remote_rejected_q};
    assign ctrl_queue = '{waiting:        core_status_waiting_task | exited_q,
                          checkout_empty: checkout_queue_empty,
                          ready_empty:    ready_queue_empty,
                          push:           park_slot_push,
                          moved:          park_moved,
                          load:           ctrl_load};
    assign ctrl_host  = '{park_req:   park_req,
                          risk_clear: risk_clear,
                          domain:     ctrl_slot_domain};
    assign type_cfg   = '{cerf_fb_en:    cerf_fb_en_i,
                          cerf_fb_clear: cerf_fb_clear_i,
                          cerf_fb_set:   cerf_fb_set_i,
                          fence_fb:      bingo_hw_manager_fence_fb_i};
    assign ctrl_cfg   = '{idle_delay:     32'(bingo_hw_manager_idle_entry_delay_i),
                          access_hold:    32'(bingo_hw_manager_access_wake_hold_i),
                          recovery_hold:  bingo_hw_manager_recovery_hold_i,
                          risk_thresh:    bingo_hw_manager_risk_policy_i[3:0],
                          risk_action:    bingo_hw_manager_risk_policy_i[5:4],
                          risk_epoch:     32'(bingo_hw_manager_risk_epoch_i),
                          boost_policy:   bingo_hw_manager_boost_policy_i[0],
                          boost_credit:   bingo_hw_manager_boost_policy_i[11:8],
                          boost_load_min: bingo_hw_manager_boost_policy_i[23:16]};
    bingo_hw_manager_ctrl #(
        .NumCores(NUM_CORES_PER_CLUSTER),
        .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
        .CoreIdWidth(cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)),
        .ClusterIdWidth(cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET)),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreTypeId(CoreTypeId),
        .SubstituteLevelMask(SubstituteLevelMask),
        .SubstituteL2TypeEn(SubstituteL2TypeEn),
        .SubstitutePolicy(SubstitutePolicy),
        .LoadWidth(CheckoutUsageWidth + 1),
        .ParkSupported(READY_AND_DONE_QUEUE_INTERFACE_TYPE == 1),
        .slot_state_t(slot_state_t),
        .ctrl_queue_t(ctrl_queue_t),
        .ctrl_host_t(ctrl_host_t),
        .ctrl_view_t(ctrl_view_t),
        .type_cfg_t(type_cfg_t),
        .smt_t(smt_t),
        .park_t(park_t),
        .cerf_fb_t(cerf_fb_t)
    ) i_ctrl (
        .clk_i            ( clk_i                             ),
        .rst_ni           ( rst_ni                            ),
        .slot_i           ( slot_state                        ),
        .late_i           ( wd_late                           ),
        .queue_i          ( ctrl_queue                        ),
        .host_i           ( ctrl_host                         ),
        .type_cfg_i       ( type_cfg                          ),
        .cfg_i            ( ctrl_cfg                          ),
        .pm_i             ( pm_view                           ),
        .cluster_access_i ( bingo_hw_manager_cluster_access_i ),
        .cerf_fb_done_i   ( cerf_fb_done                      ),
        .view_o           ( ctrl_view                         ),
        .smt_o            ( smt                               ),
        .smt_update_o     ( smt_update                        ),
        .park_o           ( park                              ),
        .cerf_fb_o        ( cerf_fb                           ),
        .cerf_fb_evt_o    ( cerf_fb_evt_o                     )
    );

    bingo_hw_manager_replay_ctrl #(
        .NumCores(NUM_CORES_PER_CLUSTER),
        .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
        .CoreIdWidth(cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)),
        .ClusterIdWidth(cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET)),
        .CoreTypeIdWidth(CoreTypeIdWidth),
        .CoreTypeId(CoreTypeId),
        .SubstituteLevelMask(SubstituteLevelMask)
    ) i_replay_ctrl (
        .clk_i                   ( clk_i                  ),
        .rst_ni                  ( rst_ni                 ),
        .fenced_i                ( core_fenced            ),
        .done_q_empty_i          ( done_q_empty           ),
        .checkout_empty_i        ( checkout_queue_empty   ),
        .checkout_full_i         ( checkout_queue_full    ),
        .ready_full_i            ( ready_queue_full       ),
        .checkout_logical_core_i    ( replay_head.logical_core    ),
        .checkout_logical_cluster_i ( replay_head.logical_cluster ),
        .checkout_no_exec_i         ( replay_head.no_exec         ),
        .checkout_no_replay_i       ( replay_head.no_replay       ),
        .checkout_exported_i        ( replay_head.exported        ),
        .checkout_imported_i        ( checkout_head_imported      ),
        .export_type_ready_i        ( export_type_ready           ),
        .remote_type_en_i           ( remote_export_type_en       ),
        .bounce_ready_i             ( !reject_pending             ),
        .smt_found_i                ( smt.found                   ),
        .smt_core_i                 ( smt.core                    ),
        .smt_cluster_i              ( smt.cluster                 ),
        .smt_update_i               ( smt_update                  ),
        .retired_o               ( core_retired           ),
        .ready_flush_o           ( replay_ready_flush     ),
        .move_o                  ( replay_move            ),
        .hold_o                  ( replay_hold_slot       ),
        .move_fire_o             ( replay_move_fire       ),
        .push_ready_o            ( replay_push_ready_q    ),
        .src_core_o              ( replay_src_core        ),
        .src_cluster_o           ( replay_src_cluster     ),
        .dst_core_o              ( replay_dst_core        ),
        .dst_cluster_o           ( replay_dst_cluster     ),
        .stuck_o                 ( replay_stuck_slot      ),
        .blocked_o               ( replay_blocked_o       ),
        .rotate_o                ( replay_rotate          ),
        .export_o                ( replay_export          ),
        .bounce_o                ( replay_bounce          ),
        .exec_moved_o            ( replay_exec_moved      )
    );
    assign replay_stuck   = (|replay_stuck_slot) || (|remote_rejected_q);
    assign replay_stuck_o = replay_stuck;
    // One MOVE step of the replay controller, and what it does to the queues
    replay_step_t replay_step;
    replay_op_t   replay_op;
    assign replay_step = '{move:        replay_move,
                           fire:        replay_move_fire,
                           push_ready:  replay_push_ready_q,
                           bounce:      replay_bounce,
                           rotate:      replay_rotate,
                           exported:    replay_export,
                           src_core:    replay_src_core,
                           src_cluster: replay_src_cluster,
                           dst_core:    replay_dst_core,
                           dst_cluster: replay_dst_cluster};
    bingo_hw_manager_replay_path #(
        .NumCores        ( NUM_CORES_PER_CLUSTER                            ),
        .NumClusters     ( NUM_CLUSTERS_PER_CHIPLET                         ),
        .CoreIdWidth     ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
        .ClusterIdWidth  ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
        .CoreTypeIdWidth ( CoreTypeIdWidth                                  ),
        .CoreTypeId      ( CoreTypeId                                       ),
        .RemoteEn        ( RemoteEn                                         ),
        .task_desc_t     ( bingo_hw_manager_task_desc_t                     ),
        .checkout_t      ( checkout_t                                       ),
        .smt_t           ( smt_t                                            ),
        .slot_state_t    ( slot_state_t                                     ),
        .replay_head_t   ( replay_head_t                                    ),
        .replay_step_t   ( replay_step_t                                    ),
        .replay_op_t     ( replay_op_t                                      )
    ) i_replay_path (
        .checkout_i       ( checkout_queue        ),
        .smt_i            ( smt                   ),
        .remote_type_en_i ( remote_export_type_en ),
        .head_o           ( replay_head           ),
        .slot_i           ( slot_state            ),
        .step_i           ( replay_step           ),
        .op_o             ( replay_op             ),
        .pending_o        ( replay_pending        ),
        .move_cluster_o   ( replay_move_cluster   )
    );
    assign replay_data        = replay_op.data;
    assign replay_exit_absorb = replay_op.exit_absorb;
    assign replay_pop         = replay_op.pop;
    assign replay_push        = replay_op.push;
    assign replay_push_ready  = replay_op.push_ready;

    //////////////////////////////////////////////////////////////////////
    // Placement of new tasks
    //////////////////////////////////////////////////////////////////////
    // Each checked waiting queue head goes to its logical slot, to the
    // substitute the control plane chose for it (retired or parked slot), or
    // stays on its proxy slot and is exported (level 3); see
    // bingo_hw_manager_router.
    bingo_hw_manager_router #(
        .NumCores            ( NUM_CORES_PER_CLUSTER                            ),
        .NumClusters         ( NUM_CLUSTERS_PER_CHIPLET                         ),
        .CoreIdWidth         ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
        .ClusterIdWidth      ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
        .CoreTypeIdWidth     ( CoreTypeIdWidth                                  ),
        .CoreTypeId          ( CoreTypeId                                       ),
        .SubstituteLevelMask ( SubstituteLevelMask                              ),
        .RemoteEn            ( RemoteEn                                         ),
        .OutstandingWidth    ( RemapOutstandingWidth                            ),
        .waiting_t           ( waiting_t                                        ),
        .slot_state_t        ( slot_state_t                                     ),
        .smt_t               ( smt_t                                            ),
        .park_t              ( park_t                                           ),
        .checkout_t          ( checkout_t                                       ),
        .ready_t             ( ready_t                                          ),
        .select_t            ( select_t                                         ),
        .route_t             ( route_t                                          )
    ) i_router (
        .waiting_i             ( waiting_heads                            ),
        .offer_valid_i         ( demux_ready_and_checkout_queue_oup_valid ),
        .offer_ready_o         ( demux_ready_and_checkout_queue_oup_ready ),
        .slot_i                ( slot_state                               ),
        .remote_type_en_i      ( remote_export_type_en                    ),
        .smt_i                 ( smt                                      ),
        .smt_update_i          ( smt_update                               ),
        .park_i                ( park                                     ),
        .outstanding_i         ( remap_outstanding_q                      ),
        .replay_pending_i      ( replay_pending                           ),
        .replay_move_cluster_i ( replay_move_cluster                      ),
        .checkout_i            ( checkout_queue                           ),
        .ready_i               ( ready_queue                              ),
        .export_grant_i        ( route_remote_grant                       ),
        .select_o              ( remap_select                             ),
        .route_o               ( route                                    )
    );

    // Tasks of each logical slot that sit in the checkout queue of another slot
    bingo_hw_manager_remap_count #(
        .NumCores          ( NUM_CORES_PER_CLUSTER                            ),
        .NumClusters       ( NUM_CLUSTERS_PER_CHIPLET                         ),
        .CoreIdWidth       ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
        .ClusterIdWidth    ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
        .Width             ( RemapOutstandingWidth                            ),
        .ForeignStuckDrain ( ForeignStuckDrain                                ),
        .task_desc_t       ( bingo_hw_manager_task_desc_t                     ),
        .checkout_t        ( checkout_t                                       )
    ) i_remap_count (
        .clk_i         ( clk_i               ),
        .rst_ni        ( rst_ni              ),
        .checkout_i    ( checkout_queue      ),
        .outstanding_o ( remap_outstanding_q ),
        .moved_in_o    ( moved_in_q          )
    );

    //////////////////////////////////////////////////////////////////////
    // Level 3: remote dispatch (SubstituteLevelMask[2])
    //////////////////////////////////////////////////////////////////////
    // Origin side (bingo_hw_manager_remote_export): a task of a fenced slot D
    // that no live core of this chiplet may run stays in D's checkout queue,
    // marked exported, and a copy goes to another chiplet. D is its proxy: the
    // remote done comes back into D's done queue and retires the head with the
    // normal dep_set. A proxy whose export is rejected or times out stops
    // (bingo_hw_manager_remote_proxy).
    // Executor side: an imported task runs on a live core of its type here, as
    // a task of the lowest slot of that type, and its done (or a reject) goes
    // back to its origin (bingo_hw_manager_remote_reply).
    bingo_hw_manager_task_desc_full_t [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] route_desc_full;
    // The proxy slots of the exported tasks, and the exports to the transport
    proxy_t                                                               proxy;
    remote_dispatch_out_t                                                 remote_dispatch_out;
    remote_dispatch_in_t                                                  remote_dispatch_in;
    // Level 3, executor side: a reject for the origin (bingo_hw_manager_remote_reply)
    logic                                                                 import_reject_valid;
    remote_done_t                                                         import_reject_out;
    always_comb begin
        for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++)
            for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++)
                route_desc_full[c][cl] = desc_to_full(waiting_dep_check_task_desc[route.src_core[c][cl]]);
    end
    bingo_hw_manager_remote_export #(
        .NumCores          ( NUM_CORES_PER_CLUSTER                            ),
        .NumClusters       ( NUM_CLUSTERS_PER_CHIPLET                         ),
        .RemoteEn          ( RemoteEn                                         ),
        .CoreTypeIdWidth   ( CoreTypeIdWidth                                  ),
        .CoreTypeId        ( CoreTypeId                                       ),
        .CoreIdWidth       ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
        .ClusterIdWidth    ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
        .NumPeers          ( RemoteNumPeers                                   ),
        .PeerIdWidth       ( RemotePeerIdWidth                                ),
        .FifoDepth         ( RemoteExportFifoDepth                            ),
        .task_desc_full_t  ( bingo_hw_manager_task_desc_full_t                ),
        .remote_slot_t     ( remote_slot_t                                    ),
        .remote_export_t   ( remote_export_t                                  ),
        .chip_id_t         ( chip_id_t                                        ),
        .replay_step_t     ( replay_step_t                                    ),
        .route_t           ( route_t                                          ),
        .checkout_t        ( checkout_t                                       ),
        .remote_dispatch_t ( remote_dispatch_out_t                            )
    ) i_remote_export (
        .clk_i            ( clk_i                         ),
        .rst_ni           ( rst_ni                        ),
        .chip_id_i        ( chip_id_i                     ),
        .step_i           ( replay_step                   ),
        .replay_desc_i    ( desc_to_full(replay_data)     ),
        .route_i          ( route                         ),
        .route_desc_i     ( route_desc_full               ),
        .checkout_i       ( checkout_queue                ),
        .route_grant_o    ( route_remote_grant            ),
        .type_ready_o     ( export_type_ready             ),
        .export_push_o    ( export_push                   ),
        .export_o         ( export_in                     ),
        .type_peer_i      ( remote_export_type_peer_i     ),
        .peer_ready_i     ( remote_export_peer_ready_i    ),
        .dispatch_valid_o ( remote_dispatch_valid_o       ),
        .dispatch_ready_i ( remote_dispatch_ready_i       ),
        .dispatch_o       ( remote_dispatch_out           )
    );
    bingo_hw_manager_remote_proxy #(
        .NumCores      ( NUM_CORES_PER_CLUSTER    ),
        .NumClusters   ( NUM_CLUSTERS_PER_CHIPLET ),
        .RemoteEn      ( RemoteEn                 ),
        .checkout_t    ( checkout_t               ),
        .done_t        ( done_t                   ),
        .done_remote_t ( done_remote_t            ),
        .proxy_t       ( proxy_t                  )
    ) i_remote_proxy (
        .clk_i            ( clk_i                       ),
        .rst_ni           ( rst_ni                      ),
        .checkout_i       ( checkout_queue              ),
        .head_mismatch_i  ( remote_head_mismatch        ),
        .done_i           ( done_queue                  ),
        .remote_i         ( done_remote                 ),
        .timeout_cycles_i ( 32'(remote_proxy_timeout_i) ),
        .proxy_o          ( proxy                       )
    );
    assign remote_dispatch_desc_o        = host_axi_lite_data_t'(remote_dispatch_out.desc);
    assign remote_dispatch_core_type_o   = remote_dispatch_out.core_type;
    assign remote_dispatch_origin_chip_o = remote_dispatch_out.origin_chip;
    assign remote_dispatch_proxy_slot_o  = remote_dispatch_out.proxy_slot;
    assign remote_rejected_q             = proxy.rejected;
    assign remote_timeout_o              = proxy.timeout_any;
    assign remote_done_mismatch_o        = proxy.mismatch;

    assign remote_dispatch_in = '{desc:        full_to_desc(bingo_hw_manager_task_desc_full_t'(remote_dispatch_desc_i)),
                                  core_type:   remote_dispatch_core_type_i,
                                  origin_chip: remote_dispatch_origin_chip_i,
                                  proxy_slot:  remote_dispatch_proxy_slot_i};
    bingo_hw_manager_remote_import #(
        .NumCores                  ( NUM_CORES_PER_CLUSTER                            ),
        .NumClusters               ( NUM_CLUSTERS_PER_CHIPLET                         ),
        .RemoteEn                  ( RemoteEn                                         ),
        .CoreTypeIdWidth           ( CoreTypeIdWidth                                  ),
        .CoreTypeId                ( CoreTypeId                                       ),
        .ImportSubstituteLevelMask ( ImportSubstituteLevelMask                        ),
        .SubstituteL2TypeEn        ( SubstituteL2TypeEn                               ),
        .CoreIdWidth               ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
        .ClusterIdWidth            ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
        .task_desc_t               ( bingo_hw_manager_task_desc_t                     ),
        .remote_tag_t              ( remote_tag_t                                     ),
        .remote_done_t             ( remote_done_t                                    ),
        .chip_id_t                 ( chip_id_t                                        ),
        .remote_dispatch_t         ( remote_dispatch_in_t                             ),
        .slot_state_t              ( slot_state_t                                     ),
        .replay_step_t             ( replay_step_t                                    ),
        .route_t                   ( route_t                                          ),
        .checkout_t                ( checkout_t                                       ),
        .ready_t                   ( ready_t                                          ),
        .import_t                  ( import_t                                         )
    ) i_remote_import (
        .clk_i               ( clk_i                                                        ),
        .rst_ni              ( rst_ni                                                       ),
        .chip_id_i           ( chip_id_i                                                    ),
        .dispatch_valid_i    ( remote_dispatch_valid_i                                      ),
        .dispatch_ready_o    ( remote_dispatch_ready_o                                      ),
        .dispatch_i          ( remote_dispatch_in                                           ),
        .slot_i              ( slot_state                                                   ),
        .replay_pending_i    ( replay_pending                                               ),
        .step_i              ( replay_step                                                  ),
        .replay_push_i       ( replay_push                                                  ),
        .route_i             ( route                                                        ),
        .checkout_i          ( checkout_queue                                               ),
        .ready_i             ( ready_queue                                                  ),
        .import_o            ( imported                                                     ),
        .bounce_tag_i        ( checkout_remote_tag_out[replay_src_core][replay_src_cluster] ),
        .bounce_desc_i       ( replay_data                                                  ),
        .reject_valid_o      ( import_reject_valid                                          ),
        .reject_o            ( import_reject_out                                            ),
        .reject_pending_i    ( reject_pending                                               )
    );
    bingo_hw_manager_remote_reply #(
        .NumSlots      ( NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET ),
        .RemoteEn      ( RemoteEn                                         ),
        .remote_done_t ( remote_done_t                                    )
    ) i_remote_reply (
        .clk_i               ( clk_i                 ),
        .rst_ni              ( rst_ni                ),
        .reject_valid_i      ( import_reject_valid   ),
        .reject_i            ( import_reject_out     ),
        .reject_pending_o    ( reject_pending        ),
        .done_i              ( remote_done_arb_data  ),
        .done_valid_i        ( remote_done_arb_valid ),
        .done_ready_o        ( remote_done_arb_ready ),
        .remote_done_valid_o ( remote_done_valid_o   ),
        .remote_done_ready_i ( remote_done_ready_i   ),
        .remote_done_o       ( remote_done_out       )
    );
    assign remote_done_chip_o       = remote_done_out.chip;
    assign remote_done_proxy_slot_o = remote_done_out.proxy_slot;
    assign remote_done_task_id_o    = remote_done_out.task_id;
    assign remote_done_reject_o     = remote_done_out.reject;

    //////////////////////////////////////////////////////////////////////
    // Control event log (observation only)
    //////////////////////////////////////////////////////////////////////
    if (EventLogDepth != 0) begin : gen_evlog
        localparam int unsigned Types = 2**CoreTypeIdWidth;
        localparam int unsigned Events = EvlogGroups * NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET + Types;
        logic [Events-1:0]       events;
        logic [Events-1:0][15:0] args;
        evlog_move_t             move;
        // The threshold that would confirm a fence of the slot now, the argument
        // of its fence record: 3 learned, 1 at risk, 2 type or parameter, 0 none
        logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][1:0] fence_reason;
        always_comb begin
            for (int c = 0; c < NUM_CORES_PER_CLUSTER; c++)
                for (int cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++)
                    fence_reason[c][cl] = wd_adapt_confirm[c][cl] ? 2'd3 :
                        (ctrl_view.risk[c][cl] && wd_risk_confirm_valid[c][cl] &&
                        bingo_hw_manager_risk_confirm_i != 0) ? 2'd1 :
                        ((wd_confirm_thr[c][cl] != 0) ? 2'd2 : 2'd0);
        end
        bingo_hw_manager_evlog_events #(
            .NumCores        ( NUM_CORES_PER_CLUSTER                            ),
            .NumClusters     ( NUM_CLUSTERS_PER_CHIPLET                         ),
            .CoreIdWidth     ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
            .ClusterIdWidth  ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
            .CoreTypeIdWidth ( CoreTypeIdWidth                                  ),
            .task_desc_t     ( bingo_hw_manager_task_desc_t                     ),
            .slot_state_t    ( slot_state_t                                     ),
            .ctrl_view_t     ( ctrl_view_t                                      ),
            .park_t          ( park_t                                           ),
            .checkout_t      ( checkout_t                                       ),
            .type_cfg_t      ( type_cfg_t                                       ),
            .cerf_fb_t       ( cerf_fb_t                                        ),
            .replay_step_t   ( replay_step_t                                    )
        ) i_events (
            .clk_i          ( clk_i             ),
            .rst_ni         ( rst_ni            ),
            .slot_i         ( slot_state        ),
            .suspect_i      ( core_dead_suspect ),
            .blocked_i      ( replay_blocked_o  ),
            .view_i         ( ctrl_view         ),
            .park_i         ( park              ),
            .fence_reason_i ( fence_reason      ),
            .checkout_i     ( checkout_queue    ),
            .cerf_fb_i      ( cerf_fb           ),
            .cerf_fb_done_i ( cerf_fb_done      ),
            .type_cfg_i     ( type_cfg          ),
            .cerf_fb_evt_i  ( cerf_fb_evt_o     ),
            .step_i         ( replay_step       ),
            .move_desc_i    ( replay_data       ),
            .events_o       ( events            ),
            .args_o         ( args              ),
            .move_o         ( move              )
        );
        bingo_hw_manager_evlog #(
            .NumCores(NUM_CORES_PER_CLUSTER),
            .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
            .NumTypes(Types),
            .Depth(EventLogDepth),
            .AgeWidth(EventLogAgeWidth)
        ) i_evlog (
            .clk_i, .rst_ni, .enable_i(evlog_enable_i),
            .clear_i(evlog_clear_i), .pop_i(evlog_pop_i),
            .event_i(events), .arg_i(args),
            .move_i(move.valid), .move_code_i(move.code), .move_slot_i(move.slot), .move_arg_i(move.arg),
            .head_o(evlog_head_o), .count_o(evlog_count_o), .dropped_o(evlog_dropped_o)
        );
    end else begin : gen_no_evlog
        assign evlog_head_o = '0;
        assign evlog_count_o = '0;
        assign evlog_dropped_o = '0;
    end

`ifndef SYNTHESIS
    // Simulation-only event log: makes watchdog / remap / replay activity visible
    // in any flow that simulates this RTL (bingo unit tests and full-system sims).
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_dead_suspect_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_fenced_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_retired_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_stuck_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_blocked_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_hold_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_parked_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_fail_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] park_unpark_log_q;
    always @(posedge clk_i or negedge rst_ni) begin : watchdog_remap_event_log
        if (!rst_ni) begin
            core_dead_suspect_log_q <= '0;
            core_fenced_log_q       <= '0;
            core_retired_log_q      <= '0;
            replay_stuck_log_q      <= '0;
            replay_blocked_log_q    <= '0;
            park_hold_log_q         <= '0;
            park_parked_log_q       <= '0;
            park_fail_log_q         <= '0;
            park_unpark_log_q       <= '0;
        end else begin
            core_dead_suspect_log_q <= core_dead_suspect;
            park_hold_log_q         <= park.hold;
            park_parked_log_q       <= park.parked;
            park_fail_log_q         <= park.fail;
            park_unpark_log_q       <= park.unpark;
            core_fenced_log_q       <= core_fenced;
            core_retired_log_q      <= core_retired;
            replay_stuck_log_q      <= replay_stuck_slot;
            replay_blocked_log_q    <= replay_blocked_o;
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                    if (exit_absorb[c][cl]) begin
                        $display("[BINGO_EXIT_ABSORB] %0t chip=%0d task=%0d logical_core=%0d logical_cluster=%0d core=%0d cluster=%0d",
                                 $time, chip_id_i, checkout_queue_data_in[c][cl].task_id,
                                 checkout_queue_data_in[c][cl].assigned_core_id,
                                 checkout_queue_data_in[c][cl].assigned_cluster_id, c, cl);
                    end
                    if ((core_dead_suspect[c][cl] != core_dead_suspect_log_q[c][cl]) ||
                        (core_fenced[c][cl] != core_fenced_log_q[c][cl])) begin
                        $display("[BINGO_WD] %0t chip=%0d core=%0d cluster=%0d dead_suspect=%0b fenced=%0b",
                                 $time, chip_id_i, c, cl, core_dead_suspect[c][cl], core_fenced[c][cl]);
                    end
                    if (park.hold[c][cl] && !park_hold_log_q[c][cl]) begin
                        $display("[BINGO_PARK] %0t chip=%0d core=%0d cluster=%0d HOLD",
                                 $time, chip_id_i, c, cl);
                    end
                    if (park.parked[c][cl] && !park_parked_log_q[c][cl]) begin
                        $display("[BINGO_PARK] %0t chip=%0d core=%0d cluster=%0d PARKED -> core=%0d cluster=%0d",
                                 $time, chip_id_i, c, cl, smt.core[c][cl], smt.cluster[c][cl]);
                    end
                    if (park.fail[c][cl] && !park_fail_log_q[c][cl]) begin
                        $display("[BINGO_PARK] %0t chip=%0d core=%0d cluster=%0d FAIL",
                                 $time, chip_id_i, c, cl);
                    end
                    if (park.unpark[c][cl] && !park_unpark_log_q[c][cl]) begin
                        $display("[BINGO_PARK] %0t chip=%0d core=%0d cluster=%0d UNPARK (%0d tasks left elsewhere)",
                                 $time, chip_id_i, c, cl, remap_outstanding_q[c][cl]);
                    end
                    if (park_parked_log_q[c][cl] && !park.parked[c][cl] && park_unpark_log_q[c][cl] &&
                        !core_fenced[c][cl] && !park.fail[c][cl]) begin
                        $display("[BINGO_PARK] %0t chip=%0d core=%0d cluster=%0d UNPARKED",
                                 $time, chip_id_i, c, cl);
                    end
                    if ((park_hold_log_q[c][cl] || park_parked_log_q[c][cl]) &&
                        !park.hold[c][cl] && !park.parked[c][cl] && core_fenced[c][cl]) begin
                        $display("[BINGO_PARK] %0t chip=%0d core=%0d cluster=%0d dropped, fenced",
                                 $time, chip_id_i, c, cl);
                    end
                    if (core_retired[c][cl] && !core_retired_log_q[c][cl]) begin
                        $display("[BINGO_RETIRED] %0t chip=%0d core=%0d cluster=%0d",
                                 $time, chip_id_i, c, cl);
                    end
                    if (replay_stuck_slot[c][cl] && !replay_stuck_log_q[c][cl]) begin
                        $display("[BINGO_REPLAY_STUCK] %0t chip=%0d core=%0d cluster=%0d: no live core may run task %0d (logical core %0d)",
                                 $time, chip_id_i, c, cl,
                                 checkout_queue_data_out[c][cl].task_id,
                                 checkout_queue_data_out[c][cl].assigned_core_id);
                    end
                    if (replay_blocked_o[c][cl] && !replay_blocked_log_q[c][cl]) begin
                        $display("[BINGO_REPLAY_BLOCKED] %0t chip=%0d core=%0d cluster=%0d task=%0d",
                                 $time, chip_id_i, c, cl, checkout_queue_data_out[c][cl].task_id);
                    end
                    if (route.fire[c][cl] &&
                        ((route.src_core[c][cl] != bingo_hw_manager_assigned_core_id_t'(c)) ||
                         (waiting_dep_check_task_desc[route.src_core[c][cl]].assigned_cluster_id !=
                          bingo_hw_manager_assigned_cluster_id_t'(cl)))) begin
                        $display("[BINGO_REMAP] %0t chip=%0d task=%0d logical_core=%0d -> physical_core=%0d cluster=%0d logical_cluster=%0d",
                                 $time, chip_id_i,
                                 waiting_dep_check_task_desc[route.src_core[c][cl]].task_id,
                                 route.src_core[c][cl], c, cl,
                                 waiting_dep_check_task_desc[route.src_core[c][cl]].assigned_cluster_id);
                    end
                end
            end
            if (export_push) begin
                $display("[BINGO_EXPORT] %0t chip=%0d task=%0d type_id=%0d proxy_slot=%0d (replay %0b)",
                         $time, chip_id_i, export_in.desc.task_id, export_in.core_type,
                         export_in.proxy_slot, replay_export);
            end
            if (imported.fire) begin
                $display("[BINGO_IMPORT] %0t chip=%0d task=%0d from chip=%0d proxy_slot=%0d -> core=%0d cluster=%0d (home core=%0d cluster=%0d)",
                         $time, chip_id_i, imported.desc.task_id, remote_dispatch_origin_chip_i,
                         remote_dispatch_proxy_slot_i, imported.core, imported.cluster,
                         imported.home_core, imported.home_cluster);
            end
            if (remote_done_valid_o && remote_done_ready_i && !remote_done_reject_o) begin
                $display("[BINGO_REMOTE_DONE_OUT] %0t chip=%0d task=%0d -> chip=%0d proxy_slot=%0d",
                         $time, chip_id_i, remote_done_task_id_o, remote_done_chip_o, remote_done_proxy_slot_o);
            end
            if (remote_done_valid_i && remote_done_ready_o && !remote_done_reject_i) begin
                $display("[BINGO_REMOTE_DONE_IN] %0t chip=%0d task=%0d proxy_slot=%0d",
                         $time, chip_id_i, remote_done_task_id_i, remote_done_proxy_slot_i);
            end
            if (imported.reject) begin
                $display("[BINGO_REMOTE_REJECT_OUT] %0t chip=%0d task=%0d -> chip=%0d proxy_slot=%0d (no live core of its type)",
                         $time, chip_id_i, imported.desc.task_id, remote_dispatch_origin_chip_i,
                         remote_dispatch_proxy_slot_i);
            end
            if (replay_bounce) begin
                $display("[BINGO_REMOTE_REJECT_OUT] %0t chip=%0d task=%0d -> chip=%0d proxy_slot=%0d (its core died, no other live core)",
                         $time, chip_id_i, replay_data.task_id,
                         checkout_remote_tag_out[replay_src_core][replay_src_cluster].origin_chip,
                         checkout_remote_tag_out[replay_src_core][replay_src_cluster].proxy_slot);
            end
            if (done_remote.stray) begin
                $display("[BINGO_REMOTE_DONE_DROP] %0t chip=%0d task=%0d proxy_slot=%0d reject=%0b: no proxy slot, dropped",
                         $time, chip_id_i, remote_done_task_id_i, remote_done_proxy_slot_i, remote_done_reject_i);
            end
            if (|done_remote.reject) begin
                $display("[BINGO_REMOTE_REJECT_IN] %0t chip=%0d task=%0d proxy_slot=%0d: the proxy is stuck",
                         $time, chip_id_i, remote_done_task_id_i, remote_done_proxy_slot_i);
            end
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                    if (proxy.timeout[c][cl]) begin
                        $display("[BINGO_REMOTE_TIMEOUT] %0t chip=%0d task=%0d proxy_slot=%0d: no remote done for %0d cycles, the proxy is stuck",
                                 $time, chip_id_i, checkout_queue_data_out[c][cl].task_id,
                                 c + cl * NUM_CORES_PER_CLUSTER, proxy.wait_cycles[c][cl]);
                    end
                end
            end
            if (replay_move_fire && !replay_bounce) begin
                $display("[BINGO_REPLAY] %0t chip=%0d task=%0d type=%0d logical_core=%0d from=%0d to=%0d cluster=%0d to_cluster=%0d logical_cluster=%0d",
                         $time, chip_id_i, replay_data.task_id, replay_data.task_type,
                         replay_data.assigned_core_id, replay_src_core, replay_dst_core, replay_src_cluster,
                         replay_dst_cluster, replay_data.assigned_cluster_id);
            end
        end
    end

    // Invariants of the replay, remap and retire paths
    slot_state_t slot_log_q;
    assign slot_log_q = '{fenced:   core_fenced_log_q,
                          retired:  core_retired_log_q,
                          stuck:    replay_stuck_log_q,
                          rejected: '0};
    bingo_hw_manager_top_assert #(
        .NumCores           ( NUM_CORES_PER_CLUSTER                            ),
        .NumClusters        ( NUM_CLUSTERS_PER_CHIPLET                         ),
        .CheckoutQueueDepth ( CheckoutQueueDepth                               ),
        .ForeignStuckDrain  ( ForeignStuckDrain                                ),
        .CoreIdWidth        ( cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)    ),
        .ClusterIdWidth     ( cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET) ),
        .OutstandingWidth   ( RemapOutstandingWidth                            ),
        .task_desc_t        ( bingo_hw_manager_task_desc_t                     ),
        .remote_export_t    ( remote_export_t                                  ),
        .chip_id_t          ( chip_id_t                                        ),
        .slot_state_t       ( slot_state_t                                     ),
        .replay_step_t      ( replay_step_t                                    ),
        .replay_op_t        ( replay_op_t                                      ),
        .waiting_t          ( waiting_t                                        ),
        .route_t            ( route_t                                          ),
        .ready_t            ( ready_t                                          ),
        .checkout_t         ( checkout_t                                       ),
        .done_t             ( done_t                                           ),
        .done_remote_t      ( done_remote_t                                    )
    ) i_top_assert (
        .clk_i              ( clk_i                      ),
        .rst_ni             ( rst_ni                     ),
        .chip_id_i          ( chip_id_i                  ),
        .task_queue_empty_i ( task_queue_mbox_empty      ),
        .task_queue_pop_i   ( task_queue_mbox_pop        ),
        .task_head_i        ( cur_task_desc              ),
        .slot_i             ( slot_state                 ),
        .blocked_i          ( replay_blocked_o           ),
        .slot_log_i         ( slot_log_q                 ),
        .step_i             ( replay_step                ),
        .replay_op_i        ( replay_op                  ),
        .exec_moved_i       ( replay_exec_moved          ),
        .export_push_i      ( export_push                ),
        .export_i           ( export_in                  ),
        .waiting_i          ( waiting_heads              ),
        .route_i            ( route                      ),
        .park_block_i       ( park_block                 ),
        .outstanding_i      ( remap_outstanding_q        ),
        .moved_in_i         ( moved_in_q                 ),
        .ready_i            ( ready_queue                ),
        .checkout_i         ( checkout_queue             ),
        .stuck_drain_i      ( stuck_drain                ),
        .stuck_head_skip_i  ( stuck_head_skip            ),
        .done_i             ( done_queue                 ),
        .done_pop_i         ( done_q_pop                 ),
        .remote_i           ( done_remote                )
    );
`endif

    //////////////////////////////////////////////////////////////////////
    // DARTS Tier 3: Load Monitor
    //////////////////////////////////////////////////////////////////////
    bingo_hw_manager_load_monitor #(
        .NumCores   (NUM_CORES_PER_CLUSTER),
        .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
        .CounterWidth(8)
    ) i_load_monitor (
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        .task_dispatched_i  (ready_queue_pop),
        .task_done_i        (done_q_push),
        .clear_i            (ctrl_view.load_clear),
        .pending_per_core_o (/* CSR readable — connect when needed */),
        .total_pending_o    (load_total_pending_o)
    );

    //////////////////////////////////////////////////////////////////////
    // DARTS Tier 1: Conditional Execution Register File (CERF)
    //////////////////////////////////////////////////////////////////////

    bingo_hw_manager_cond_exec_controller #(
        .NumGroups(32)
    ) i_cerf (
        .clk_i            ( clk_i                  ),
        .rst_ni           ( rst_ni                 ),
        .cerf_state_o     ( cerf_state             ),
        .cerf_write_data_i( cerf_write_data_i      ),
        .cerf_write_en_i  ( cerf_write_en_i        ),
        .policy_req_i     ( cerf_fb.req            ),
        .policy_clear_i   ( cerf_fb.clear          ),
        .policy_set_i     ( cerf_fb.set            ),
        .policy_done_o    ( cerf_fb_done           )
    );

endmodule
