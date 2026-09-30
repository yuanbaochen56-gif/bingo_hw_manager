// Copyright 2025 KU Leuven.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Fanchen Kong <fanchen.kong@kuleuven.be>
// - Xiaoling Yi  <xiaoling.yi@kuleuven.be>
// - Yunhao Deng  <yunhao.deng@kuleuven.be>

module bingo_hw_manager_top #(
    // Top-level parameters can be defined here
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
    // L1 at local addresses) a different type per cluster. Type 0: the slot
    // neither hands over its tasks nor takes over others' (e.g. a host slot).
    // Default: all cores of the chiplet are interchangeable.
    parameter int unsigned CoreTypeIdWidth = 4,
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
    // Levels an imported task (level 3, from another chiplet) may use to find a
    // live stand-in for its home slot here (bit 2 is ignored: never re-exported).
    // Default: the local levels of SubstituteLevelMask. Setting it apart from
    // SubstituteLevelMask is a debug / loopback aid: e.g. SubstituteLevelMask =
    // 3'b100 with ImportSubstituteLevelMask = 3'b001 exports every task of a
    // fenced core (force remote) and still lets it run on a same-cluster core
    // when it comes back through a loopback link.
    parameter logic [2:0] ImportSubstituteLevelMask = SubstituteLevelMask & 3'b011,
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
    // Address Offsets
    parameter int unsigned ReadyQueueAddrOffset = 4096,
    // Dependent parameters, DO NOT OVERRIDE!
    parameter type chip_id_t = logic [ChipIdWidth-1:0],
    parameter type host_axi_lite_addr_t = logic [HostAxiLiteAddrWidth-1:0],
    parameter type host_axi_lite_data_t = logic [HostAxiLiteDataWidth-1:0],
    parameter type device_axi_lite_addr_t = logic [DeviceAxiLiteAddrWidth-1:0],
    parameter type device_axi_lite_data_t = logic [DeviceAxiLiteDataWidth-1:0],
    // Flat slot id (core + cluster * NUM_CORES_PER_CLUSTER) of the remote interface
    parameter int unsigned RemoteSlotIdWidth = cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET)
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
    // Control signals to start the HW Manager
    // The start signals are from the reg gen modules
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
    /// CSR Req/Resp Interface for ready queue and the done queue
    // CSR Will Read from the ready queue and write to the done queue
    input  csr_req_t                            [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_req_i,
    input  logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_req_valid_i,
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_req_ready_o,
    output csr_rsp_t                            [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_rsp_o,
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_rsp_valid_o,
    input  logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    csr_rsp_ready_i,
    /// The interface to the Power Management Module
    // Host configuration interface
    input device_axi_lite_data_t                bingo_hw_manager_enable_idle_pm_i,
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
    // DARTS: Load Monitor output (CSR readable)
    output logic [10:0]                         load_total_pending_o,
    // Watchdog: confirmed dead cores (sticky). The system may reset / isolate them.
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    core_fenced_o,
    // Replay: a fenced core holds a task that no live core may run (level 3:
    // on this chiplet, or on the remote chiplet that rejected its export)
    output logic                                replay_stuck_o,
    // Watchdog: busy cores without heartbeat for WatchdogHeartbeatTimeoutCycles
    // (not sticky, cleared by a heartbeat or a done)
    output logic                                [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    core_dead_suspect_o,
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
    // waiting forever in the export FIFO. Default: every type.
    input  logic [2**CoreTypeIdWidth-1:0]       remote_export_type_en_i = '1,
    // Sticky: the done at the head of a proxy slot's done queue does not
    // belong to its exported head task (the slot then stops retiring)
    output logic                                remote_done_mismatch_o
);
    // --------Type definitions and signal declarations--------------------//
    // ---- Start of Type definitions -------------------------------------//
    // Task Type (DARTS: expanded to 2 bits for gating support)
    // 2'b00: Normal Task
    // 2'b01: Dummy Task (set/check synchronization)
    // 2'b10: Gating Task (executes on core, writes CERF on completion)
    // 2'b11: Reserved
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

    localparam int unsigned TaskDescWidth = $bits(bingo_hw_manager_task_desc_t);
    localparam int unsigned ReservedBitsForTaskDesc = HostAxiLiteDataWidth - TaskDescWidth;
    if (TaskDescWidth>HostAxiLiteDataWidth) begin : gen_task_desc_width_check
        initial begin
        $error("Task Decriptor width (%0d) exceeds Host AXI Lite Data Width (%0d)! Please adjust the parameters accordingly.", TaskDescWidth, HostAxiLiteDataWidth);
        $finish;
        end
    end
    // 64bit Task Descriptor with reserved bits
    typedef struct packed{
        logic [ReservedBitsForTaskDesc-1:0]          reserved_bits;
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
        return f;
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

    ////////////////////////////////
    // Ready Queue Filter Signals
    ////////////////////////////////
    logic                                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_filter_inp_valid;
    logic                                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_filter_inp_ready;
    logic                                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_filter_drop;
    logic                                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_filter_oup_valid;
    logic                                   [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_filter_oup_ready;

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
    device_axi_lite_addr_t                  [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] ready_queue_base_addr;
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
    logic                                                                                    remote_done_mismatch_q;
    logic                          [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_retire_valid;
    // Level 3 export (origin side)
    logic [NUM_CORES_PER_CLUSTER-1:0]                                       remap_remote;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]         route_remote;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]         route_remote_grant;
    logic                                  replay_rotate;
    logic                                  replay_export;
    logic                                  replay_bounce;
    remote_export_t                        export_in;
    remote_export_t                        export_out;
    logic                                  export_push;
    logic                                  export_full;
    logic                                  export_empty;
    // Level 3 import (executor side)
    logic                                  import_home_found;
    bingo_hw_manager_assigned_core_id_t    import_home_core;
    bingo_hw_manager_assigned_cluster_id_t import_home_cluster;
    logic                                  import_sub_found;
    bingo_hw_manager_assigned_core_id_t    import_core;
    bingo_hw_manager_assigned_cluster_id_t import_cluster;
    logic                                  import_fire;
    logic                                  import_reject;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] import_push;
    bingo_hw_manager_task_desc_t           import_desc;
    // Level 3 remote dones
    remote_done_t [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] remote_done_arb_data;
    logic         [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] remote_done_arb_valid;
    logic         [NUM_CORES_PER_CLUSTER*NUM_CLUSTERS_PER_CHIPLET-1:0] remote_done_arb_ready;
    remote_done_t                                                      remote_done_out;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    remote_done_push;
    // Level 3 rejects: executor side (one pending reject), origin side (sticky
    // per proxy slot)
    logic                                                              reject_valid_q;
    remote_done_t                                                      reject_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    remote_rejected_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0]    remote_reject_in;

    ///////////////////////////////////////////
    // Stream Demux Checkout Queue Chiplet Set
    ///////////////////////////////////////////
    // After each checkout queue, we need to demux the chiplet dep set tasks
    // There are two types of outputs from the checkout queue
    // [0]: Local dep set
    // [1]: Chiplet dep set
    typedef logic [1:0] stream_demux_checkout_queue_chiplet_dep_set_oup_t;
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_checkout_queue_chiplet_dep_set_inp_valid;
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_checkout_queue_chiplet_dep_set_inp_ready;
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_checkout_queue_chiplet_dep_set_oup_sel;
    stream_demux_checkout_queue_chiplet_dep_set_oup_t [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_checkout_queue_chiplet_dep_set_oup_valid;
    stream_demux_checkout_queue_chiplet_dep_set_oup_t [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_demux_checkout_queue_chiplet_dep_set_oup_ready;

    ///////////////////////////////////////////
    // Stream Filter Checkout Queue Dep Set Enable
    ///////////////////////////////////////////    
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_filter_checkout_queue_dep_set_enable_inp_valid;
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_filter_checkout_queue_dep_set_enable_inp_ready;
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_filter_checkout_queue_dep_set_enable_drop;
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_filter_checkout_queue_dep_set_enable_oup_valid;
    logic                                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] stream_filter_checkout_queue_dep_set_enable_oup_ready;
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
    logic                             [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] done_q_full;
    // Legacy single-queue signals for AXI-Lite mailbox mode (TYPE==0)
    // In AXI-Lite mode, we still use a single mailbox + internal demux
    device_axi_lite_data_t               done_queue_mbox_data;
    logic                                done_queue_mbox_pop;
    logic                                done_queue_mbox_empty;
    bingo_hw_manager_done_info_full_t    cur_done_queue_info_axi;
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
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_busy;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_available;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_dead_suspect;
    // Confirmed dead (sticky): the slot's ready reads, dones and heartbeats are ignored
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_fenced;
    // Fenced and all its outstanding tasks moved to other cores (sticky)
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_retired;
    // Checkout queue head executes on a core (normal / gating): retires with its done
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] checkout_head_exec;

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
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)-1:0] replay_head_logical;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0][cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET)-1:0] replay_head_logical_cluster;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_head_no_exec;
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

    // Per logical core (waiting queue head): 1 if the task executes on a core
    // and may therefore be remapped (not a dummy-set, not CERF-skipped).
    logic [NUM_CORES_PER_CLUSTER-1:0] remap_remappable;
    logic [NUM_CORES_PER_CLUSTER-1:0] remap_select_valid_raw;
    logic [NUM_CORES_PER_CLUSTER-1:0] remap_select_valid;
    bingo_hw_manager_assigned_core_id_t    [NUM_CORES_PER_CLUSTER-1:0] remap_physical_core;
    bingo_hw_manager_assigned_cluster_id_t [NUM_CORES_PER_CLUSTER-1:0] remap_physical_cluster;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] remap_route_valid;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] remap_route_fire;
    bingo_hw_manager_assigned_core_id_t    [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] remap_route_src_core;

    // Number of tasks of logical core [core] (cluster [cluster]) that were remapped
    // to another physical core and are still in that core's checkout queue.
    localparam int unsigned RemapOutstandingWidth =
        $clog2(NUM_CORES_PER_CLUSTER * CheckoutQueueDepth + 1);
    logic [RemapOutstandingWidth-1:0] remap_outstanding_d [NUM_CORES_PER_CLUSTER][NUM_CLUSTERS_PER_CHIPLET];
    logic [RemapOutstandingWidth-1:0] remap_outstanding_q [NUM_CORES_PER_CLUSTER][NUM_CLUSTERS_PER_CHIPLET];

    // Watchdog counter wide enough for the configured timeouts (saturating timer).
    localparam int unsigned WatchdogMaxTimeoutCycles =
        (WatchdogConfirmTimeoutCycles > WatchdogHeartbeatTimeoutCycles) ?
        WatchdogConfirmTimeoutCycles : WatchdogHeartbeatTimeoutCycles;
    localparam int unsigned WatchdogCounterWidth = $clog2(WatchdogMaxTimeoutCycles + 1) + 1;
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
        assign reset_start_o = 1'b0;
        assign reset_start_enable_o = 1'b0;
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
            .task_queue_empty_o        (task_queue_mbox_empty                )
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
    assign cur_task_desc.task_id = cur_task_desc_full.task_id;
    assign cur_task_desc.task_type = cur_task_desc_full.task_type;
    assign cur_task_desc.assigned_chiplet_id = cur_task_desc_full.assigned_chiplet_id;
    assign cur_task_desc.assigned_cluster_id = cur_task_desc_full.assigned_cluster_id;
    assign cur_task_desc.assigned_core_id = cur_task_desc_full.assigned_core_id;
    assign cur_task_desc.dep_check_info = cur_task_desc_full.dep_check_info;
    assign cur_task_desc.dep_set_info = cur_task_desc_full.dep_set_info;
    // DARTS Tier 1: CERF fields
    assign cur_task_desc.cond_exec_en = cur_task_desc_full.cond_exec_en;
    assign cur_task_desc.cond_exec_group_id = cur_task_desc_full.cond_exec_group_id;
    assign cur_task_desc.cond_exec_invert = cur_task_desc_full.cond_exec_invert;


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
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].reserved_bits = '0;
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].dep_set_info = checkout_queue_data_out[core][cluster].dep_set_info;
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].dep_check_info = checkout_queue_data_out[core][cluster].dep_check_info;
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].assigned_core_id = checkout_queue_data_out[core][cluster].assigned_core_id;
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].assigned_cluster_id = checkout_queue_data_out[core][cluster].assigned_cluster_id;
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].assigned_chiplet_id = checkout_queue_data_out[core][cluster].assigned_chiplet_id;
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].task_id = checkout_queue_data_out[core][cluster].task_id;
                stream_arbiter_chiplet_dep_set_inp_task_desc[core + cluster * NUM_CORES_PER_CLUSTER].task_type = checkout_queue_data_out[core][cluster].task_type;
                stream_arbiter_chiplet_dep_set_inp_valid[core + cluster * NUM_CORES_PER_CLUSTER] = stream_demux_checkout_queue_chiplet_dep_set_oup_valid[core][cluster][1];
            end           
        end
    end


    //////////////////////////////////////////////////////////////////////
    // Chiplet from remote Done Queue
    //////////////////////////////////////////////////////////////////////
    bingo_hw_manager_write_mailbox #(
        .MailboxDepth(ChipletDoneQueueDepth                    ),
        .IrqEdgeTrig (1'b0                                     ),
        .IrqActHigh  (1'b1                                     ),
        .AxiAddrWidth(HostAxiLiteAddrWidth                     ),
        .AxiDataWidth(HostAxiLiteDataWidth                     ),
        .ChipIdWidth (ChipIdWidth                              ),
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
            .DEPTH        ( 8                                  ),
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
            .drop_i  ( (waiting_dep_check_task_desc[core].task_type == 2'b01) && (waiting_dep_check_task_desc[core].dep_check_info.dep_check_en) ), // Drop if it is a dummy check task
            .valid_o ( demux_ready_and_checkout_queue_inp_valid[core]  ),
            .ready_i ( demux_ready_and_checkout_queue_inp_ready[core]  )
        );
        stream_demux #(
            .N_OUP ( NUM_CLUSTERS_PER_CHIPLET           )
        ) i_stream_demux_from_waiting_dep_check_queue_to_ready_and_checkout_queue (
            .inp_valid_i ( demux_ready_and_checkout_queue_inp_valid[core]    ),
            .inp_ready_o ( demux_ready_and_checkout_queue_inp_ready[core]    ),
            // Physical cluster: the logical one unless the task goes to a substitute
            .oup_sel_i   ( remap_physical_cluster[core]                          ),
            .oup_valid_o ( demux_ready_and_checkout_queue_oup_valid[core]    ),
            .oup_ready_i ( demux_ready_and_checkout_queue_oup_ready[core]    )
        );

        always_comb begin : connect_demux_ready_and_checkout_queue_ready_signals
            for (int cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin
                demux_ready_and_checkout_queue_oup_ready[core][cluster] = 1'b0;

                if (remap_select_valid[core] &&
                    (remap_physical_cluster[core] == bingo_hw_manager_assigned_cluster_id_t'(cluster)) &&
                    (remap_route_src_core[remap_physical_core[core]][remap_physical_cluster[core]] ==
                        bingo_hw_manager_assigned_core_id_t'(core))) begin

                    demux_ready_and_checkout_queue_oup_ready[core][cluster] =
                        remap_route_fire[remap_physical_core[core]][remap_physical_cluster[core]];
                end
            end
        end
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
                    // Handshake from the checkout demux and the per-(core,cluster) done queue
                    // Dummy set: no done queue check needed
                    // Normal: per-(core,cluster) done queue must be non-empty
                    stream_arbiter_dep_matrix_set_inp_valid[stream_arbiter_inp_idx] = (checkout_queue_data_out[core][cluster].task_type == 2'b01) ?
                                                                                      stream_filter_checkout_queue_dep_set_enable_oup_valid[core][cluster] :
                                                                                      ((stream_filter_checkout_queue_dep_set_enable_oup_valid[core][cluster]) &&
                                                                                       (!done_q_empty[core][cluster]));
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
    // This is the ready queue interface
    // Device will read ready tasks info from this queue via 32bit AXI Lite
    // The information contains only task ID
    // Before each ready queue, there is a filter to filter out the dummy set tasks since it will not be run on the core
    for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin: gen_ready_queue_per_core
        for (genvar cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin: gen_ready_queue_per_core_per_cluster
            stream_filter i_stream_filter_for_ready_queue_dummy_set (
                .valid_i (   ready_queue_filter_inp_valid[core][cluster]       ),
                .ready_o (   ready_queue_filter_inp_ready[core][cluster]       ),
                .drop_i  (   ready_queue_filter_drop[core][cluster]            ),
                .valid_o (   ready_queue_filter_oup_valid[core][cluster]       ),
                .ready_i (   ready_queue_filter_oup_ready[core][cluster]       )
            );
            assign ready_queue_filter_inp_valid[core][cluster] =
                remap_route_valid[core][cluster] && !checkout_queue_full[core][cluster];
            // Level 3: an exported task only enters the proxy's checkout queue
            // (with its export) and never its ready queue
            assign route_remote[core][cluster] = RemoteEn && remap_route_valid[core][cluster] &&
                                                 remap_remote[remap_route_src_core[core][cluster]];
            assign remap_route_fire[core][cluster] =
                remap_route_valid[core][cluster] &&
                !checkout_queue_full[core][cluster] &&
                ready_queue_filter_inp_ready[core][cluster] &&
                (!route_remote[core][cluster] || route_remote_grant[core][cluster]);
            // Drop the dummy set tasks
            // Drop from ready queue if:
            // 1. Dummy set task (task_type==01, dep_set_en==1) — existing behavior
            // 2. DARTS CERF: conditionally skipped task — skip execution but propagate deps
            assign ready_queue_filter_drop[core][cluster] =
                remap_route_valid[core][cluster] &&
                (((waiting_dep_check_task_desc[remap_route_src_core[core][cluster]].task_type == 2'b01) &&
                  (waiting_dep_check_task_desc[remap_route_src_core[core][cluster]].dep_set_info.dep_set_en == 1'b1)) ||
                 cond_exec_skip[remap_route_src_core[core][cluster]] ||
                 route_remote[core][cluster]);
            assign ready_queue_filter_oup_ready[core][cluster] = ~ready_queue_full[core][cluster];
            if (READY_AND_DONE_QUEUE_INTERFACE_TYPE==0) begin: gen_ready_queue_axi_lite_mailbox                               
                bingo_hw_manager_read_mailbox #(
                    .MailboxDepth(ReadyQueueDepth                ),
                    .IrqEdgeTrig (1'b0                           ),
                    .IrqActHigh  (1'b1                           ),
                    .AxiAddrWidth(DeviceAxiLiteAddrWidth         ),
                    .AxiDataWidth(DeviceAxiLiteDataWidth         ),
                    .ChipIdWidth (ChipIdWidth                    ),
                    .req_lite_t  (device_axi_lite_req_t          ),
                    .resp_lite_t (device_axi_lite_resp_t         )
                ) i_bingo_hw_manager_ready_queue (
                    .clk_i       (clk_i                                                        ),
                    .rst_ni      (rst_ni                                                       ),
                    .chip_id_i   (chip_id_i                                                    ),
                    .test_i      (1'b0                                                         ),
                    .req_i       (ready_queue_axi_lite_req_i[core][cluster]                    ),
                    .resp_o      (ready_queue_axi_lite_resp_o[core][cluster]                   ),
                    .irq_o       (/*not used*/                                                 ),
                    .base_addr_i (ready_queue_base_addr[core][cluster]                         ),
                    .mbox_data_i (ready_queue_data_in[core][cluster]                           ),
                    .mbox_push_i (ready_queue_push[core][cluster]                              ),
                    .mbox_full_o (ready_queue_full[core][cluster]                              ),
                    .mbox_flush_i(1'b0                                                         )
                );
                // Connect to the core_status_waiting_task
                // This signal indicates whether the core is waiting for a task to be read from the ready queue
                // If ar_valid is high and r_ready is low, it means the core is waiting for a task
                assign core_status_waiting_task[core][cluster] = ready_queue_axi_lite_req_i[core][cluster].ar_valid && 
                                                                !ready_queue_axi_lite_req_i[core][cluster].r_ready;
                // Tie off the generic fifo read signals
                assign ready_queue_pop[core][cluster] = 1'b0;
                assign ready_queue_empty[core][cluster] = 1'b0;
                assign ready_queue_data_out[core][cluster] = '0;
            end else begin: gen_ready_queue_generic_fifo
                fifo_v3 #(
                    .FALL_THROUGH ( 1'b0                                      ),
                    .DEPTH        ( ReadyQueueDepth                           ),
                    .dtype        ( bingo_hw_manager_ready_task_desc_full_t   )
                ) i_ready_queue (
                    .clk_i       ( clk_i                                  ),
                    .rst_ni      ( rst_ni                                 ),
                    .testmode_i  ( 1'b0                                   ),
                    // Replay: the entries of a fenced core are moved from its checkout queue
                    .flush_i     ( replay_ready_flush[core][cluster]      ),
                    .full_o      ( ready_queue_full[core][cluster]        ),
                    .empty_o     ( ready_queue_empty[core][cluster]       ),
                    .usage_o     ( /*not used*/                           ),
                    .data_i      ( ready_queue_data_in[core][cluster]     ),
                    .push_i      ( ready_queue_push[core][cluster]        ),
                    .data_o      ( ready_queue_data_out[core][cluster]    ),
                    .pop_i       ( ready_queue_pop[core][cluster]         )
                );
                // Connect to the core_status_waiting_task
                // Since we do not have the axi lite interface, we tie off the ready queue axi lite resp signals
                assign ready_queue_axi_lite_resp_o[core][cluster] = '0;
            end
            assign ready_queue_base_addr[core][cluster] = ready_queue_base_addr_i +
                                                        (core + cluster * NUM_CORES_PER_CLUSTER) * ReadyQueueAddrOffset;
            // Replay pushes never collide with routed pushes: routing into the cluster
            // is blocked while a replay MOVE may push into it.
            assign ready_queue_data_in[core][cluster].task_id = replay_push_ready[core][cluster] ?
                replay_data.task_id :
                import_push[core][cluster] ? import_desc.task_id :
                waiting_dep_check_task_desc[remap_route_src_core[core][cluster]].task_id;
            assign ready_queue_data_in[core][cluster].reserved_bits = '0;
            assign ready_queue_push[core][cluster] = (ready_queue_filter_oup_valid[core][cluster] & ~ready_queue_full[core][cluster]) |
                                                     replay_push_ready[core][cluster] | import_push[core][cluster];
        end
    end


    //////////////////////////////////////////////////////////////////////
    // Checkout Queue
    //////////////////////////////////////////////////////////////////////
    // Check out queues are internal fifos
    // input is from the waiting dep check queue
    // after it has been checked by the dep matrix, it will be pushed to the checkout queue
    // and then wait the done queue to pop it
    for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin: gen_checkout_queue_per_core
        for (genvar cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin: gen_checkout_queue_per_core_per_cluster
            fifo_v3 #(
                .FALL_THROUGH ( 1'b0                                  ),
                .DEPTH        ( CheckoutFifoDepth                     ),
                .dtype        ( bingo_hw_manager_task_desc_t          )
            ) i_checkout_queue (
                .clk_i       ( clk_i                                  ),
                .rst_ni      ( rst_ni                                 ),
                .testmode_i  ( 1'b0                                   ),
                .flush_i     ( 1'b0                                   ),
                .full_o      ( checkout_queue_full_raw[core][cluster] ),
                .empty_o     ( checkout_queue_empty[core][cluster]    ),
                .usage_o     ( checkout_queue_usage[core][cluster]    ),
                .data_i      ( checkout_queue_data_in[core][cluster]  ),
                .push_i      ( checkout_queue_push[core][cluster]     ),
                .data_o      ( checkout_queue_data_out[core][cluster] ),
                .pop_i       ( checkout_queue_pop[core][cluster]      )
            );
            // Level 3: the spare entry is only for replay rotations
            if (RemoteEn) begin : gen_checkout_remote
                assign checkout_queue_full[core][cluster] = checkout_queue_full_raw[core][cluster] ||
                    (checkout_queue_usage[core][cluster] == CheckoutUsageWidth'(CheckoutQueueDepth));
                fifo_v3 #(
                    .FALL_THROUGH ( 1'b0              ),
                    .DEPTH        ( CheckoutFifoDepth ),
                    .dtype        ( remote_tag_t      )
                ) i_checkout_remote_tag (
                    .clk_i       ( clk_i                                  ),
                    .rst_ni      ( rst_ni                                 ),
                    .testmode_i  ( 1'b0                                   ),
                    .flush_i     ( 1'b0                                   ),
                    .full_o      ( /* same as the checkout queue */       ),
                    .empty_o     ( /* same as the checkout queue */       ),
                    .usage_o     ( /* same as the checkout queue */       ),
                    .data_i      ( checkout_remote_tag_in[core][cluster]  ),
                    .push_i      ( checkout_queue_push[core][cluster]     ),
                    .data_o      ( checkout_remote_tag_out[core][cluster] ),
                    .pop_i       ( checkout_queue_pop[core][cluster]      )
                );
                always_comb begin
                    checkout_remote_tag_in[core][cluster] = '0;
                    if (replay_push[core][cluster]) begin
                        checkout_remote_tag_in[core][cluster] =
                            checkout_remote_tag_out[replay_src_core][replay_src_cluster];
                        if (replay_rotate) checkout_remote_tag_in[core][cluster].exported = 1'b1;
                    end else if (import_push[core][cluster]) begin
                        checkout_remote_tag_in[core][cluster].imported    = 1'b1;
                        checkout_remote_tag_in[core][cluster].origin_chip = remote_dispatch_origin_chip_i;
                        checkout_remote_tag_in[core][cluster].proxy_slot  = remote_dispatch_proxy_slot_i;
                    end else begin
                        checkout_remote_tag_in[core][cluster].exported = route_remote[core][cluster];
                    end
                end
            end else begin : gen_checkout_no_remote
                assign checkout_queue_full[core][cluster]     = checkout_queue_full_raw[core][cluster];
                assign checkout_remote_tag_in[core][cluster]  = '0;
                assign checkout_remote_tag_out[core][cluster] = '0;
            end
            assign checkout_head_imported[core][cluster] = RemoteEn && checkout_remote_tag_out[core][cluster].imported;

            // DARTS CERF: if task is conditionally skipped, mark as dummy (2'b01)
            // so checkout logic fires dep_set without done_queue match
            always_comb begin
                if (replay_push[core][cluster]) begin
                    // Replayed entry: already checked (and CERF-marked) at its first dispatch
                    checkout_queue_data_in[core][cluster] = replay_data;
                end else if (import_push[core][cluster]) begin
                    // Imported entry: checked on its origin chiplet
                    checkout_queue_data_in[core][cluster] = import_desc;
                end else begin
                    checkout_queue_data_in[core][cluster] =
                        waiting_dep_check_task_desc[remap_route_src_core[core][cluster]];
                    if (cond_exec_skip[remap_route_src_core[core][cluster]]) begin
                        checkout_queue_data_in[core][cluster].task_type = 2'b01;
                    end
                end
            end
            assign checkout_queue_push[core][cluster] = remap_route_fire[core][cluster] | replay_push[core][cluster] |
                                                        import_push[core][cluster];
            // Pop on the handshake only: the downstream arbiters may raise ready without
            // valid, which would retire an executing head before its done arrived.
            // While a replay MOVE drains this queue, only the replay controller pops it
            // (valid is held low then). A stuck slot keeps its remaining entries: a
            // dummy-set among them must not fire before its lost source task.
            // An imported head retires into the remote done stream instead.
            assign checkout_queue_pop[core][cluster] =
                (stream_demux_checkout_queue_chiplet_dep_set_inp_valid[core][cluster] &&
                 stream_demux_checkout_queue_chiplet_dep_set_inp_ready[core][cluster]) ||
                (remote_done_arb_valid[core + cluster * NUM_CORES_PER_CLUSTER] &&
                 remote_done_arb_ready[core + cluster * NUM_CORES_PER_CLUSTER]) ||
                replay_pop[core][cluster];

            stream_demux #(
                .N_OUP ( 2 )
            ) i_stream_demux_checkout_queue_chiplet_dep_set (
                .inp_valid_i ( stream_demux_checkout_queue_chiplet_dep_set_inp_valid[core][cluster]    ),
                .inp_ready_o ( stream_demux_checkout_queue_chiplet_dep_set_inp_ready[core][cluster]    ),
                .oup_sel_i   ( stream_demux_checkout_queue_chiplet_dep_set_oup_sel[core][cluster]      ),
                .oup_valid_o ( stream_demux_checkout_queue_chiplet_dep_set_oup_valid[core][cluster]    ),
                .oup_ready_i ( stream_demux_checkout_queue_chiplet_dep_set_oup_ready[core][cluster]    )
            );

            // An executing task (normal / gating) only leaves the checkout queue with its
            // own done, for the local and the chiplet dep_set path alike.
            assign checkout_head_exec[core][cluster] = (checkout_queue_data_out[core][cluster].task_type == 2'b00) ||
                                                       (checkout_queue_data_out[core][cluster].task_type == 2'b10);
            // Level 3: an exported head only retires on the remote done of the
            // same task (the dones of a proxy slot come back in export order)
            assign remote_head_mismatch[core][cluster] = RemoteEn &&
                checkout_remote_tag_out[core][cluster].exported &&
                !checkout_queue_empty[core][cluster] && checkout_head_exec[core][cluster] &&
                !done_q_empty[core][cluster] &&
                (done_q_info[core][cluster].task_id != checkout_queue_data_out[core][cluster].task_id);
            assign checkout_retire_valid[core][cluster] = !checkout_queue_empty[core][cluster] &&
                                                          !replay_hold_slot[core][cluster] &&
                                                          !replay_stuck_slot[core][cluster] &&
                                                          !remote_head_mismatch[core][cluster] &&
                                                          !remote_rejected_q[core][cluster] &&
                                                          (!checkout_head_exec[core][cluster] ||
                                                           !done_q_empty[core][cluster]);
            assign stream_demux_checkout_queue_chiplet_dep_set_inp_valid[core][cluster] =
                checkout_retire_valid[core][cluster] && !checkout_head_imported[core][cluster];
            assign remote_done_arb_valid[core + cluster * NUM_CORES_PER_CLUSTER] =
                checkout_retire_valid[core][cluster] && checkout_head_imported[core][cluster];
            assign remote_done_arb_data[core + cluster * NUM_CORES_PER_CLUSTER] = '{
                chip:       checkout_remote_tag_out[core][cluster].origin_chip,
                proxy_slot: checkout_remote_tag_out[core][cluster].proxy_slot,
                task_id:    checkout_queue_data_out[core][cluster].task_id,
                reject:     1'b0
            };
            assign stream_demux_checkout_queue_chiplet_dep_set_oup_sel[core][cluster] = 
                (checkout_queue_data_out[core][cluster].dep_set_info.dep_set_chiplet_id != chip_id_i);
            // To Chiplet Dep Set
            assign stream_demux_checkout_queue_chiplet_dep_set_oup_ready[core][cluster][1] = stream_arbiter_chiplet_dep_set_inp_ready[core + cluster * NUM_CORES_PER_CLUSTER];
            // To Local Dep Set
            assign stream_demux_checkout_queue_chiplet_dep_set_oup_ready[core][cluster][0] = stream_filter_checkout_queue_dep_set_enable_inp_ready[core][cluster];

            stream_filter i_stream_filter_checkout_queue_dep_set_enable (
                .valid_i ( stream_filter_checkout_queue_dep_set_enable_inp_valid[core][cluster]    ),
                .ready_o ( stream_filter_checkout_queue_dep_set_enable_inp_ready[core][cluster]    ),
                .drop_i  ( stream_filter_checkout_queue_dep_set_enable_drop[core][cluster]         ),
                .valid_o ( stream_filter_checkout_queue_dep_set_enable_oup_valid[core][cluster]    ),
                .ready_i ( stream_filter_checkout_queue_dep_set_enable_oup_ready[core][cluster]    )
            );
            assign stream_filter_checkout_queue_dep_set_enable_inp_valid[core][cluster] = stream_demux_checkout_queue_chiplet_dep_set_oup_valid[core][cluster][0];
            // Only drop the signal when dep set is disabled and the per-(core,cluster) done queue is non-empty
            assign stream_filter_checkout_queue_dep_set_enable_drop[core][cluster] =
                (checkout_queue_data_out[core][cluster].dep_set_info.dep_set_en == 1'b0) &&
                (!done_q_empty[core][cluster]);
            assign stream_filter_checkout_queue_dep_set_enable_oup_ready[core][cluster] = stream_arbiter_dep_matrix_set_inp_ready[core + cluster * NUM_CORES_PER_CLUSTER];

        end
    end

    //////////////////////////////////////////////////////////////////////
    // Local Per-Core Done Queues
    //////////////////////////////////////////////////////////////////////
    // Each core has its own done queue FIFO. This eliminates HOL blocking
    // where one core's completion stalls behind another core's entry in a
    // shared FIFO. Completions for different cores drain independently.

    if (READY_AND_DONE_QUEUE_INTERFACE_TYPE==0) begin: gen_done_queue_axi_lite_mailbox
        // AXI-Lite mailbox mode: single mailbox writes into a shared FIFO,
        // then we demux to per-(core,cluster) FIFOs based on done_info fields.
        bingo_hw_manager_write_mailbox #(
            .MailboxDepth(DoneQueueDepth               ),
            .IrqEdgeTrig (1'b0                         ),
            .IrqActHigh  (1'b1                         ),
            .AxiAddrWidth(DeviceAxiLiteAddrWidth       ),
            .AxiDataWidth(DeviceAxiLiteDataWidth       ),
            .ChipIdWidth (ChipIdWidth                  ),
            .req_lite_t  (device_axi_lite_req_t        ),
            .resp_lite_t (device_axi_lite_resp_t       )
        ) i_bingo_hw_manager_done_queue (
            .clk_i       (clk_i                     ),
            .rst_ni      (rst_ni                    ),
            .chip_id_i   (chip_id_i                 ),
            .test_i      (1'b0                      ),
            .req_i       (done_queue_axi_lite_req_i ),
            .resp_o      (done_queue_axi_lite_resp_o),
            .irq_o       (),
            .base_addr_i (done_queue_base_addr_i    ),
            .mbox_data_o (done_queue_mbox_data      ),
            .mbox_pop_i  (done_queue_mbox_pop       ),
            .mbox_empty_o(done_queue_mbox_empty     ),
            .mbox_flush_i(1'b0)
        );
        assign cur_done_queue_info_axi = bingo_hw_manager_done_info_full_t'(done_queue_mbox_data);
        // Pop the mailbox when the target per-(core,cluster) FIFO accepts it
        assign done_queue_mbox_pop = !done_queue_mbox_empty &&
                                     !done_q_full[cur_done_queue_info_axi.assigned_core_id][cur_done_queue_info_axi.assigned_cluster_id];
        // Route mailbox data to per-(core,cluster) FIFOs
        always_comb begin
            for (int c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                for (int cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                    done_q_data_in[c][cl] = cur_done_queue_info_axi;
                    done_q_push[c][cl] = done_queue_mbox_pop &&
                        (cur_done_queue_info_axi.assigned_core_id == bingo_hw_manager_assigned_core_id_t'(c)) &&
                        (cur_done_queue_info_axi.assigned_cluster_id == bingo_hw_manager_assigned_cluster_id_t'(cl));
                end
            end
        end
    end else begin: gen_done_queue_generic_fifo
        // Generic FIFO mode: CSR writes go through arbiter, then demux to per-(core,cluster) FIFOs.
        assign done_queue_axi_lite_resp_o = '0;
        assign done_queue_mbox_empty = 1'b1;
        assign done_queue_mbox_data = '0;
        assign done_queue_mbox_pop = 1'b0;
    end

    // Per-(core, cluster) done queue FIFO instantiation
    for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core++) begin: gen_done_q_core
        for (genvar cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster++) begin: gen_done_q_cluster
            fifo_v3 #(
                .FALL_THROUGH ( 1'b0                               ),
                .DEPTH        ( DoneQueueDepth                     ),
                .dtype        ( bingo_hw_manager_done_info_full_t  )
            ) i_done_q (
                .clk_i       ( clk_i                            ),
                .rst_ni      ( rst_ni                           ),
                .testmode_i  ( 1'b0                             ),
                .flush_i     ( 1'b0                             ),
                .full_o      ( done_q_full[core][cluster]       ),
                .empty_o     ( done_q_empty[core][cluster]      ),
                .usage_o     ( /*not used*/                     ),
                .data_i      ( done_q_data_in[core][cluster]    ),
                .push_i      ( done_q_push[core][cluster]       ),
                .data_o      ( done_q_info[core][cluster]       ),
                .pop_i       ( done_q_pop[core][cluster]        )
            );
        end
    end

    // Per-(core, cluster) done queue pop logic:
    // Pop together with the checkout queue head when that head is a normal (2'b00)
    // or gating (2'b10) task: it only leaves the checkout queue with its done
    // (local dep_set, chiplet dep_set, or dropped when dep_set is disabled).
    // Popping on the dep-matrix arbiter's ready instead left the done behind when
    // the head took another path, so the next task retired on it prematurely.
    // A replay move is not a retirement. No cross-core or cross-cluster blocking.
    always_comb begin
        for (int core = 0; core < NUM_CORES_PER_CLUSTER; core++) begin
            for (int cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster++) begin
                done_q_pop[core][cluster] = checkout_queue_pop[core][cluster] &&
                                            !replay_pop[core][cluster] &&
                                            checkout_head_exec[core][cluster];
            end
        end
    end

    // For generic FIFO done queue, we need to connect the CSR interface signals
    if (READY_AND_DONE_QUEUE_INTERFACE_TYPE==1) begin: gen_csr_to_fifo_intf
        localparam N_CORES_TOTAL = NUM_CLUSTERS_PER_CHIPLET * NUM_CORES_PER_CLUSTER;
        // 1D CSR Requests
        csr_req_t [N_CORES_TOTAL-1:0] csr_req_1d;
        logic     [N_CORES_TOTAL-1:0] csr_req_valid_1d;
        logic     [N_CORES_TOTAL-1:0] csr_req_ready_1d;
        csr_rsp_t [N_CORES_TOTAL-1:0] csr_rsp_1d;
        logic     [N_CORES_TOTAL-1:0] csr_rsp_valid_1d;
        logic     [N_CORES_TOTAL-1:0] csr_rsp_ready_1d;
        // 1D Ready Queue FIFO Interface
        device_axi_lite_data_t [N_CORES_TOTAL-1:0] read_ready_queue_data_1d;
        logic                  [N_CORES_TOTAL-1:0] read_ready_queue_valid_1d;
        logic                  [N_CORES_TOTAL-1:0] read_ready_queue_ready_1d;
        // 1D Done QUeue FIFO Interface
        device_axi_lite_data_t [N_CORES_TOTAL-1:0] write_done_queue_data_1d;
        logic                  [N_CORES_TOTAL-1:0] write_done_queue_valid_1d;
        logic                  [N_CORES_TOTAL-1:0] write_done_queue_ready_1d;
        device_axi_lite_data_t write_done_queue_data;
        logic                  write_done_queue_valid;
        logic                  write_done_queue_ready;
        //
        logic                  [N_CORES_TOTAL-1:0] heartbeat_valid_1d;
        device_axi_lite_data_t [N_CORES_TOTAL-1:0] heartbeat_data_1d;
        logic                  [N_CORES_TOTAL-1:0] csr_req_unknown_1d;
        // Done writes of fenced cores are accepted and dropped before the arbiter
        logic                  [N_CORES_TOTAL-1:0] core_fenced_1d;
        logic                  [N_CORES_TOTAL-1:0] write_done_arb_valid_1d;
        logic                  [N_CORES_TOTAL-1:0] write_done_arb_ready_1d;

        bingo_hw_manager_csr_to_fifo #(
            .TaskIdWidth (TaskIdWidth),
            .N (N_CORES_TOTAL),
            .NUM_CORES_PER_CLUSTER (NUM_CORES_PER_CLUSTER),
            .NUM_CLUSTERS_PER_CHIPLET (NUM_CLUSTERS_PER_CHIPLET),
            .csr_req_t (csr_req_t),
            .csr_rsp_t (csr_rsp_t),
            .data_t    (device_axi_lite_data_t),
            .bingo_hw_manager_done_info_full_t (bingo_hw_manager_done_info_full_t),
            .CsrHeartbeatAddr (CsrHeartbeatAddr)
        ) i_bingo_hw_manager_csr_to_fifo (
            .csr_req_i         (csr_req_1d               ),
            .csr_req_valid_i   (csr_req_valid_1d         ),
            .csr_req_ready_o   (csr_req_ready_1d         ),
            .csr_rsp_o         (csr_rsp_1d               ),
            .csr_rsp_valid_o   (csr_rsp_valid_1d         ),
            .csr_rsp_ready_i   (csr_rsp_ready_1d         ),
            // FIFO Read Interface
            .fifo_data_i       (read_ready_queue_data_1d ),
            .fifo_data_valid_i (read_ready_queue_valid_1d),
            .fifo_data_ready_o (read_ready_queue_ready_1d),
            // FIFO Write Interface
            .fifo_data_o       (write_done_queue_data_1d ),
            .fifo_data_valid_o (write_done_queue_valid_1d),
            .fifo_data_ready_i (write_done_queue_ready_1d),
            // heartbeat signal
            .heartbeat_valid_o  ( heartbeat_valid_1d     ),
            .heartbeat_data_o   ( heartbeat_data_1d      ), // heartbeat_data_1d is reserved for future progress counters.
            .csr_req_unknown_o  ( csr_req_unknown_1d     )
            );

`ifndef SYNTHESIS
        // A CSR request with an unknown address is never served and silently stalls
        // its core (e.g. an integration that forwards a translated CSR number).
        // Report it once when it appears.
        logic [N_CORES_TOTAL-1:0] csr_req_unknown_q;
        always @(posedge clk_i or negedge rst_ni) begin
            if (!rst_ni) begin
                csr_req_unknown_q <= '0;
            end else begin
                csr_req_unknown_q <= csr_req_unknown_1d;
                for (int unsigned i = 0; i < N_CORES_TOTAL; i++) begin
                    if (csr_req_unknown_1d[i] && !csr_req_unknown_q[i]) begin
                        $error("[BINGO_CSR] chip %0d core %0d cluster %0d: unknown CSR address 0x%0h (write=%0b), request will stall",
                               chip_id_i, i % NUM_CORES_PER_CLUSTER, i / NUM_CORES_PER_CLUSTER,
                               csr_req_1d[i].addr, csr_req_1d[i].write);
                    end
                end
            end
        end
`endif

        always_comb begin : connect_ready_queue_1d_to_2d
            for (int unsigned core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin
                for (int unsigned cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin
                    csr_req_1d[core + cluster * NUM_CORES_PER_CLUSTER] = csr_req_i[core][cluster];
                    csr_req_valid_1d[core + cluster * NUM_CORES_PER_CLUSTER] = csr_req_valid_i[core][cluster];
                    csr_req_ready_o[core][cluster] = csr_req_ready_1d[core + cluster * NUM_CORES_PER_CLUSTER];
                    csr_rsp_o[core][cluster] = csr_rsp_1d[core + cluster * NUM_CORES_PER_CLUSTER];
                    csr_rsp_valid_o[core][cluster] = csr_rsp_valid_1d[core + cluster * NUM_CORES_PER_CLUSTER];
                    csr_rsp_ready_1d[core + cluster * NUM_CORES_PER_CLUSTER] = csr_rsp_ready_i[core][cluster];
                    // A fenced core can no longer take tasks (its ready reads stall) or
                    // report heartbeats: its outstanding tasks are replayed elsewhere.
                    read_ready_queue_data_1d[core + cluster * NUM_CORES_PER_CLUSTER] = device_axi_lite_data_t'(ready_queue_data_out[core][cluster]);
                    read_ready_queue_valid_1d[core + cluster * NUM_CORES_PER_CLUSTER] = !ready_queue_empty[core][cluster] &&
                                                                                        !core_fenced[core][cluster];
                    ready_queue_pop[core][cluster] = read_ready_queue_ready_1d[core + cluster * NUM_CORES_PER_CLUSTER] &&
                                                     !ready_queue_empty[core][cluster] && !core_fenced[core][cluster];
                    heartbeat_valid[core][cluster] = heartbeat_valid_1d[core + cluster * NUM_CORES_PER_CLUSTER] &&
                                                     !core_fenced[core][cluster];
                    core_fenced_1d[core + cluster * NUM_CORES_PER_CLUSTER] = core_fenced[core][cluster];
                end
            end
        end
        // Connect to the core_status_waiting_task
        // This signal indicates whether the core is waiting for a task to be read from the ready queue
        // If csr_req_i.write==0 and csr_req_valid_i is high and csr_req_ready_o is low, it means the core is waiting for a task
        always_comb begin : connect_core_status_waiting_task_signals
            for ( int core = 0; core < NUM_CORES_PER_CLUSTER; core = core + 1) begin
                for ( int cluster = 0; cluster < NUM_CLUSTERS_PER_CHIPLET; cluster = cluster + 1) begin
                    core_status_waiting_task[core][cluster] = (csr_req_i[core][cluster].write == 1'b0) &&
                                                              csr_req_valid_i[core][cluster] &&
                                                              !csr_req_ready_o[core][cluster];
                end
            end
        end

        // For the Done Queue, we arbitrate all cores' write requests, then demux
        // the result to per-core FIFOs based on assigned_core_id in the data.
        stream_arbiter #(
            .DATA_T(device_axi_lite_data_t),
            .N_INP (N_CORES_TOTAL)
        ) i_stream_arbiter_done_queue_write (
            .clk_i      (clk_i),
            .rst_ni     (rst_ni),
            .inp_data_i (write_done_queue_data_1d),
            .inp_valid_i(write_done_arb_valid_1d),
            .inp_ready_o(write_done_arb_ready_1d),
            .oup_data_o (write_done_queue_data),
            .oup_valid_o(write_done_queue_valid),
            .oup_ready_i(write_done_queue_ready)
        );
        // A fenced core's task is replayed on another core, so its own (late) done
        // must never retire the task a second time: accept the write and drop it.
        assign write_done_arb_valid_1d  = write_done_queue_valid_1d & ~core_fenced_1d;
        assign write_done_queue_ready_1d = write_done_arb_ready_1d | core_fenced_1d;

`ifndef SYNTHESIS
        always @(posedge clk_i) begin
            if (rst_ni) begin
                for (int unsigned i = 0; i < N_CORES_TOTAL; i++) begin
                    if (write_done_queue_valid_1d[i] && core_fenced_1d[i]) begin
                        $display("[BINGO_FENCE] %0t chip=%0d core=%0d cluster=%0d dropped done task=%0d",
                                 $time, chip_id_i, i % NUM_CORES_PER_CLUSTER, i / NUM_CORES_PER_CLUSTER,
                                 write_done_queue_data_1d[i][TaskIdWidth-1:0]);
                    end
                end
            end
        end
`endif

        // Extract core_id + cluster_id from the arbitrated done_info to route to per-(core,cluster) FIFO
        bingo_hw_manager_done_info_full_t write_done_info;
        assign write_done_info = bingo_hw_manager_done_info_full_t'(write_done_queue_data);
        // Route to per-(core, cluster) done queue FIFOs
        // Level 3: the done of an exported task enters the done FIFO of its
        // proxy slot (fenced, so it gets no CSR done); a CSR done to the same
        // FIFO in the same cycle wins.
        logic csr_done_to_remote_slot;
        always_comb begin
            remote_done_push        = '0;
            remote_reject_in        = '0;
            remote_done_ready_o     = 1'b0;
            csr_done_to_remote_slot = write_done_queue_valid &&
                (write_done_info.assigned_core_id ==
                 bingo_hw_manager_assigned_core_id_t'(remote_done_proxy_slot_i % NUM_CORES_PER_CLUSTER)) &&
                (write_done_info.assigned_cluster_id ==
                 bingo_hw_manager_assigned_cluster_id_t'(remote_done_proxy_slot_i / NUM_CORES_PER_CLUSTER));
            for (int c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                for (int cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                    done_q_data_in[c][cl] = write_done_info;
                    done_q_push[c][cl] = write_done_queue_valid &&
                        (write_done_info.assigned_core_id == bingo_hw_manager_assigned_core_id_t'(c)) &&
                        (write_done_info.assigned_cluster_id == bingo_hw_manager_assigned_cluster_id_t'(cl)) &&
                        !done_q_full[c][cl];
                    if (RemoteEn && remote_done_valid_i && remote_done_reject_i &&
                        (int'(remote_done_proxy_slot_i) == c + cl * NUM_CORES_PER_CLUSTER)) begin
                        // A reject marks the slot once the dones received before
                        // it have retired their entries
                        if (done_q_empty[c][cl]) begin
                            remote_reject_in[c][cl] = 1'b1;
                            remote_done_ready_o     = 1'b1;
                        end
                    end else if (RemoteEn && remote_done_valid_i && !csr_done_to_remote_slot &&
                        (int'(remote_done_proxy_slot_i) == c + cl * NUM_CORES_PER_CLUSTER) &&
                        !done_q_full[c][cl]) begin
                        remote_done_push[c][cl]             = 1'b1;
                        remote_done_ready_o                 = 1'b1;
                        done_q_push[c][cl]                  = 1'b1;
                        done_q_data_in[c][cl]               = '0;
                        done_q_data_in[c][cl].assigned_core_id    = bingo_hw_manager_assigned_core_id_t'(c);
                        done_q_data_in[c][cl].assigned_cluster_id = bingo_hw_manager_assigned_cluster_id_t'(cl);
                        done_q_data_in[c][cl].task_id             = remote_done_task_id_i;
                    end
                end
            end
        end
        assign write_done_queue_ready = !done_q_full[write_done_info.assigned_core_id][write_done_info.assigned_cluster_id];


    end else begin: gen_no_csr_to_fifo_intf
        // If it is AXI Lite Mailbox interface, the ready queue and done queue interface are already connected
        // So we do not need to do anything here
        // Level 3 needs the CSR interface (see gen_remote_intf_check)
        assign remote_done_push    = '0;
        assign remote_reject_in    = '0;
        assign remote_done_ready_o = 1'b0;
        // Tie the csr signals to zero
        assign csr_req_ready_o = '0;
        assign csr_rsp_o = '0;
        assign csr_rsp_valid_o = '0;
        assign heartbeat_valid = '0;
    end

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
        .pm_base_addr_i        ( bingo_hw_manager_pm_base_addr_i        ),
        .core_power_domain_i   ( bingo_hw_manager_core_power_domain_i   ),
        // Internal Core status
        .core_status_waiting_task_i ( core_status_waiting_task         ),
        // DVFS mode: monitor + notify host
        .pm_mode_i             ( bingo_hw_manager_pm_mode_i             ),
        .dvfs_clint_msip_addr_i( bingo_hw_manager_dvfs_clint_msip_addr_i),
        .dvfs_ack_i            ( bingo_hw_manager_dvfs_ack_i            ),
        .dvfs_request_o        ( bingo_hw_manager_dvfs_request_o        ),
        // Interface to Host AXI Lite
        .pm_axi_lite_req_o     (pm_axi_lite_req_o                      ),
        .pm_axi_lite_resp_i    (pm_axi_lite_resp_i                     )
    );
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
        .CoreMask(WatchdogCoreMask)
    ) i_watchdog (
        .clk_i                 ( clk_i                          ),
        .rst_ni                ( rst_ni                         ),
        .task_dispatched_i     ( ready_queue_pop                 ), // ready pop: busy, timer cleared
        .task_done_i           ( done_q_push                     ), // done reached the done FIFO: idle
        .heartbeat_i           ( heartbeat_valid                 ), // clears the timer (ignored once fenced)
        .waiting_task_i        ( core_status_waiting_task        ), // only feeds core_available
        .core_busy_o           ( core_busy                       ),
        .core_available_o      ( core_available                  ),
        .core_dead_suspect_o   ( core_dead_suspect               ),
        .core_fenced_o         ( core_fenced                     )
    );
    assign core_fenced_o = core_fenced;
    assign core_dead_suspect_o = core_dead_suspect;

    //////////////////////////////////////////////////////////////////////
    // Task Replay
    //////////////////////////////////////////////////////////////////////
    // A fenced core's checkout queue holds all its dispatched, unfinished tasks
    // in order (the head is the one it was running). The replay controller
    // moves them to live cores; see bingo_hw_manager_replay_ctrl.
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_head_exported;
    always_comb begin : compose_replay_inputs
        for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
            for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                replay_head_exported[c][cl]        = checkout_remote_tag_out[c][cl].exported;
                replay_head_logical[c][cl]         = checkout_queue_data_out[c][cl].assigned_core_id;
                replay_head_logical_cluster[c][cl] = checkout_queue_data_out[c][cl].assigned_cluster_id;
                replay_head_no_exec[c][cl] = (checkout_queue_data_out[c][cl].task_type == 2'b01);
            end
        end
    end

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
        .checkout_logical_core_i    ( replay_head_logical         ),
        .checkout_logical_cluster_i ( replay_head_logical_cluster ),
        .checkout_no_exec_i         ( replay_head_no_exec         ),
        .checkout_exported_i        ( replay_head_exported        ),
        .checkout_imported_i        ( checkout_head_imported      ),
        .export_ready_i             ( !export_full                ),
        .remote_type_en_i           ( remote_export_type_en_i     ),
        .bounce_ready_i             ( !reject_valid_q             ),
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
        .rotate_o                ( replay_rotate          ),
        .export_o                ( replay_export          ),
        .bounce_o                ( replay_bounce          )
    );
    assign replay_stuck   = (|replay_stuck_slot) || (|remote_rejected_q);
    assign replay_stuck_o = replay_stuck;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) remote_done_mismatch_q <= 1'b0;
        else         remote_done_mismatch_q <= remote_done_mismatch_q | (|remote_head_mismatch);
    end
    assign remote_done_mismatch_o = remote_done_mismatch_q;
    assign replay_data    = checkout_queue_data_out[replay_src_core][replay_src_cluster];

    always_comb begin : compose_replay_signals
        replay_pop          = '0;
        replay_push         = '0;
        replay_push_ready   = '0;
        replay_pending      = 1'b0;
        replay_move_cluster = '0;
        if (replay_move_fire) begin
            replay_pop[replay_src_core][replay_src_cluster]         = 1'b1;
            // Level 3: a bounced (rejected) imported entry is only dropped
            if (!replay_bounce) begin
                replay_push[replay_dst_core][replay_dst_cluster]        = 1'b1;
                replay_push_ready[replay_dst_core][replay_dst_cluster]  = replay_push_ready_q;
            end
        end
        // Nothing is routed into the slot being moved (fenced, not retired), only
        // into its current destination, which may be in another cluster.
        if (|replay_move) begin
            replay_move_cluster[replay_dst_cluster] = 1'b1;
        end
        for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                // A stuck slot is never migrated, so a retired core's new tasks
                // need not wait for it (its entries could only go to a core of
                // its own type, and none is live).
                if (core_fenced[c][cl] && !core_retired[c][cl] && !replay_stuck_slot[c][cl]) begin
                    replay_pending = 1'b1;
                end
            end
        end
    end

    //////////////////////////////////////////////////////////////////////
    // Core Remapping
    //////////////////////////////////////////////////////////////////////
    // A task only leaves its logical core once that core is retired (fenced and
    // its outstanding tasks replayed, see bingo_hw_manager_core_remap).
    // Dummy-set and CERF-skipped tasks never execute on a core and must drain
    // through their own core's checkout FIFO, which is what orders them after the
    // core's earlier tasks, so they are never remapped.
    for (genvar core = 0; core < NUM_CORES_PER_CLUSTER; core++) begin : gen_core_remap
        logic                                  is_dummy_set;
        bingo_hw_manager_assigned_cluster_id_t task_cluster;
        logic                                  outstanding_clear;
        logic                                  replay_hold;

        assign is_dummy_set = (waiting_dep_check_task_desc[core].task_type == 2'b01) &&
                              waiting_dep_check_task_desc[core].dep_set_info.dep_set_en;
        assign remap_remappable[core] = !is_dummy_set && !cond_exec_skip[core];

        bingo_hw_manager_core_remap #(
            .NumCores(NUM_CORES_PER_CLUSTER),
            .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
            .CoreIdWidth(cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)),
            .ClusterIdWidth(cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET)),
            .CoreTypeIdWidth(CoreTypeIdWidth),
            .CoreTypeId(CoreTypeId),
            .SubstituteLevelMask(SubstituteLevelMask)
        ) i_core_remap (
            .req_valid_i(!waiting_dep_check_queue_empty[core]),
            .logical_core_i(bingo_hw_manager_assigned_core_id_t'(core)),
            .logical_cluster_i(waiting_dep_check_task_desc[core].assigned_cluster_id),
            .remappable_i(remap_remappable[core]),
            .core_fenced_i(core_fenced),
            .core_retired_i(core_retired),
            .remote_type_en_i(remote_export_type_en_i),
            .remote_rejected_i(remote_rejected_q),
            .select_valid_o(remap_select_valid_raw[core]),
            .physical_core_o(remap_physical_core[core]),
            .physical_cluster_o(remap_physical_cluster[core]),
            .remote_o(remap_remote[core])
        );

        // A dummy-set / skipped task stays on its logical core, but earlier tasks of
        // that core may have been remapped (or replayed) elsewhere. Hold it back
        // until all of those have left their checkout queues, so that its dep_set
        // still fires after every earlier task of its logical core.
        // Healthy cores never have outstanding remapped tasks: no behaviour change.
        //
        // A fenced logical core is also held while any slot still waits to be
        // replayed: its older tasks may sit in that slot (in any cluster, after a
        // cross-cluster remap) and must reach their new core first.
        assign task_cluster = waiting_dep_check_task_desc[core].assigned_cluster_id;
        always_comb begin
            outstanding_clear = 1'b1;
            replay_hold       = 1'b0;
            for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                if (bingo_hw_manager_assigned_cluster_id_t'(cl) == task_cluster) begin
                    if (remap_outstanding_q[core][cl] != '0) begin
                        outstanding_clear = 1'b0;
                    end
                    if (core_fenced[core][cl] && replay_pending) begin
                        replay_hold = 1'b1;
                    end
                end
            end
        end
        assign remap_select_valid[core] = remap_select_valid_raw[core] &&
                                          (remap_remappable[core] || outstanding_clear) &&
                                          !replay_hold;
    end

    // Track tasks per logical core that sit in the checkout queue of another
    // physical core, in its own or another cluster (remapped or replayed;
    // checkout entries keep the logical core and cluster).
    always_comb begin : update_remap_outstanding
        remap_outstanding_d = remap_outstanding_q;
        for (int unsigned p = 0; p < NUM_CORES_PER_CLUSTER; p++) begin
            for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                if (checkout_queue_push[p][cl] &&
                    ((checkout_queue_data_in[p][cl].assigned_core_id != bingo_hw_manager_assigned_core_id_t'(p)) ||
                     (checkout_queue_data_in[p][cl].assigned_cluster_id != bingo_hw_manager_assigned_cluster_id_t'(cl))) &&
                    (int'(checkout_queue_data_in[p][cl].assigned_core_id) < NUM_CORES_PER_CLUSTER) &&
                    (int'(checkout_queue_data_in[p][cl].assigned_cluster_id) < NUM_CLUSTERS_PER_CHIPLET)) begin
                    remap_outstanding_d[checkout_queue_data_in[p][cl].assigned_core_id][checkout_queue_data_in[p][cl].assigned_cluster_id] =
                        remap_outstanding_d[checkout_queue_data_in[p][cl].assigned_core_id][checkout_queue_data_in[p][cl].assigned_cluster_id] + 1'b1;
                end
                if (checkout_queue_pop[p][cl] &&
                    ((checkout_queue_data_out[p][cl].assigned_core_id != bingo_hw_manager_assigned_core_id_t'(p)) ||
                     (checkout_queue_data_out[p][cl].assigned_cluster_id != bingo_hw_manager_assigned_cluster_id_t'(cl))) &&
                    (int'(checkout_queue_data_out[p][cl].assigned_core_id) < NUM_CORES_PER_CLUSTER) &&
                    (int'(checkout_queue_data_out[p][cl].assigned_cluster_id) < NUM_CLUSTERS_PER_CHIPLET)) begin
                    remap_outstanding_d[checkout_queue_data_out[p][cl].assigned_core_id][checkout_queue_data_out[p][cl].assigned_cluster_id] =
                        remap_outstanding_d[checkout_queue_data_out[p][cl].assigned_core_id][checkout_queue_data_out[p][cl].assigned_cluster_id] - 1'b1;
                end
            end
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin : remap_outstanding_regs
        if (!rst_ni) begin
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                    remap_outstanding_q[c][cl] <= '0;
                end
            end
        end else begin
            remap_outstanding_q <= remap_outstanding_d;
        end
    end

    always_comb begin : compose_remap_route_signals
        remap_route_valid = '0;
        remap_route_src_core = '0;

        for (int unsigned dst_core = 0; dst_core < NUM_CORES_PER_CLUSTER; dst_core++) begin
            for (int unsigned dst_cluster = 0; dst_cluster < NUM_CLUSTERS_PER_CHIPLET; dst_cluster++) begin
                for (int unsigned src_core = 0; src_core < NUM_CORES_PER_CLUSTER; src_core++) begin
                    // Replay pushes have priority over routed tasks in their cluster.
                    if (!remap_route_valid[dst_core][dst_cluster] &&
                        !replay_move_cluster[dst_cluster] &&
                        demux_ready_and_checkout_queue_oup_valid[src_core][dst_cluster] &&
                        remap_select_valid[src_core] &&
                        (remap_physical_core[src_core] == bingo_hw_manager_assigned_core_id_t'(dst_core)) &&
                        (remap_physical_cluster[src_core] == bingo_hw_manager_assigned_cluster_id_t'(dst_cluster))) begin

                        remap_route_valid[dst_core][dst_cluster] = 1'b1;
                        remap_route_src_core[dst_core][dst_cluster] =
                            bingo_hw_manager_assigned_core_id_t'(src_core);
                    end
                end
            end
        end
    end

    //////////////////////////////////////////////////////////////////////
    // Level 3: remote dispatch (SubstituteLevelMask[2])
    //////////////////////////////////////////////////////////////////////
    // Origin side. A task whose logical slot D is fenced and has no live
    // substitute on this chiplet (any enabled level) stays in D's checkout
    // queue, marked exported, and a copy leaves through remote_dispatch_o:
    //   - outstanding tasks: rotated by the replay controller (see there)
    //   - new tasks of a retired D: routed to D's checkout queue only
    // D is the proxy: the remote done (remote_done_i) is pushed into D's done
    // FIFO and retires the head with the normal dep_set, so dependencies,
    // dummy-sets and the per-logical-core order are handled as for a local
    // task. The remote chiplet must return the dones of one proxy in export
    // order (it runs them on one core of that type, in order).
    // Executor side. An imported task goes to the lowest live slot of its
    // CoreTypeId (level 1/2 substitute of the lowest slot of that type), into
    // its ready and checkout queues, as a task of that home slot, tagged with
    // its origin. When it retires, its done leaves through remote_done_o
    // instead of setting local dependencies. Imports wait while a local replay
    // is pending, and never collide with a routed or replayed push.

    // Export FIFO: replay rotations and routed exports never coincide (the
    // new tasks of fenced logical cores are held while a replay is pending);
    // routed exports of different slots are granted one per cycle.
    always_comb begin : compose_route_remote_grant
        automatic logic taken;
        taken = replay_export || export_full;
        route_remote_grant = '0;
        for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                if (route_remote[c][cl] && !checkout_queue_full[c][cl] && !taken) begin
                    route_remote_grant[c][cl] = 1'b1;
                    taken = 1'b1;
                end
            end
        end
    end

    always_comb begin : compose_export_input
        export_in   = '0;
        export_push = 1'b0;
        if (replay_export) begin
            export_push          = 1'b1;
            export_in.desc       = desc_to_full(replay_data);
            export_in.core_type  = CoreTypeId[replay_data.assigned_core_id][replay_data.assigned_cluster_id];
            export_in.proxy_slot = remote_slot_t'(replay_src_core + replay_src_cluster * NUM_CORES_PER_CLUSTER);
        end
        for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                if (route_remote_grant[c][cl] && remap_route_fire[c][cl]) begin
                    export_push          = 1'b1;
                    export_in.desc       = desc_to_full(waiting_dep_check_task_desc[remap_route_src_core[c][cl]]);
                    export_in.core_type  = CoreTypeId[c][cl];
                    export_in.proxy_slot = remote_slot_t'(c + cl * NUM_CORES_PER_CLUSTER);
                end
            end
        end
    end

    if (RemoteEn) begin : gen_remote_dispatch
        fifo_v3 #(
            .FALL_THROUGH ( 1'b0            ),
            .DEPTH        ( 4               ),
            .dtype        ( remote_export_t )
        ) i_export_fifo (
            .clk_i       ( clk_i                                            ),
            .rst_ni      ( rst_ni                                           ),
            .testmode_i  ( 1'b0                                             ),
            .flush_i     ( 1'b0                                             ),
            .full_o      ( export_full                                      ),
            .empty_o     ( export_empty                                     ),
            .usage_o     ( /*not used*/                                     ),
            .data_i      ( export_in                                        ),
            .push_i      ( export_push                                      ),
            .data_o      ( export_out                                       ),
            .pop_i       ( remote_dispatch_ready_i && !export_empty         )
        );
        assign remote_dispatch_valid_o       = !export_empty;
        assign remote_dispatch_desc_o        = host_axi_lite_data_t'(export_out.desc);
        assign remote_dispatch_core_type_o   = export_out.core_type;
        assign remote_dispatch_origin_chip_o = chip_id_i;
        assign remote_dispatch_proxy_slot_o  = export_out.proxy_slot;

        // Import: home slot = lowest slot of the requested type
        always_comb begin : find_import_home
            import_home_found   = 1'b0;
            import_home_core    = '0;
            import_home_cluster = '0;
            for (int cl = NUM_CLUSTERS_PER_CHIPLET - 1; cl >= 0; cl--) begin
                for (int c = NUM_CORES_PER_CLUSTER - 1; c >= 0; c--) begin
                    if ((remote_dispatch_core_type_i != '0) &&
                        (CoreTypeId[c][cl] == remote_dispatch_core_type_i)) begin
                        import_home_found   = 1'b1;
                        import_home_core    = bingo_hw_manager_assigned_core_id_t'(c);
                        import_home_cluster = bingo_hw_manager_assigned_cluster_id_t'(cl);
                    end
                end
            end
        end
        // Its live stand-in on this chiplet (never re-exported)
        bingo_hw_manager_substitute_sel #(
            .NumCores(NUM_CORES_PER_CLUSTER),
            .NumClusters(NUM_CLUSTERS_PER_CHIPLET),
            .CoreIdWidth(cf_math_pkg::idx_width(NUM_CORES_PER_CLUSTER)),
            .ClusterIdWidth(cf_math_pkg::idx_width(NUM_CLUSTERS_PER_CHIPLET)),
            .CoreTypeIdWidth(CoreTypeIdWidth),
            .CoreTypeId(CoreTypeId),
            .LevelMask(ImportSubstituteLevelMask & 3'b011)
        ) i_import_sel (
            .logical_core_i(import_home_core),
            .logical_cluster_i(import_home_cluster),
            .fenced_i(core_fenced),
            .found_o(import_sub_found),
            .core_o(import_core),
            .cluster_o(import_cluster)
        );
        always_comb begin : compose_import_desc
            automatic bingo_hw_manager_task_desc_full_t f;
            f = bingo_hw_manager_task_desc_full_t'(remote_dispatch_desc_i);
            import_desc.task_id             = f.task_id;
            import_desc.task_type           = f.task_type;
            import_desc.assigned_chiplet_id = chip_id_i;
            import_desc.assigned_cluster_id = import_home_cluster;
            import_desc.assigned_core_id    = import_home_core;
            import_desc.dep_check_info      = '0;
            import_desc.dep_set_info        = '0;
            import_desc.cond_exec_en        = 1'b0;
            import_desc.cond_exec_group_id  = '0;
            import_desc.cond_exec_invert    = 1'b0;
        end
        assign import_fire = remote_dispatch_valid_i && import_home_found && import_sub_found &&
                             !replay_pending && !(|replay_move) &&
                             !remap_route_valid[import_core][import_cluster] &&
                             !replay_push[import_core][import_cluster] &&
                             !checkout_queue_full[import_core][import_cluster] &&
                             !ready_queue_full[import_core][import_cluster];
        always_comb begin
            import_push = '0;
            import_push[import_core][import_cluster] = import_fire;
        end
        // No live core here may run it, now or later (fenced is sticky): take it
        // and reject it back to its origin instead of blocking the link. Waits
        // for a local replay, which may first bounce older imports of the type.
        assign import_reject = remote_dispatch_valid_i && !(import_home_found && import_sub_found) &&
                               !replay_pending && !(|replay_move) && !reject_valid_q;
        assign remote_dispatch_ready_o = import_fire || import_reject;

        // One pending reject (import_reject or a bounced imported entry)
        localparam int unsigned NumSlots = NUM_CORES_PER_CLUSTER * NUM_CLUSTERS_PER_CHIPLET;
        logic reject_arb_valid, reject_arb_ready, reject_raised_q;
        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (!rst_ni) begin
                reject_valid_q  <= 1'b0;
                reject_q        <= '0;
                reject_raised_q <= 1'b0;
            end else begin
                if (reject_arb_valid && reject_arb_ready) begin
                    reject_valid_q  <= 1'b0;
                    reject_raised_q <= 1'b0;
                end else if (reject_arb_valid) begin
                    reject_raised_q <= 1'b1;
                end
                if (import_reject) begin
                    reject_valid_q <= 1'b1;
                    reject_q       <= '{chip:       remote_dispatch_origin_chip_i,
                                        proxy_slot: remote_dispatch_proxy_slot_i,
                                        task_id:    import_desc.task_id,
                                        reject:     1'b1};
                end else if (replay_bounce) begin
                    reject_valid_q <= 1'b1;
                    reject_q       <= '{chip:       checkout_remote_tag_out[replay_src_core][replay_src_cluster].origin_chip,
                                        proxy_slot: checkout_remote_tag_out[replay_src_core][replay_src_cluster].proxy_slot,
                                        task_id:    replay_data.task_id,
                                        reject:     1'b1};
                end
            end
        end
        // Below every retiring done: the dones of a proxy slot reach its origin
        // before its reject (once raised, the request is held until taken)
        assign reject_arb_valid = reject_valid_q && (reject_raised_q || !(|remote_done_arb_valid));

        // Dones of imported tasks and rejects, back to their origin
        remote_done_t [NumSlots:0] remote_done_all_data;
        logic         [NumSlots:0] remote_done_all_valid, remote_done_all_ready;
        assign remote_done_all_data  = {reject_q, remote_done_arb_data};
        assign remote_done_all_valid = {reject_arb_valid, remote_done_arb_valid};
        assign {reject_arb_ready, remote_done_arb_ready} = remote_done_all_ready;
        stream_arbiter #(
            .DATA_T(remote_done_t),
            .N_INP (NumSlots + 1)
        ) i_remote_done_arbiter (
            .clk_i      ( clk_i                 ),
            .rst_ni     ( rst_ni                ),
            .inp_data_i ( remote_done_all_data  ),
            .inp_valid_i( remote_done_all_valid ),
            .inp_ready_o( remote_done_all_ready ),
            .oup_data_o ( remote_done_out       ),
            .oup_valid_o( remote_done_valid_o   ),
            .oup_ready_i( remote_done_ready_i   )
        );
        assign remote_done_chip_o       = remote_done_out.chip;
        assign remote_done_proxy_slot_o = remote_done_out.proxy_slot;
        assign remote_done_task_id_o    = remote_done_out.task_id;
        assign remote_done_reject_o     = remote_done_out.reject;
    end else begin : gen_no_remote_dispatch
        assign export_full                   = 1'b1;
        assign export_empty                  = 1'b1;
        assign export_out                    = '0;
        assign remote_dispatch_valid_o       = 1'b0;
        assign remote_dispatch_desc_o        = '0;
        assign remote_dispatch_core_type_o   = '0;
        assign remote_dispatch_origin_chip_o = '0;
        assign remote_dispatch_proxy_slot_o  = '0;
        assign import_home_found             = 1'b0;
        assign import_home_core              = '0;
        assign import_home_cluster           = '0;
        assign import_sub_found              = 1'b0;
        assign import_core                   = '0;
        assign import_cluster                = '0;
        assign import_desc                   = '0;
        assign import_fire                   = 1'b0;
        assign import_reject                 = 1'b0;
        assign import_push                   = '0;
        assign remote_dispatch_ready_o       = 1'b0;
        assign reject_valid_q                = 1'b0;
        assign reject_q                      = '0;
        assign remote_done_arb_ready         = '0;
        assign remote_done_out               = '0;
        assign remote_done_valid_o           = 1'b0;
        assign remote_done_chip_o            = '0;
        assign remote_done_proxy_slot_o      = '0;
        assign remote_done_task_id_o         = '0;
        assign remote_done_reject_o          = 1'b0;
    end

    // Origin side: a proxy slot whose export was rejected stops retiring and
    // exporting (sticky, part of replay_stuck_o)
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) remote_rejected_q <= '0;
        else         remote_rejected_q <= remote_rejected_q | remote_reject_in;
    end

`ifndef SYNTHESIS
    // Simulation-only event log: makes watchdog / remap / replay activity visible
    // in any flow that simulates this RTL (bingo unit tests and full-system sims).
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_dead_suspect_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_fenced_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] core_retired_log_q;
    logic [NUM_CORES_PER_CLUSTER-1:0][NUM_CLUSTERS_PER_CHIPLET-1:0] replay_stuck_log_q;
    always @(posedge clk_i or negedge rst_ni) begin : watchdog_remap_event_log
        if (!rst_ni) begin
            core_dead_suspect_log_q <= '0;
            core_fenced_log_q       <= '0;
            core_retired_log_q      <= '0;
            replay_stuck_log_q      <= '0;
        end else begin
            core_dead_suspect_log_q <= core_dead_suspect;
            core_fenced_log_q       <= core_fenced;
            core_retired_log_q      <= core_retired;
            replay_stuck_log_q      <= replay_stuck_slot;
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                    if ((core_dead_suspect[c][cl] != core_dead_suspect_log_q[c][cl]) ||
                        (core_fenced[c][cl] != core_fenced_log_q[c][cl])) begin
                        $display("[BINGO_WD] %0t chip=%0d core=%0d cluster=%0d dead_suspect=%0b fenced=%0b",
                                 $time, chip_id_i, c, cl, core_dead_suspect[c][cl], core_fenced[c][cl]);
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
                    if (remap_route_fire[c][cl] &&
                        ((remap_route_src_core[c][cl] != bingo_hw_manager_assigned_core_id_t'(c)) ||
                         (waiting_dep_check_task_desc[remap_route_src_core[c][cl]].assigned_cluster_id !=
                          bingo_hw_manager_assigned_cluster_id_t'(cl)))) begin
                        $display("[BINGO_REMAP] %0t chip=%0d task=%0d logical_core=%0d -> physical_core=%0d cluster=%0d logical_cluster=%0d",
                                 $time, chip_id_i,
                                 waiting_dep_check_task_desc[remap_route_src_core[c][cl]].task_id,
                                 remap_route_src_core[c][cl], c, cl,
                                 waiting_dep_check_task_desc[remap_route_src_core[c][cl]].assigned_cluster_id);
                    end
                end
            end
            if (export_push) begin
                $display("[BINGO_EXPORT] %0t chip=%0d task=%0d type_id=%0d proxy_slot=%0d (replay %0b)",
                         $time, chip_id_i, export_in.desc.task_id, export_in.core_type,
                         export_in.proxy_slot, replay_export);
            end
            if (import_fire) begin
                $display("[BINGO_IMPORT] %0t chip=%0d task=%0d from chip=%0d proxy_slot=%0d -> core=%0d cluster=%0d (home core=%0d cluster=%0d)",
                         $time, chip_id_i, import_desc.task_id, remote_dispatch_origin_chip_i,
                         remote_dispatch_proxy_slot_i, import_core, import_cluster,
                         import_home_core, import_home_cluster);
            end
            if (remote_done_valid_o && remote_done_ready_i && !remote_done_reject_o) begin
                $display("[BINGO_REMOTE_DONE_OUT] %0t chip=%0d task=%0d -> chip=%0d proxy_slot=%0d",
                         $time, chip_id_i, remote_done_task_id_o, remote_done_chip_o, remote_done_proxy_slot_o);
            end
            if (remote_done_valid_i && remote_done_ready_o && !remote_done_reject_i) begin
                $display("[BINGO_REMOTE_DONE_IN] %0t chip=%0d task=%0d proxy_slot=%0d",
                         $time, chip_id_i, remote_done_task_id_i, remote_done_proxy_slot_i);
            end
            if (import_reject) begin
                $display("[BINGO_REMOTE_REJECT_OUT] %0t chip=%0d task=%0d -> chip=%0d proxy_slot=%0d (no live core of its type)",
                         $time, chip_id_i, import_desc.task_id, remote_dispatch_origin_chip_i,
                         remote_dispatch_proxy_slot_i);
            end
            if (replay_bounce) begin
                $display("[BINGO_REMOTE_REJECT_OUT] %0t chip=%0d task=%0d -> chip=%0d proxy_slot=%0d (its core died, no other live core)",
                         $time, chip_id_i, replay_data.task_id,
                         checkout_remote_tag_out[replay_src_core][replay_src_cluster].origin_chip,
                         checkout_remote_tag_out[replay_src_core][replay_src_cluster].proxy_slot);
            end
            if (|remote_reject_in) begin
                $display("[BINGO_REMOTE_REJECT_IN] %0t chip=%0d task=%0d proxy_slot=%0d: the proxy is stuck",
                         $time, chip_id_i, remote_done_task_id_i, remote_done_proxy_slot_i);
            end
            if (replay_move_fire && !replay_bounce) begin
                $display("[BINGO_REPLAY] %0t chip=%0d task=%0d type=%0d logical_core=%0d from=%0d to=%0d cluster=%0d to_cluster=%0d logical_cluster=%0d",
                         $time, chip_id_i, replay_data.task_id, replay_data.task_type,
                         replay_data.assigned_core_id, replay_src_core, replay_dst_core, replay_src_cluster,
                         replay_dst_cluster, replay_data.assigned_cluster_id);
            end
        end
    end

    // Replay invariants
    always @(posedge clk_i) begin : replay_assertions
        if (rst_ni) begin
            for (int unsigned c = 0; c < NUM_CORES_PER_CLUSTER; c++) begin
                for (int unsigned cl = 0; cl < NUM_CLUSTERS_PER_CHIPLET; cl++) begin
                    // A1: a fenced core neither takes a task nor retires one
                    if (core_fenced[c][cl] && (ready_queue_pop[c][cl] ||
                                               (done_q_push[c][cl] && !remote_done_push[c][cl]))) begin
                        $error("[BINGO_ASSERT] fenced core %0d cluster %0d: ready pop %0b / done push %0b",
                               c, cl, ready_queue_pop[c][cl], done_q_push[c][cl]);
                    end
                    // A2: a fenced core receives no executing task; before it is
                    // retired it receives nothing at all
                    if (core_fenced[c][cl] && ready_queue_push[c][cl]) begin
                        $error("[BINGO_ASSERT] ready push into fenced core %0d cluster %0d", c, cl);
                    end
                    // (level 3: exported entries stay on their fenced proxy slot)
                    if (core_fenced[c][cl] && checkout_queue_push[c][cl] &&
                        !checkout_remote_tag_in[c][cl].exported &&
                        (!core_retired[c][cl] || (checkout_queue_data_in[c][cl].task_type != 2'b01))) begin
                        $error("[BINGO_ASSERT] checkout push of task %0d (type %0d) into fenced core %0d cluster %0d (retired %0b)",
                               checkout_queue_data_in[c][cl].task_id, checkout_queue_data_in[c][cl].task_type,
                               c, cl, core_retired[c][cl]);
                    end
                    // A3: outstanding counter never wraps
                    if (remap_outstanding_q[c][cl] > NUM_CORES_PER_CLUSTER * CheckoutQueueDepth) begin
                        $error("[BINGO_ASSERT] remap_outstanding[%0d][%0d] out of range: %0d",
                               c, cl, remap_outstanding_q[c][cl]);
                    end
                    // A4: a slot being moved is not retired through its done queue
                    if (replay_move[c][cl] && done_q_pop[c][cl]) begin
                        $error("[BINGO_ASSERT] done pop on core %0d cluster %0d during replay MOVE", c, cl);
                    end
                    // A5: a replay push never collides with a routed push
                    if (replay_push[c][cl] && remap_route_fire[c][cl]) begin
                        $error("[BINGO_ASSERT] replay and routed push into core %0d cluster %0d", c, cl);
                    end
                    // A7: an executing task retires together with its own done
                    if (checkout_queue_pop[c][cl] && !replay_pop[c][cl] &&
                        (checkout_queue_data_out[c][cl].task_type != 2'b01) &&
                        (!done_q_pop[c][cl] || done_q_empty[c][cl] ||
                         (done_q_info[c][cl].task_id != checkout_queue_data_out[c][cl].task_id))) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d retired task %0d with done pop %0b (done task %0d)",
                               c, cl, checkout_queue_data_out[c][cl].task_id, done_q_pop[c][cl],
                               done_q_info[c][cl].task_id);
                    end
                    if (done_q_pop[c][cl] &&
                        !(checkout_queue_pop[c][cl] && !replay_pop[c][cl])) begin
                        $error("[BINGO_ASSERT] core %0d cluster %0d popped done of task %0d without retiring a task",
                               c, cl, done_q_info[c][cl].task_id);
                    end
                    // A8: a stuck slot keeps its entries
                    if (replay_stuck_slot[c][cl] && checkout_queue_pop[c][cl]) begin
                        $error("[BINGO_ASSERT] stuck core %0d cluster %0d popped task %0d",
                               c, cl, checkout_queue_data_out[c][cl].task_id);
                    end
                    // A6: fenced / retired / stuck are sticky
                    if ((core_fenced_log_q[c][cl] && !core_fenced[c][cl]) ||
                        (core_retired_log_q[c][cl] && !core_retired[c][cl]) ||
                        (replay_stuck_log_q[c][cl] && !replay_stuck_slot[c][cl])) begin
                        $error("[BINGO_ASSERT] fenced/retired/stuck of core %0d cluster %0d fell", c, cl);
                    end
                end
            end
        end
    end
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
        .cerf_write_en_i  ( cerf_write_en_i        )
    );

endmodule
