`define TB_STIMULUS_FILE "tb_stimulus_remap_mask_off.svh"
`define TB_NUM_CHIPLET 1
`define TB_NUM_CLUSTERS_PER_CHIPLET 1
`define TB_NUM_CORES_PER_CLUSTER 3
`define TB_WATCHDOG_HEARTBEAT_TIMEOUT 32
`define TB_CORE_REMAP_ALLOW_MASK '0
`define TB_DISABLE_CORE_WORKERS 1

module tb_bingo_hw_manager_remap_mask_off;
  `include "tb_bingo_hw_manager_harness.svh"
endmodule
