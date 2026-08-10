# Copyright © 2019-2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Zynq-7000 (Arty Z7) integration: PS7 + Vortex_z7_top module reference.
#   - M_AXI_GP0 -> smartconnect -> Vortex_z7_top/s_axi_ctrl @ 0x43C00000 (64K)
#   - Vortex_z7_top/m_axi_mem -> smartconnect -> S_AXI_HP0 (DDR @ 0x0, 512M)
#   - FCLK_CLK0 (CLK_MHZ, default 50) clocks the whole PL design.
# Requires the Digilent board files (BOARD_REPO env) for the PS7 DDR/MIO preset.

if { $::argc != 2 } {
    puts "ERROR: Program \"$::argv0\" requires 2 arguments!\n"
    puts "Usage: $::argv0 <device_part> <vcs_file>\n"
    exit 1
}

set device_part [lindex $::argv 0]
set vcs_file [lindex $::argv 1]

set tool_dir $::env(TOOL_DIR)
set script_dir [ file dirname [ file normalize [ info script ] ] ]

set clk_mhz 50
if {[info exists ::env(CLK_MHZ)]} { set clk_mhz $::env(CLK_MHZ) }

if {[info exists ::env(MAX_JOBS)]} {
  set num_jobs $::env(MAX_JOBS)
} else {
  set num_jobs 0
}

puts "Using device_part=$device_part vcs_file=$vcs_file clk_mhz=$clk_mhz"

# Digilent board files provide the PS7 DDR3/MIO preset for the target board.
# board.repoPaths must be set before any board query in the session.
set board_name "arty-z7-10"
if {[info exists ::env(BOARD)]} { set board_name $::env(BOARD) }
if {[info exists ::env(BOARD_REPO)]} {
  set_param board.repoPaths [list $::env(BOARD_REPO)]
}
set board_parts [get_board_parts -quiet *${board_name}*]
if {[llength $board_parts] == 0} {
  puts "ERROR: ${board_name} board part not found. Set BOARD_REPO to the new/board_files subdir of a https://github.com/Digilent/vivado-boards checkout."
  exit 1
}
set board_part [lindex $board_parts end]
puts "Using board_part=$board_part"

proc run_setup {} {
  global device_part vcs_file tool_dir script_dir clk_mhz board_part
  global argv argc ;# xilinx_ip_gen.tcl reads the global ::argv/::argc

  set project_name "project_1"

  # create the Xilinx floating_point FPU IP when EXT_F is enabled — before
  # create_project: the generator opens its own in-memory project.
  # (FPU_IP env = ip output dir, same contract as the dut flow)
  if {[info exists ::env(FPU_IP)]} {
    set ip_dir $::env(FPU_IP)
    set argv [list $ip_dir $device_part]
    set argc 2
    source ${tool_dir}/xilinx_ip_gen.tcl
  }

  source "${tool_dir}/parse_vcs_list.tcl"
  set vlist [parse_vcs_list "${vcs_file}"]

  set vsources_list  [lindex $vlist 0]
  set vincludes_list [lindex $vlist 1]
  set vdefines_list  [lindex $vlist 2]

  create_project $project_name $project_name -force -part $device_part
  set_property board_part $board_part [current_project]

  if {[info exists ::env(FPU_IP)]} {
    set ip_dir $::env(FPU_IP)
    add_files -norecurse -verbose ${ip_dir}/xil_fma/xil_fma.xci
    add_files -norecurse -verbose ${ip_dir}/xil_fdiv/xil_fdiv.xci
    add_files -norecurse -verbose ${ip_dir}/xil_fsqrt/xil_fsqrt.xci
    add_files -norecurse -verbose ${ip_dir}/xil_fmul/xil_fmul.xci
    add_files -norecurse -verbose ${ip_dir}/xil_fadd/xil_fadd.xci
  }

  set obj [get_filesets sources_1]
  add_files -norecurse -verbose -fileset $obj ${vsources_list}
  foreach def $vdefines_list {
    set_property -name "verilog_define" -value $def -objects $obj
  }
  update_compile_order -fileset sources_1

  # Block design
  create_bd_design "design_1"

  # PS7 with board preset (DDR + MIO + UART0)
  set ps7 [ create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 processing_system7_0 ]
  apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps7
  set_property -dict [ list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
    CONFIG.PCW_EN_CLK0_PORT {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ $clk_mhz \
  ] $ps7

  # Vortex top (module reference): Vortex_z7_top (shim) or Vortex_z7_cp_top
  # (Command Processor integration) — selected by the Makefile's CP knob.
  set vx_top "Vortex_z7_top"
  if {[info exists ::env(VX_TOP)]} { set vx_top $::env(VX_TOP) }
  set with_cp [expr {$vx_top eq "Vortex_z7_cp_top"}]
  set vortex_top [ create_bd_cell -type module -reference $vx_top Vortex_z7_top_0 ]

  # Reset generator
  set rst [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset proc_sys_reset_0 ]

  # Control path: GP0 -> smartconnect -> s_axi_ctrl
  set smc_ctrl [ create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect smartconnect_ctrl ]
  set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $smc_ctrl

  # Memory path: m_axi_mem (+ m_axi_host with CP) -> smartconnect -> S_AXI_HP0
  set smc_mem [ create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect smartconnect_mem ]
  set num_si [expr {$with_cp ? 2 : 1}]
  set_property -dict [list CONFIG.NUM_SI $num_si CONFIG.NUM_MI {1}] $smc_mem

  # Clocks: FCLK_CLK0 drives everything
  connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins processing_system7_0/M_AXI_GP0_ACLK] \
    [get_bd_pins processing_system7_0/S_AXI_HP0_ACLK] \
    [get_bd_pins Vortex_z7_top_0/clk] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
    [get_bd_pins smartconnect_ctrl/aclk] \
    [get_bd_pins smartconnect_mem/aclk]

  # Resets
  connect_bd_net [get_bd_pins processing_system7_0/FCLK_RESET0_N] [get_bd_pins proc_sys_reset_0/ext_reset_in]
  connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins Vortex_z7_top_0/resetn] \
    [get_bd_pins smartconnect_ctrl/aresetn] \
    [get_bd_pins smartconnect_mem/aresetn]

  # AXI connections
  connect_bd_intf_net [get_bd_intf_pins processing_system7_0/M_AXI_GP0] [get_bd_intf_pins smartconnect_ctrl/S00_AXI]
  connect_bd_intf_net [get_bd_intf_pins smartconnect_ctrl/M00_AXI] [get_bd_intf_pins Vortex_z7_top_0/s_axi_ctrl]
  connect_bd_intf_net [get_bd_intf_pins Vortex_z7_top_0/m_axi_mem] [get_bd_intf_pins smartconnect_mem/S00_AXI]
  if {$with_cp} {
    connect_bd_intf_net [get_bd_intf_pins Vortex_z7_top_0/m_axi_host] [get_bd_intf_pins smartconnect_mem/S01_AXI]
  }
  connect_bd_intf_net [get_bd_intf_pins smartconnect_mem/M00_AXI] [get_bd_intf_pins processing_system7_0/S_AXI_HP0]

  # Optional System ILA on the Vortex memory AXI (ENABLE_ILA=1)
  if {[info exists ::env(ENABLE_ILA)] && $::env(ENABLE_ILA) == 1} {
    set mem_net [get_bd_intf_nets -of_objects [get_bd_intf_pins Vortex_z7_top_0/m_axi_mem]]
    apply_bd_automation -rule xilinx.com:bd_rule:debug -dict [list \
      $mem_net {AXI_R_ADDRESS "Data and Trigger" AXI_R_DATA "Data and Trigger" AXI_W_ADDRESS "Data and Trigger" AXI_W_DATA "Data and Trigger" AXI_W_RESPONSE "Data and Trigger" CLK_SRC "/processing_system7_0/FCLK_CLK0" SYSTEM_ILA "Auto" APC_EN "0"} ]
    set_property CONFIG.C_DATA_DEPTH 2048 [get_bd_cells -quiet *system_ila*]
  }

  # Address map
  assign_bd_address -offset 0x43C00000 -range 0x00010000 \
    -target_address_space [get_bd_addr_spaces processing_system7_0/Data] \
    [get_bd_addr_segs Vortex_z7_top_0/s_axi_ctrl/reg0] -force
  assign_bd_address -offset 0x00000000 -range 0x20000000 \
    -target_address_space [get_bd_addr_spaces Vortex_z7_top_0/m_axi_mem] \
    [get_bd_addr_segs processing_system7_0/S_AXI_HP0/HP0_DDR_LOWOCM] -force
  if {$with_cp} {
    assign_bd_address -offset 0x00000000 -range 0x20000000 \
      -target_address_space [get_bd_addr_spaces Vortex_z7_top_0/m_axi_host] \
      [get_bd_addr_segs processing_system7_0/S_AXI_HP0/HP0_DDR_LOWOCM] -force
  }

  validate_bd_design
  save_bd_design
  close_bd_design "design_1"

  # Global (non-OOC) synthesis: the Vortex module reference instantiates the
  # floating_point IP (xil_fma etc.), which OOC module-ref sub-runs cannot
  # resolve — everything must synthesize in one pass when the FPU is enabled.
  if {[info exists ::env(FPU_IP)]} {
    set_property SYNTH_CHECKPOINT_MODE "None" [get_files design_1.bd]
  } else {
    set_property SYNTH_CHECKPOINT_MODE "Hierarchical" [get_files design_1.bd]
  }

  # Wrapper
  set wrapper_path [make_wrapper -fileset sources_1 -files [get_files -norecurse design_1.bd] -top]
  add_files -norecurse -fileset sources_1 $wrapper_path
  set_property -name "top" -value "design_1_wrapper" -objects [get_filesets sources_1]

  # ASYNC_BRAM_PATCH netlist fixup (see hw/scripts/xilinx_async_bram_patch.tcl)
  set_property STEPS.OPT_DESIGN.TCL.PRE ${script_dir}/pre_opt_hook.tcl [get_runs impl_1]

  set opt_level 3
  if {[info exists ::env(OPT_LEVEL)]} { set opt_level $::env(OPT_LEVEL) }
  if {$opt_level == 0} {
    set_property strategy "Flow_RuntimeOptimized"        [get_runs synth_1]
    set_property strategy "Performance_RuntimeOptimized" [get_runs impl_1]
  }
  # AREA_OPT=1: trade Fmax for slices (needed for near-capacity designs
  # like the 512-bit CP integration on the Z020).
  if {[info exists ::env(AREA_OPT)] && $::env(AREA_OPT) == 1} {
    set_property strategy "Flow_AreaOptimized_high" [get_runs synth_1]
    set_property strategy "Area_Explore"            [get_runs impl_1]
  }

  update_compile_order -fileset sources_1
}

proc run_build {} {
  global num_jobs
  if {$num_jobs != 0} {
    launch_runs impl_1 -to_step write_bitstream -jobs $num_jobs
  } else {
    launch_runs impl_1 -to_step write_bitstream
  }
  wait_on_run impl_1
  open_run impl_1
  report_utilization -file post_impl_util.rpt
  report_timing_summary -file timing.rpt
  report_drc -file drc.rpt

  # export outputs
  file copy -force project_1/project_1.runs/impl_1/design_1_wrapper.bit vortex_z7.bit
  write_hw_platform -fixed -include_bit -force vortex_z7.xsa
}

set start_time [clock seconds]

run_setup
run_build

set elapsed_time [expr {[clock seconds] - $start_time}]
puts "Total elapsed time: [format %02d [expr {$elapsed_time / 3600}]]h [format %02d [expr {($elapsed_time % 3600) / 60}]]m [format %02d [expr {$elapsed_time % 60}]]s"
