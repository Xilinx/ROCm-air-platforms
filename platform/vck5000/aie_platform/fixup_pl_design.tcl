
# Function to get the working directory of the 
namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}
variable script_folder
set script_folder [_tcl::get_script_folder]

set platform_name [lindex $argv 0]
puts "Fixing up the shim DMA NOC connections before generating the hardware" 

# Open the internal block design to fixup the connections
puts "Opening the internal Vitis block design" 
open_project $script_folder/_x/link/vivado/vpl/prj/prj.xpr
open_bd_design ${script_folder}/_x/link/vivado/vpl/prj/prj.srcs/sources_1/bd/project_1/project_1.bd

# Connecting the reset
connect_bd_net [get_bd_pins proc_sys_reset_0/ext_reset_in] [get_bd_pins qdma_host_mem/axi_aresetn]

# Deleting the dummy HLS kernel
delete_bd_objs [get_bd_intf_nets counter_hls_0_strm_out] [get_bd_intf_nets cdc_ai_engine_0_M00_AXIS_M_AXIS] [get_bd_cells counter_hls_0]

# Instantiating and configuring the axi dma IP
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 axi_dma_0
set_property -dict [list CONFIG.c_include_sg {0} CONFIG.c_sg_include_stscntrl_strm {0} CONFIG.c_m_axi_mm2s_data_width {64} CONFIG.c_m_axis_mm2s_tdata_width {64} CONFIG.c_mm2s_burst_size {8} CONFIG.c_addr_width {64}] [get_bd_cells axi_dma_0]

# Modifying the NOC to accomodate the AXI dma
startgroup
set_property -dict [list CONFIG.NUM_SI {11} CONFIG.NUM_MI {3} CONFIG.NUM_CLKS {10}] [get_bd_cells axi_noc_0]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5}} }] [get_bd_intf_pins /axi_noc_0/S00_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S01_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S02_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S03_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S04_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S05_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S06_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S07_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S08_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S09_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M01_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M02_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} M00_AXI { read_bw {5} write_bw {5} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S10_AXI]
set_property -dict [list CONFIG.ASSOCIATED_BUSIF {S03_AXI:S09_AXI:S10_AXI:M01_AXI:M02_AXI:S04_AXI}] [get_bd_pins /axi_noc_0/aclk0]
endgroup

# Connecting the new NOC properly
connect_bd_net [get_bd_pins axi_noc_0/aclk9] [get_bd_pins clk_wiz/clk_out1]
set_property -dict [list CONFIG.ASSOCIATED_BUSIF {M02_AXI:S10_AXI}] [get_bd_pins axi_noc_0/aclk9]

# Instantiating a smart connect to connect the AXI DMA to the NOC
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_0

# Connecting the smart connect
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXI_MM2S] [get_bd_intf_pins smartconnect_0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXI_S2MM] [get_bd_intf_pins smartconnect_0/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins smartconnect_0/M00_AXI] [get_bd_intf_pins axi_noc_0/S10_AXI]
connect_bd_net [get_bd_pins smartconnect_0/aclk] [get_bd_pins clk_wiz/clk_out1]
connect_bd_net [get_bd_pins smartconnect_0/aresetn] [get_bd_pins rst_clk_wiz_250M/peripheral_aresetn]

# Instantiating a smart connect for the AXI lite connection
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect_2
connect_bd_intf_net [get_bd_intf_pins axi_noc_0/M02_AXI] [get_bd_intf_pins smartconnect_2/S00_AXI]
set_property -dict [list CONFIG.NUM_SI {1}] [get_bd_cells smartconnect_2]
connect_bd_intf_net [get_bd_intf_pins smartconnect_2/M00_AXI] [get_bd_intf_pins axi_dma_0/S_AXI_LITE]
connect_bd_net [get_bd_pins smartconnect_2/aclk] [get_bd_pins clk_wiz/clk_out1]
connect_bd_net [get_bd_pins smartconnect_2/aresetn] [get_bd_pins rst_clk_wiz_250M/peripheral_aresetn]

# Connecting the axi DMA IP
connect_bd_intf_net [get_bd_intf_pins cdc_ai_engine_0_M00_AXIS/M_AXIS] [get_bd_intf_pins axi_dma_0/S_AXIS_S2MM]
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXIS_MM2S] [get_bd_intf_pins ai_engine_0/S00_AXIS]
connect_bd_net [get_bd_pins axi_dma_0/s_axi_lite_aclk] [get_bd_pins clk_wiz/clk_out1]
connect_bd_net [get_bd_pins axi_dma_0/m_axi_mm2s_aclk] [get_bd_pins clk_wiz/clk_out1]
connect_bd_net [get_bd_pins axi_dma_0/m_axi_s2mm_aclk] [get_bd_pins clk_wiz/clk_out1]
connect_bd_net [get_bd_pins axi_dma_0/axi_resetn] [get_bd_pins rst_clk_wiz_250M/peripheral_aresetn]

# Assign addresses in the memory map
assign_bd_address

# Save, validate, and exit. Have to save to the hardware platform repo as that is what is used in the next steps
save_bd_design
validate_bd_design
save_bd_design


