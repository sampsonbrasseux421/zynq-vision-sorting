set proj D:/pl_zdt_project/yolo_hdmi.xpr
open_project $proj
add_files -norecurse -fileset sources_1 D:/pl_detector/rtl/zdt_top8.v
update_compile_order -fileset sources_1
open_bd_design [get_files system.bd]
if {[llength [get_bd_cells -quiet /axi_zdt_1]] > 0} { catch { delete_bd_objs [get_bd_cells /axi_zdt_1] } }
create_bd_cell -type module -reference zdt_top8 axi_zdt_2
connect_bd_intf_net [get_bd_intf_pins axi_zdt_2/S_AXI] [get_bd_intf_pins ps7_0_axi_periph/M08_AXI]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins axi_zdt_2/aclk] [get_bd_pins ps7_0_axi_periph/M08_ACLK]
connect_bd_net [get_bd_pins rst_ps7_0_50M/peripheral_aresetn] [get_bd_pins axi_zdt_2/aresetn] [get_bd_pins ps7_0_axi_periph/M08_ARESETN]
connect_bd_net [get_bd_ports /zdt_tx_out]   [get_bd_pins axi_zdt_2/tx]
connect_bd_net [get_bd_ports /uart_sel_out] [get_bd_pins axi_zdt_2/uart_sel]
puts "ZDT8_OK"
assign_bd_address
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top -import
set_property source_mgmt_mode All [current_project]
update_compile_order -fileset sources_1
set_property top camera_hdmi_top [current_fileset]
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} { error SYNTH_FAILED }
puts "SYNTH_OK"
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} { error IMPL_FAILED }
puts "BUILD_DONE"
foreach s [get_bd_addr_segs -quiet] {
    set nm [get_property NAME $s]
    if {[string match "*zdt*" $nm]} { puts "ADDR: $nm" }
}