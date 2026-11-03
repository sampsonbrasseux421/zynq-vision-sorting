set proj D:/pl_zdt_project/yolo_hdmi.xpr
open_project $proj

set bad {}
foreach f [get_files -quiet *ikrtl*] { lappend bad $f }
foreach f [get_files -quiet *ikip*] { lappend bad $f }
foreach f [get_files -quiet *cordic*] { lappend bad $f }
foreach f [get_files -quiet *sqrt_32*] { lappend bad $f }
foreach f [get_files -quiet *div_gen*] { lappend bad $f }
foreach f [get_files -quiet *ip_Multiplier*] { lappend bad $f }
foreach f [get_files -quiet *fifo_input*] { lappend bad $f }
foreach f [get_files -quiet *fifo_output*] { lappend bad $f }
puts "REMOVING: [llength $bad]"
foreach f $bad { catch { remove_files $f } }

add_files -norecurse -fileset sources_1 {
    D:/pl_detector/rtl/zdt_frame_tx.v
    D:/pl_detector/rtl/zdt_tx_engine.v
    D:/pl_detector/rtl/zdt_axil.v
    D:/pl_detector/rtl/zdt_top.v
}
update_compile_order -fileset sources_1
set_property top camera_hdmi_top [current_fileset]
puts "TOP: [get_property top [current_fileset]]"

open_bd_design [get_files system.bd]
set_property -dict [list CONFIG.NUM_MI {9}] [get_bd_cells ps7_0_axi_periph]
if {[llength [get_bd_cells -quiet /axi_zdt_0]] > 0} { catch { delete_bd_objs [get_bd_cells /axi_zdt_0] } }
create_bd_cell -type module -reference zdt_top axi_zdt_0
connect_bd_intf_net [get_bd_intf_pins axi_zdt_0/S_AXI] [get_bd_intf_pins ps7_0_axi_periph/M08_AXI]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins axi_zdt_0/aclk] [get_bd_pins ps7_0_axi_periph/M08_ACLK]
connect_bd_net [get_bd_pins rst_ps7_0_50M/peripheral_aresetn] [get_bd_pins axi_zdt_0/aresetn] [get_bd_pins ps7_0_axi_periph/M08_ARESETN]
if {[llength [get_bd_ports -quiet /zdt_tx_out]] == 0} {
    create_bd_port -dir O zdt_tx_out
}
connect_bd_net [get_bd_ports /zdt_tx_out] [get_bd_pins axi_zdt_0/tx]
puts "BD_WIRED"
assign_bd_address
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
puts "TARGET_OK"
foreach s [get_bd_addr_segs -quiet] {
    if {[string match "*zdt*" [get_property NAME $s]]} { puts "ADDR: $s" }
}
puts "DONE"