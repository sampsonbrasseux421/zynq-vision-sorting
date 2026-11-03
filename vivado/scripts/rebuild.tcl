# rebuild.tcl -- recreate the design and produce a bitstream
#
# Portable version of scripts/build_skipen.tcl: every path is resolved relative
# to the repository root, so it runs unchanged on any machine.
#
# Usage, from the repository root:
#     vivado -mode batch -source vivado/scripts/rebuild.tcl
#
# What it does
#   1. opens vivado/yolo_hdmi.xpr
#   2. refreshes the custom zdt_top8 module-reference cell in the block design
#   3. regenerates the wrapper and the IP output products
#   4. runs synthesis and implementation, writes the bitstream
#
# Note on module references
#   Vivado's module_ref IP is cached by module NAME. Editing the RTL of
#   zdt_top8.v in place does not refresh the cached IP, which is why the
#   original flow renamed the module each time (zdt_top7 -> zdt_top8). This
#   script deletes and re-creates the BD cell instead, so renames are no longer
#   needed.

set script_dir [file normalize [file dirname [info script]]]
set vivado_dir [file normalize [file join $script_dir ..]]
set repo_root  [file normalize [file join $vivado_dir ..]]
set xpr        [file join $vivado_dir yolo_hdmi.xpr]

if {![file exists $xpr]} {
    error "project not found: $xpr"
}
puts "repository root : $repo_root"
puts "project         : $xpr"

open_project $xpr

# ---------------------------------------------------------------- sources
# The RTL under src/rtl is the readable copy; the project keeps its own copy
# under yolo_hdmi.srcs. Add the repository copy so edits there take effect.
foreach f {zdt_top8.v zdt_tx_engine.v zdt_axil.v zdt_frame_tx.v
           video_osd_text21.v} {
    set p [file join $repo_root src rtl $f]
    if {[file exists $p]} {
        add_files -norecurse -fileset sources_1 $p
    }
}
update_compile_order -fileset sources_1

# ------------------------------------------------------------ block design
open_bd_design [get_files system.bd]

# drop any stale copy of the command-engine cell, then re-create it
foreach cell [get_bd_cells -quiet -hier -filter {VLNV =~ "*zdt_top*"}] {
    puts "removing stale BD cell $cell"
    delete_bd_objs $cell
}

create_bd_cell -type module -reference zdt_top8 axi_zdt_2
connect_bd_intf_net [get_bd_intf_pins axi_zdt_2/S_AXI]  [get_bd_intf_pins ps7_0_axi_periph/M08_AXI]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_zdt_2/aclk] [get_bd_pins ps7_0_axi_periph/M08_ACLK]
connect_bd_net [get_bd_pins rst_ps7_0_50M/peripheral_aresetn] \
    [get_bd_pins axi_zdt_2/aresetn] [get_bd_pins ps7_0_axi_periph/M08_ARESETN]
connect_bd_net [get_bd_ports /zdt_tx_out]   [get_bd_pins axi_zdt_2/tx]
connect_bd_net [get_bd_ports /uart_sel_out] [get_bd_pins axi_zdt_2/uart_sel]
puts "ZDT cell re-created"

assign_bd_address
validate_bd_design
save_bd_design

generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top -import
set_property source_mgmt_mode All [current_project]
update_compile_order -fileset sources_1
set_property top camera_hdmi_top [current_fileset]

# ------------------------------------------------------------------- build
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "SYNTHESIS FAILED -- see yolo_hdmi.runs/synth_1/runme.log"
}
puts "SYNTH_OK"

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    error "IMPLEMENTATION FAILED -- see yolo_hdmi.runs/impl_1/runme.log"
}
puts "BUILD_DONE"

# ------------------------------------------------------------- the reports
open_run impl_1
report_utilization    -file [file join $repo_root build utilization.rpt]
report_timing_summary -file [file join $repo_root build timing_summary.rpt] \
                      -delay_type max -max_paths 10
report_power          -file [file join $repo_root build power.rpt]
write_hw_platform     -fixed -include_bit -force \
                      -file [file join $repo_root build system.xsa]

puts "REPORTS_WRITTEN"
puts "WNS = [get_property SLACK [get_timing_paths -delay_type max]] ns"

# ---------------------------------------------------------------- addresses
foreach s [get_bd_addr_segs -quiet] {
    if {[string match "*zdt*" [get_property NAME $s]]} {
        puts "ADDR: [get_property NAME $s]"
    }
}
