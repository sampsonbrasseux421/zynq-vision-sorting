# Recreate the ZDT module reference with an 8-bit AXI-Lite local address.
# Requires the Vivado project to be open before sourcing this script.

set rtl_dir {D:/share/openrepo/src/rtl}
open_bd_design [get_files system.bd]

foreach f {zdt_top21.v zdt_axil.v ik_solver.v zdt_tx_engine.v zdt_frame_tx.v zdt_interp.v} {
    set p [file join $rtl_dir $f]
    if {![file exists $p]} { error "missing RTL: $p" }
    if {[llength [get_files -quiet $p]] == 0} {
        add_files -norecurse -fileset sources_1 $p
        puts "  added $f"
    }
}

foreach c [get_bd_cells] {
    if {[string match "*zdt*" [get_property NAME $c]]} {
        puts "  removing [get_property NAME $c]"
        catch { delete_bd_objs $c }
    }
}

set intercon [get_bd_cells /ps7_0_axi_periph]
set freep ""
foreach pin [get_bd_intf_pins -of_objects $intercon] {
    if {![string match "*M*_AXI" [get_property NAME $pin]]} { continue }
    if {[llength [get_bd_intf_nets -quiet -of_objects $pin]] == 0} {
        set freep $pin
        break
    }
}
if {$freep eq ""} {
    set n [get_property CONFIG.NUM_MI $intercon]
    set_property -dict [list CONFIG.NUM_MI [expr {$n + 1}]] $intercon
    set freep [get_bd_intf_pins -of_objects $intercon -filter "NAME =~ \"M[format %02d $n]_AXI\""]
}
set idx [string range [get_property NAME $freep] 1 2]
puts "  using $freep (idx=$idx)"

create_bd_cell -type module -reference zdt_top21 axi_zdt_21
set_property -dict [list CONFIG.CLK_HZ {50000000} CONFIG.BAUD {115200}] [get_bd_cells /axi_zdt_21]
connect_bd_intf_net $freep [get_bd_intf_pins /axi_zdt_21/S_AXI]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins /axi_zdt_21/aclk] [get_bd_pins $intercon/M${idx}_ACLK]
connect_bd_net [get_bd_pins rst_ps7_0_50M/peripheral_aresetn] \
    [get_bd_pins /axi_zdt_21/aresetn] [get_bd_pins $intercon/M${idx}_ARESETN]

foreach {port pin dir} {zdt_tx_out tx O uart_sel_out uart_sel O} {
    if {[llength [get_bd_ports -quiet /$port]] == 0} { create_bd_port -dir $dir $port }
    foreach n [get_bd_nets -quiet -of_objects [get_bd_ports /$port]] { catch { delete_bd_objs $n } }
    connect_bd_net [get_bd_ports /$port] [get_bd_pins /axi_zdt_21/$pin]
}

assign_bd_address -quiet
catch {
    assign_bd_address -offset 0x40000000 -range 64K \
        -target_address_space [get_bd_addr_spaces processing_system7_0/Data] \
        [get_bd_addr_segs /axi_zdt_21/S_AXI/Reg]
}

validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top -import -force

foreach s [get_bd_addr_segs -quiet] {
    if {[string match "*zdt*" [get_property NAME $s]]} {
        puts "  address [get_property NAME $s] = [get_property OFFSET $s]"
    }
}
