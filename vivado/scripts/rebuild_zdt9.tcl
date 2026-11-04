# =============================================================================
# rebuild_zdt9.tcl -- 用 zdt_top9 替换 BD 里的 zdt_top8，强制重新生成 IP 定义
#
# 用法（在 yolo_hdmi 工程里）：
#     source D:/share/openrepo/vivado/scripts/rebuild_zdt9.tcl
#
# 做四件事：
#   1. 加入 zdt_top9.v 与 ik_solver.v
#   2. 删掉旧 /axi_zdt_1，用 zdt_top9 重建（同名）
#   3. 重连 AXI / 时钟 / 复位 / 输出端口，重新分配地址
#   4. 校验 + 保存
# =============================================================================

set rtl_dir {D:/share/openrepo/src/rtl}

open_bd_design [get_files system.bd]

# ---------------------------------------------------------------- 1. 源文件
foreach f {zdt_top9.v ik_solver.v zdt_axil.v zdt_tx_engine.v zdt_frame_tx.v} {
    set p [file join $rtl_dir $f]
    if {![file exists $p]} { error "missing RTL: $p" }
    if {[llength [get_files -quiet $p]] == 0} {
        add_files -norecurse -fileset sources_1 $p
        puts "added $f"
    }
}

# ---------------------------------------------------------------- 2. 记录旧连接
set intercon [get_bd_cells -quiet /ps7_0_axi_periph]
if {$intercon eq ""} { error "ps7_0_axi_periph not found" }

# 找到 zdt 当前挂在哪个 M 口（有连接的）
set old_port ""
foreach pin [get_bd_intf_pins -of_objects $intercon] {
    if {![string match "*M*_AXI" [get_property NAME $pin]]} { continue }
    set nets [get_bd_intf_nets -quiet -of_objects $pin]
    if {[llength $nets] > 0} {
        set far [get_bd_intf_pins -quiet -of_objects $nets]
        foreach fp $far {
            if {[string match "*axi_zdt_1*" $fp]} { set old_port $pin }
        }
    }
}
if {$old_port ne ""} {
    set idx [string range [get_property NAME $old_port] 1 2]
    puts "旧连接在 $old_port (idx=$idx)"
} else {
    # 没连过就用第一个空口
    foreach pin [get_bd_intf_pins -of_objects $intercon] {
        if {![string match "*M*_AXI" [get_property NAME $pin]]} { continue }
        if {[llength [get_bd_intf_nets -quiet -of_objects $pin]] == 0} {
            set old_port $pin
            set idx [string range [get_property NAME $pin] 1 2]
            break
        }
    }
    if {$old_port eq ""} {
        set n [get_property CONFIG.NUM_MI $intercon]
        set_property -dict [list CONFIG.NUM_MI [expr {$n + 1}]] $intercon
        set old_port [get_bd_intf_pins -of_objects $intercon -filter "NAME =~ \"M[format %02d $n]_AXI\""]
        set idx [string range [get_property NAME $old_port] 1 2]
    }
    puts "使用空闲口 $old_port"
}

# ---------------------------------------------------------------- 3. 删除旧 cell
if {[llength [get_bd_cells -quiet /axi_zdt_1]]} {
    delete_bd_objs [get_bd_cells /axi_zdt_1]
    puts "已删除旧 /axi_zdt_1"
}

# ---------------------------------------------------------------- 4. 重建
create_bd_cell -type module -reference zdt_top9 axi_zdt_1
set_property -dict [list CONFIG.CLK_HZ {50000000} CONFIG.BAUD {115200}] \
    [get_bd_cells /axi_zdt_1]
puts "已用 zdt_top9 创建 /axi_zdt_1"

# AXI
connect_bd_intf_net $old_port [get_bd_intf_pins /axi_zdt_1/S_AXI]

# 时钟 / 复位
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins /axi_zdt_1/aclk] [get_bd_pins $intercon/M${idx}_ACLK]
connect_bd_net [get_bd_pins rst_ps7_0_100M/peripheral_aresetn] \
    [get_bd_pins /axi_zdt_1/aresetn] [get_bd_pins $intercon/M${idx}_ARESETN]

# 顶层端口（可能已存在）
foreach {port pin dir} {zdt_tx_out tx O uart_sel_out uart_sel O} {
    if {[llength [get_bd_ports -quiet /$port]] == 0} {
        create_bd_port -dir $dir $port
    }
    if {[llength [get_bd_nets -quiet -of_objects [get_bd_ports /$port]]] == 0} {
        connect_bd_net [get_bd_ports /$port] [get_bd_pins /axi_zdt_1/$pin]
    }
}

# ---------------------------------------------------------------- 5. 收尾
assign_bd_address -quiet
validate_bd_design
save_bd_design

generate_target all [get_files system.bd]
make_wrapper -files [get_files system.bd] -top -import -force
update_compile_order -fileset sources_1

puts ""
puts "=== 地址 ==="
foreach s [get_bd_addr_segs -quiet] {
    set nm [get_property NAME $s]
    if {[string match "*zdt*" $nm]} {
        puts [format "  %-30s %s" $nm [get_property OFFSET $s]]
    }
}
puts "=== 完成，下一步：reset_run synth_1 / impl_1 后构建 ==="
