# =============================================================================
# zdt_pins.xdc -- 并入 OV5640 工程后新增的引脚约束
#
# 追加到 conqueror.xdc 末尾即可（或作为独立约束文件加入 constrs_1）。
# 引脚与机械臂工程完全一致：K14 = UART TX，J14 = UART RX，N15 = 舵机 PWM。
# =============================================================================

# ---- ZDT 步进驱动器串口（115200 8N1，3.3 V TTL，不是 RS485）----
# FPGA TX (K14) -> 驱动器 RX ; FPGA RX (J14) -> 驱动器 TX ; 必须共地
set_property -dict {PACKAGE_PIN K14 IOSTANDARD LVCMOS33} [get_ports zdt_tx_out]
set_property -dict {PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports uart_sel_out]

# ---- 夹爪舵机 PWM（50 Hz）----
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports servo_pwm_out]
