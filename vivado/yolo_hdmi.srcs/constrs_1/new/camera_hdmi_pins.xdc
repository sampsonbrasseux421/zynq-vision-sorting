set_property PACKAGE_PIN M19 [get_ports hdmi_clk_p]
set_property PACKAGE_PIN M20 [get_ports hdmi_clk_n]

set_property PACKAGE_PIN L19 [get_ports {hdmi_data_p[0]}]
set_property PACKAGE_PIN L20 [get_ports {hdmi_data_n[0]}]
set_property PACKAGE_PIN J20 [get_ports {hdmi_data_p[1]}]
set_property PACKAGE_PIN H20 [get_ports {hdmi_data_n[1]}]
set_property PACKAGE_PIN G19 [get_ports {hdmi_data_p[2]}]
set_property PACKAGE_PIN G20 [get_ports {hdmi_data_n[2]}]

set_property IOSTANDARD TMDS_33 [get_ports {
    hdmi_clk_p
    hdmi_clk_n
    hdmi_data_p[*]
    hdmi_data_n[*]
}]

# ---- ZDT stepper bus UART (115200 8N1, 3.3 V TTL -- NOT RS485) ----
# FPGA TX (K14) -> driver RX ; FPGA RX (J14) -> driver TX ; GND must be common
# The UART itself is the PL command engine (zdt_top8), not an axi_uartlite.
set_property PACKAGE_PIN K14 [get_ports uart_tx]
set_property PACKAGE_PIN J14 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports {uart_tx uart_rx}]

set_property PACKAGE_PIN N15 [get_ports servo_pwm]
set_property IOSTANDARD LVCMOS33 [get_ports servo_pwm]
