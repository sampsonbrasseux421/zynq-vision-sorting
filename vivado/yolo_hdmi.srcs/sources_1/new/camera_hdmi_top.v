`timescale 1ns / 1ps

module camera_hdmi_top (
    inout  wire [14:0] DDR_addr,
    inout  wire [2:0]  DDR_ba,
    inout  wire        DDR_cas_n,
    inout  wire        DDR_ck_n,
    inout  wire        DDR_ck_p,
    inout  wire        DDR_cke,
    inout  wire        DDR_cs_n,
    inout  wire [3:0]  DDR_dm,
    inout  wire [31:0] DDR_dq,
    inout  wire [3:0]  DDR_dqs_n,
    inout  wire [3:0]  DDR_dqs_p,
    inout  wire        DDR_odt,
    inout  wire        DDR_ras_n,
    inout  wire        DDR_reset_n,
    inout  wire        DDR_we_n,
    inout  wire        FIXED_IO_ddr_vrn,
    inout  wire        FIXED_IO_ddr_vrp,
    inout  wire [53:0] FIXED_IO_mio,
    inout  wire        FIXED_IO_ps_clk,
    inout  wire        FIXED_IO_ps_porb,
    inout  wire        FIXED_IO_ps_srstb,
    output wire        hdmi_clk_p,
    output wire        hdmi_clk_n,
    output wire [2:0]  hdmi_data_p,
    output wire [2:0]  hdmi_data_n,


// ZDT stepper bus (axi_uartlite @ 0x42C00000)
    output wire        uart_tx,
    input  wire        uart_rx,

    // PL hardware PWM drives N15; Linux commands pulse width through AXI GPIO.
    output wire        servo_pwm
);

// uart_tx 由 PL 硬件 ZDT 引擎驱动（TTL 直连 K14）
wire zdt_tx_w;
wire uart_sel_w;      /* 0=软件 axi_uartlite  1=PL 硬件引擎 */
wire sys_uart_txd;

    wire pixel_clk;
    wire serial_clk;
    wire [0:0] peripheral_reset;
    wire rst = peripheral_reset[0];
    wire servo_clk;
    wire [0:0] servo_resetn;
    wire [31:0] servo_cmd;
    wire [31:0] servo_status;
    wire [15:0] servo_active_us;
    assign servo_status = {16'h5350, servo_active_us};

    servo_pwm #(.CLK_FREQ_HZ(50000000)) servo_hw (
        .clk(servo_clk), .rst_n(servo_resetn[0]),
        .pulse_us(servo_cmd[15:0]), .pwm(servo_pwm),
        .active_us(servo_active_us)
    );

    // IK register nets (between BD AXI GPIO and the IK core)
    wire [31:0] ik_x, ik_y, ik_z, ik_color;
    wire [31:0] ik_a1, ik_a2, ik_a3, ik_a4;
    wire active_video;
    wire hsync;
    wire vsync;
    wire [23:0] rgb;

    wire [9:0] tmds_blue;
    wire [9:0] tmds_green;
    wire [9:0] tmds_red;

    system_wrapper system_inst (
        .DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
        .clk_out2_0(),
        .peripheral_reset_0(peripheral_reset),
        .pixel_clk(pixel_clk),
        .servo_clk_50m(servo_clk),
        .servo_resetn_50m(servo_resetn),
        .servo_out(),
        .servo_cmd(servo_cmd),
        .servo_status(servo_status),
        .serial_clk(serial_clk),
        .vid_active_video_0(active_video),
        .vid_hsync_0(hsync),
        .vid_data(rgb),
        .vid_io_out_0_field(),
        .vid_io_out_0_hblank(),
        .vid_io_out_0_vblank(),
        .vid_vsync(vsync),
        .uart_0_rxd(uart_rx),
        .uart_0_txd(sys_uart_txd),   /* 软件 UART 接选择器 0 支路 */
        .zdt_tx_out(zdt_tx_w),
        .uart_sel_out(uart_sel_w),
        .x_out(ik_x),
        .y_out(ik_y),
        .z_out(ik_z),
        .color_out(ik_color),
        .a1_in(ik_a1),
        .a2_in(ik_a2),
        .a3_in(ik_a3),
        .a4_in(ik_a4)
    );

    tmds_encoder blue_encoder (
        .clk(pixel_clk),
        .rst(rst),
        .de(active_video),
        .control({vsync, hsync}),
        .data(rgb[7:0]),
        .symbol(tmds_blue)
    );

    tmds_encoder green_encoder (
        .clk(pixel_clk),
        .rst(rst),
        .de(active_video),
        .control(2'b00),
        .data(rgb[15:8]),
        .symbol(tmds_green)
    );

    tmds_encoder red_encoder (
        .clk(pixel_clk),
        .rst(rst),
        .de(active_video),
        .control(2'b00),
        .data(rgb[23:16]),
        .symbol(tmds_red)
    );

    tmds_serializer blue_serializer (
        .clk_pixel(pixel_clk),
        .clk_5x(serial_clk),
        .rst(rst),
        .data10(tmds_blue),
        .out_p(hdmi_data_p[0]),
        .out_n(hdmi_data_n[0])
    );

    tmds_serializer green_serializer (
        .clk_pixel(pixel_clk),
        .clk_5x(serial_clk),
        .rst(rst),
        .data10(tmds_green),
        .out_p(hdmi_data_p[1]),
        .out_n(hdmi_data_n[1])
    );

    tmds_serializer red_serializer (
        .clk_pixel(pixel_clk),
        .clk_5x(serial_clk),
        .rst(rst),
        .data10(tmds_red),
        .out_p(hdmi_data_p[2]),
        .out_n(hdmi_data_n[2])
    );

    tmds_serializer clock_serializer (
        .clk_pixel(pixel_clk),
        .clk_5x(serial_clk),
        .rst(rst),
        .data10(10'b1111100000),
        .out_p(hdmi_clk_p),
        .out_n(hdmi_clk_n)
    );

    //=========================================================================
    // Inverse kinematics: PS writes (x,y,z,color) through AXI GPIO, the core
    // computes the joint angles at 49.5 MHz, PS reads them back at 0x41220000.
    //   color bit31 rising edge = start one solve
    //   payload = {color[1:0], z[15:0], y[15:0], x[15:0]}
    //=========================================================================
    wire         ik_ready;
    reg          ik_valid;
    reg  [49:0]  ik_payload;
    reg          trig_d;
    wire         trig = ik_color[31] ^ trig_d;

    wire         m_valid;
    wire [127:0] m_data;
    reg  [31:0]  ang1, ang2, ang3, ang4;

    always @(posedge pixel_clk or posedge rst) begin
        if (rst) begin
            trig_d     <= 1'b0;
            ik_valid   <= 1'b0;
            ik_payload <= 50'd0;
            ang1 <= 32'd0; ang2 <= 32'd0; ang3 <= 32'd0; ang4 <= 32'd0;
        end else begin
            trig_d <= ik_color[31];

            if (trig) begin
                ik_payload <= {ik_color[1:0], ik_z[15:0], ik_y[15:0], ik_x[15:0]};
                ik_valid   <= 1'b1;
            end else if (ik_ready) begin
                ik_valid   <= 1'b0;
            end

            if (m_valid) begin
                ang1 <= m_data[31:0];
                ang2 <= m_data[63:32];
                ang3 <= m_data[95:64];
                ang4 <= m_data[127:96];
            end
        end
    end

    assign ik_a1 = ang1;
    assign ik_a2 = ang2;
    assign ik_a3 = ang3;
    assign ik_a4 = ang4;

    // IK core removed: does not fit in XC7Z020 (FIFO inferred as registers)
    assign m_valid = 1'b0;
    assign m_data  = 128'd0;



assign uart_tx = uart_sel_w ? zdt_tx_w : sys_uart_txd;

endmodule
