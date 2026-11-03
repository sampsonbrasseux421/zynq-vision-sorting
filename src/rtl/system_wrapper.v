//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2023.1 (win64) Build 3865809 Sun May  7 15:05:29 MDT 2023
//Date        : Tue Oct 27 23:16:36 2026
//Host        : DESKTOP-QNIE7DD running 64-bit major release  (build 9200)
//Command     : generate_target system_wrapper.bd
//Design      : system_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module system_wrapper
   (DDR_addr,
    DDR_ba,
    DDR_cas_n,
    DDR_ck_n,
    DDR_ck_p,
    DDR_cke,
    DDR_cs_n,
    DDR_dm,
    DDR_dq,
    DDR_dqs_n,
    DDR_dqs_p,
    DDR_odt,
    DDR_ras_n,
    DDR_reset_n,
    DDR_we_n,
    FIXED_IO_ddr_vrn,
    FIXED_IO_ddr_vrp,
    FIXED_IO_mio,
    FIXED_IO_ps_clk,
    FIXED_IO_ps_porb,
    FIXED_IO_ps_srstb,
    a1_in,
    a2_in,
    a3_in,
    a4_in,
    clk_out2_0,
    color_out,
    peripheral_reset_0,
    pixel_clk,
    serial_clk,
    uart_0_rxd,
    uart_0_txd,
    vid_active_video_0,
    vid_data,
    vid_hsync_0,
    vid_io_out_0_field,
    vid_io_out_0_hblank,
    vid_io_out_0_vblank,
    vid_vsync,
    x_out,
    y_out,
    z_out);
  inout [14:0]DDR_addr;
  inout [2:0]DDR_ba;
  inout DDR_cas_n;
  inout DDR_ck_n;
  inout DDR_ck_p;
  inout DDR_cke;
  inout DDR_cs_n;
  inout [3:0]DDR_dm;
  inout [31:0]DDR_dq;
  inout [3:0]DDR_dqs_n;
  inout [3:0]DDR_dqs_p;
  inout DDR_odt;
  inout DDR_ras_n;
  inout DDR_reset_n;
  inout DDR_we_n;
  inout FIXED_IO_ddr_vrn;
  inout FIXED_IO_ddr_vrp;
  inout [53:0]FIXED_IO_mio;
  inout FIXED_IO_ps_clk;
  inout FIXED_IO_ps_porb;
  inout FIXED_IO_ps_srstb;
  input [31:0]a1_in;
  input [31:0]a2_in;
  input [31:0]a3_in;
  input [31:0]a4_in;
  output clk_out2_0;
  output [31:0]color_out;
  output [0:0]peripheral_reset_0;
  output pixel_clk;
  output serial_clk;
  input uart_0_rxd;
  output uart_0_txd;
  output vid_active_video_0;
  output [23:0]vid_data;
  output vid_hsync_0;
  output vid_io_out_0_field;
  output vid_io_out_0_hblank;
  output vid_io_out_0_vblank;
  output vid_vsync;
  output [31:0]x_out;
  output [31:0]y_out;
  output [31:0]z_out;

  wire [14:0]DDR_addr;
  wire [2:0]DDR_ba;
  wire DDR_cas_n;
  wire DDR_ck_n;
  wire DDR_ck_p;
  wire DDR_cke;
  wire DDR_cs_n;
  wire [3:0]DDR_dm;
  wire [31:0]DDR_dq;
  wire [3:0]DDR_dqs_n;
  wire [3:0]DDR_dqs_p;
  wire DDR_odt;
  wire DDR_ras_n;
  wire DDR_reset_n;
  wire DDR_we_n;
  wire FIXED_IO_ddr_vrn;
  wire FIXED_IO_ddr_vrp;
  wire [53:0]FIXED_IO_mio;
  wire FIXED_IO_ps_clk;
  wire FIXED_IO_ps_porb;
  wire FIXED_IO_ps_srstb;
  wire [31:0]a1_in;
  wire [31:0]a2_in;
  wire [31:0]a3_in;
  wire [31:0]a4_in;
  wire clk_out2_0;
  wire [31:0]color_out;
  wire [0:0]peripheral_reset_0;
  wire pixel_clk;
  wire serial_clk;
  wire uart_0_rxd;
  wire uart_0_txd;
  wire vid_active_video_0;
  wire [23:0]vid_data;
  wire vid_hsync_0;
  wire vid_io_out_0_field;
  wire vid_io_out_0_hblank;
  wire vid_io_out_0_vblank;
  wire vid_vsync;
  wire [31:0]x_out;
  wire [31:0]y_out;
  wire [31:0]z_out;

  system system_i
       (.DDR_addr(DDR_addr),
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
        .a1_in(a1_in),
        .a2_in(a2_in),
        .a3_in(a3_in),
        .a4_in(a4_in),
        .clk_out2_0(clk_out2_0),
        .color_out(color_out),
        .peripheral_reset_0(peripheral_reset_0),
        .pixel_clk(pixel_clk),
        .serial_clk(serial_clk),
        .uart_0_rxd(uart_0_rxd),
        .uart_0_txd(uart_0_txd),
        .vid_active_video_0(vid_active_video_0),
        .vid_data(vid_data),
        .vid_hsync_0(vid_hsync_0),
        .vid_io_out_0_field(vid_io_out_0_field),
        .vid_io_out_0_hblank(vid_io_out_0_hblank),
        .vid_io_out_0_vblank(vid_io_out_0_vblank),
        .vid_vsync(vid_vsync),
        .x_out(x_out),
        .y_out(y_out),
        .z_out(z_out));
endmodule
