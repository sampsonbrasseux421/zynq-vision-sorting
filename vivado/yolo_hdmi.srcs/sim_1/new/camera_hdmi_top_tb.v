`timescale 1ns / 1ps

module camera_hdmi_top_tb;
    wire hdmi_clk_p;
    wire hdmi_clk_n;
    wire [2:0] hdmi_data_p;
    wire [2:0] hdmi_data_n;

    camera_hdmi_top dut (
        .DDR_addr(), .DDR_ba(), .DDR_cas_n(), .DDR_ck_n(), .DDR_ck_p(),
        .DDR_cke(), .DDR_cs_n(), .DDR_dm(), .DDR_dq(), .DDR_dqs_n(),
        .DDR_dqs_p(), .DDR_odt(), .DDR_ras_n(), .DDR_reset_n(), .DDR_we_n(),
        .FIXED_IO_ddr_vrn(), .FIXED_IO_ddr_vrp(), .FIXED_IO_mio(),
        .FIXED_IO_ps_clk(), .FIXED_IO_ps_porb(), .FIXED_IO_ps_srstb(),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n),
        .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n)
    );

    task check_rgb(input [23:0] rgb, input [2:0] expected);
        begin
            dut.system_inst.vid_io_out_0_data = rgb;
            dut.system_inst.vid_active_video_0 = 1'b1;
            #1;
            if (hdmi_data_p !== expected)
                $fatal(1, "RGB lane mapping mismatch: rgb=%h got=%b expected=%b",
                       rgb, hdmi_data_p, expected);
        end
    endtask

    initial begin
        #1;
        check_rgb(24'h000001, 3'b001);
        check_rgb(24'h000100, 3'b010);
        check_rgb(24'h010000, 3'b100);

        dut.system_inst.vid_active_video_0 = 1'b0;
        dut.system_inst.vid_hsync_0 = 1'b1;
        dut.system_inst.vid_io_out_0_vsync = 1'b0;
        #1;
        if (hdmi_data_p[0] !== 1'b1)
            $fatal(1, "HSYNC was not sent in the blue control lane");

        dut.system_inst.vid_hsync_0 = 1'b0;
        dut.system_inst.vid_io_out_0_vsync = 1'b1;
        #1;
        if (hdmi_data_p[0] !== 1'b0)
            $fatal(1, "VSYNC/HSYNC control ordering is incorrect");

        dut.system_inst.peripheral_reset_0 = 1'b1;
        dut.system_inst.vid_active_video_0 = 1'b1;
        dut.system_inst.vid_io_out_0_data = 24'hffffff;
        #1;
        if (hdmi_data_p !== 3'b000)
            $fatal(1, "Active-high peripheral reset did not reset the encoders");

        $display("PASS: camera HDMI RGB, sync, and reset connections");
        $finish;
    end
endmodule

module system_wrapper (
    inout [14:0] DDR_addr,
    inout [2:0] DDR_ba,
    inout DDR_cas_n,
    inout DDR_ck_n,
    inout DDR_ck_p,
    inout DDR_cke,
    inout DDR_cs_n,
    inout [3:0] DDR_dm,
    inout [31:0] DDR_dq,
    inout [3:0] DDR_dqs_n,
    inout [3:0] DDR_dqs_p,
    inout DDR_odt,
    inout DDR_ras_n,
    inout DDR_reset_n,
    inout DDR_we_n,
    inout FIXED_IO_ddr_vrn,
    inout FIXED_IO_ddr_vrp,
    inout [53:0] FIXED_IO_mio,
    inout FIXED_IO_ps_clk,
    inout FIXED_IO_ps_porb,
    inout FIXED_IO_ps_srstb,
    output wire clk_out2_0,
    output reg [0:0] peripheral_reset_0,
    output wire pixel_clk,
    output wire serial_clk,
    output reg vid_active_video_0,
    output reg vid_hsync_0,
    output reg [23:0] vid_io_out_0_data,
    output wire vid_io_out_0_field,
    output wire vid_io_out_0_hblank,
    output wire vid_io_out_0_vblank,
    output reg vid_io_out_0_vsync
);
    initial begin
        peripheral_reset_0 = 1'b0;
        vid_active_video_0 = 1'b1;
        vid_hsync_0 = 1'b0;
        vid_io_out_0_data = 24'h000000;
        vid_io_out_0_vsync = 1'b0;
    end
    assign clk_out2_0 = 1'b0;
    assign pixel_clk = 1'b0;
    assign serial_clk = 1'b0;
    assign vid_io_out_0_field = 1'b0;
    assign vid_io_out_0_hblank = 1'b0;
    assign vid_io_out_0_vblank = 1'b0;
endmodule

module tmds_encoder (
    input wire clk,
    input wire rst,
    input wire de,
    input wire [1:0] control,
    input wire [7:0] data,
    output reg [9:0] symbol
);
    always @* begin
        symbol = 10'b0;
        if (!rst) begin
            if (de)
                symbol[0] = data[0];
            else
                symbol[0] = control[0] & ~control[1];
        end
    end
endmodule

module tmds_serializer (
    input wire clk_pixel,
    input wire clk_5x,
    input wire rst,
    input wire [9:0] data10,
    output wire out_p,
    output wire out_n
);
    assign out_p = data10[0];
    assign out_n = ~data10[0];
endmodule
