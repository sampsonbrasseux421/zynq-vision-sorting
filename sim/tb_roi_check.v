`timescale 1ns/1ps
//=============================================================================
// tb_roi_check.v —— 用真实尺寸 1024x600 验证 ROI 是否生效
//   画面：全屏彩条（红/绿/蓝/黑）+ 一个集中在 ROI 内的小方块
//   期望：所有颜色的包围盒都落在 ROI (x120..1016, y200..450) 之内
//=============================================================================
module tb_roi_check;

    localparam integer W = 1024;
    localparam integer H = 600;

    reg clk = 0, rst_n = 0;
    reg        s_tvalid = 0, s_tuser = 0, s_tlast = 0;
    reg [23:0] s_tdata = 0;
    wire       s_tready;
    wire        m_tvalid, m_tuser, m_tlast;
    wire [23:0] m_tdata;
    reg         m_tready = 1;
    reg  [7:0]  rd_sel = 0;
    wire [31:0] rd_data;
    wire [31:0] frame_cnt;
    wire        done_toggle;

    color_blob_detector_roi dut (
        .aclk(clk), .aresetn(rst_n),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .s_axis_tlast(s_tlast), .s_axis_tuser(s_tuser),
        .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready),
        .m_axis_tlast(m_tlast), .m_axis_tuser(m_tuser),
        .rd_sel(rd_sel), .rd_data(rd_data),
        .frame_cnt(frame_cnt), .done_toggle(done_toggle)
    );

    always #5 clk = ~clk;

    // 全屏彩条：y<150 绿，150..300 红，300..450 蓝，其余黑
    function [23:0] pix_of;
        input integer x, y;
        begin
            if (y < 150)            pix_of = 24'h20C020;   // 绿
            else if (y < 300)       pix_of = 24'hD02820;   // 红
            else if (y < 450)       pix_of = 24'h2050D8;   // 蓝
            else                    pix_of = 24'h181818;   // 黑
        end
    endfunction

    integer x, y, i;
    reg [23:0] px;

    task read_field;
        input [2:0] col, fld;
        output [31:0] val;
        begin
            @(negedge clk); rd_sel = {2'b00, fld, col}; #1; val = rd_data;
        end
    endtask

    reg [31:0] cnt, x0, x1, y0, y1;
    integer fail = 0;

    task check_color;
        input [2:0] cid;
        input [127:0] nm;
        begin
            read_field(cid, 3'd0, cnt);
            read_field(cid, 3'd3, x0);
            read_field(cid, 3'd4, x1);
            read_field(cid, 3'd5, y0);
            read_field(cid, 3'd6, y1);
            if (cnt == 0) begin
                $display("  %0s: cnt=0 (ROI 内无此色)", nm);
            end else begin
                $display("  %0s: cnt=%0d bbox=(%0d..%0d, %0d..%0d)", nm, cnt, x0, x1, y0, y1);
                if (x0 < 120 || x1 > 1016 || y0 < 200 || y1 > 450) begin
                    $display("      [FAIL] 超出 ROI !");
                    fail = fail + 1;
                end
            end
        end
    endtask

    initial begin
        rst_n = 0;
        repeat(5) @(negedge clk);
        rst_n = 1;
        repeat(3) @(negedge clk);

        // 送两帧
        for (i = 0; i < 2; i = i + 1) begin
            for (y = 0; y < H; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    px = pix_of(x, y);
                    @(negedge clk);
                    s_tvalid = 1; s_tdata = px;
                    s_tuser = (x == 0 && y == 0);
                    s_tlast = (x == W - 1);
                    @(posedge clk); #1;
                end
            end
            @(negedge clk); s_tvalid = 0; s_tuser = 0; s_tlast = 0;
            @(posedge clk);
        end
        repeat(3) @(negedge clk);

        $display("");
        $display("==== ROI 验证（期望 bbox 全部落在 x120..1016, y200..450）====");
        check_color(3'd0, "red  ");
        check_color(3'd2, "green");
        check_color(3'd3, "blue ");
        check_color(3'd4, "black");
        $display("");
        if (fail == 0) $display("===== ROI 生效 =====");
        else           $display("===== ROI 未生效，失败 %0d 项 =====", fail);
        $finish;
    end
endmodule
