`timescale 1ns/1ps
//=============================================================================
// tb_color_blob_detector.v -- self-checking testbench
//   - 640x480 synthetic frame with 4 known color blobs
//   - checks statistics vs testbench-computed expectations
//   - checks pass-through channel is bit-identical
// run: iverilog -g2012 -o sim.out tb_color_blob_detector.v ../rtl/color_blob_detector.v
//=============================================================================
module tb_color_blob_detector;

    localparam integer W  = 640;
    localparam integer H  = 480;
    localparam integer HD = 4;
    localparam integer VD = 4;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    reg        s_tvalid = 1'b0;
    reg        s_tuser  = 1'b0;
    reg        s_tlast  = 1'b0;
    reg [23:0] s_tdata  = 24'd0;
    wire       s_tready;

    wire        m_tvalid, m_tuser, m_tlast;
    wire [23:0] m_tdata;
    reg         m_tready = 1'b1;

    reg  [7:0]  rd_sel  = 8'd0;
    wire [31:0] rd_data;
    wire [31:0] frame_cnt;
    wire        done_toggle;

    color_blob_detector #(.H_DIV(HD), .V_DIV(VD),
                         .ROI_X0(0), .ROI_X1(639),
                         .ROI_Y0(0), .ROI_Y1(479)) dut (
        .aclk(clk), .aresetn(rst_n),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .s_axis_tlast(s_tlast), .s_axis_tuser(s_tuser),
        .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready),
        .m_axis_tlast(m_tlast), .m_axis_tuser(m_tuser),
        .rd_sel(rd_sel), .rd_data(rd_data),
        .frame_cnt(frame_cnt), .done_toggle(done_toggle)
    );

    always #5 clk = ~clk;

    integer e_cnt [0:4];
    integer e_sx  [0:4];
    integer e_sy  [0:4];
    integer e_x0  [0:4];
    integer e_x1  [0:4];
    integer e_y0  [0:4];
    integer e_y1  [0:4];

    function [23:0] pix_of;
        input integer x, y;
        begin
            if (x >= 100 && x <= 199 && y >= 50  && y <= 149)      pix_of = 24'hD82820;
            else if (x >= 400 && x <= 499 && y >= 300 && y <= 399) pix_of = 24'h28C040;
            else if (x >= 500 && x <= 539 && y >= 100 && y <= 139) pix_of = 24'h181818;
            else if (x >=  40 && x <=  99 && y >= 400 && y <= 459) pix_of = 24'h2050D8;
            else                                                   pix_of = 24'hDCDCDC;
        end
    endfunction

    function [2:0] classify;
        input [7:0] R, G, B;
        reg [7:0] mx, mn, sat;
        begin
            mx  = (R > G) ? ((R > B) ? R : B) : ((G > B) ? G : B);
            mn  = (R < G) ? ((R < B) ? R : B) : ((G < B) ? G : B);
            sat = mx - mn;
            if (mx < 90 && sat < 40)                         classify = 3'd4;
            else if (R > G + 45 && R > B + 35)               classify = 3'd0;
            else if (R > 110 && G > 90 && B < 110 && R >= G) classify = 3'd1;
            else if (G > R + 25 && G > B + 20)               classify = 3'd2;
            else if (B > R + 35 && B > G + 15)               classify = 3'd3;
            else                                             classify = 3'd5;
        end
    endfunction

    task clear_expect;
        integer k;
        begin
            for (k = 0; k < 5; k = k + 1) begin
                e_cnt[k] = 0; e_sx[k] = 0; e_sy[k] = 0;
                e_x0[k] = 65535; e_x1[k] = 0;
                e_y0[k] = 65535; e_y1[k] = 0;
            end
        end
    endtask

    integer x, y;
    reg [23:0] px;
    reg [2:0]  c;
    integer    fwd_err = 0;

    task send_frame;
        begin
            clear_expect();
            for (y = 0; y < H; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    px = pix_of(x, y);
                    c  = classify(px[23:16], px[15:8], px[7:0]);
                    if (c != 3'd5 && (x % HD == 0) && (y % VD == 0)) begin
                        if (e_cnt[c] == 0) begin
                            e_x0[c] = x; e_x1[c] = x;
                            e_y0[c] = y; e_y1[c] = y;
                        end else begin
                            if (x < e_x0[c]) e_x0[c] = x;
                            if (x > e_x1[c]) e_x1[c] = x;
                            if (y < e_y0[c]) e_y0[c] = y;
                            if (y > e_y1[c]) e_y1[c] = y;
                        end
                        e_cnt[c] = e_cnt[c] + 1;
                        e_sx[c]  = e_sx[c] + x;
                        e_sy[c]  = e_sy[c] + y;
                    end

                    @(negedge clk);
                    s_tvalid = 1'b1;
                    s_tdata  = px;
                    s_tuser  = (x == 0 && y == 0);
                    s_tlast  = (x == W - 1);
                    @(posedge clk);
                    #1;
                    if (!(m_tvalid && m_tdata === px && m_tlast === s_tlast)) begin
                        if (fwd_err < 5)
                            $display("  [FAIL] passthrough x=%0d y=%0d m=%h exp=%h", x, y, m_tdata, px);
                        fwd_err = fwd_err + 1;
                    end
                end
            end
            @(negedge clk);
            s_tvalid = 1'b0;
            s_tuser  = 1'b0;
            s_tlast  = 1'b0;
            @(posedge clk);
        end
    endtask

    task read_field;
        input [2:0] col;
        input [2:0] fld;
        output [31:0] val;
        begin
            @(negedge clk);
            rd_sel = {2'b00, fld, col};
            #1;
            val = rd_data;
        end
    endtask

    integer    errs = 0;
    reg [31:0] got;

    task check_one;
        input [2:0]  col;
        input [2:0]  fld;
        input [31:0] exp;
        input [127:0] name;
        begin
            read_field(col, fld, got);
            if (got !== exp) begin
                $display("  [FAIL] %0s color=%0d field=%0d got=%0d exp=%0d", name, col, fld, got, exp);
                errs = errs + 1;
            end else begin
                $display("  [ ok ] %0s color=%0d field=%0d = %0d", name, col, fld, got);
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (3) @(negedge clk);

        $display("================ frame 1 ================");
        send_frame();
        $display("================ frame 2 ================");
        send_frame();
        repeat (3) @(negedge clk);

        $display("");
        $display("================ statistics check ================");
        check_one(3'd0, 3'd0, e_cnt[0], "red  cnt");
        check_one(3'd0, 3'd1, e_sx[0],  "red  sx ");
        check_one(3'd0, 3'd2, e_sy[0],  "red  sy ");
        check_one(3'd0, 3'd3, e_x0[0],  "red  x0 ");
        check_one(3'd0, 3'd4, e_x1[0],  "red  x1 ");
        check_one(3'd0, 3'd5, e_y0[0],  "red  y0 ");
        check_one(3'd0, 3'd6, e_y1[0],  "red  y1 ");
        check_one(3'd2, 3'd0, e_cnt[2], "grn  cnt");
        check_one(3'd2, 3'd1, e_sx[2],  "grn  sx ");
        check_one(3'd2, 3'd2, e_sy[2],  "grn  sy ");
        check_one(3'd2, 3'd3, e_x0[2],  "grn  x0 ");
        check_one(3'd2, 3'd4, e_x1[2],  "grn  x1 ");
        check_one(3'd4, 3'd0, e_cnt[4], "blk  cnt");
        check_one(3'd4, 3'd3, e_x0[4],  "blk  x0 ");
        check_one(3'd4, 3'd4, e_x1[4],  "blk  x1 ");
        check_one(3'd3, 3'd0, e_cnt[3], "blue cnt");
        check_one(3'd3, 3'd5, e_y0[3],  "blue y0 ");
        check_one(3'd3, 3'd6, e_y1[3],  "blue y1 ");
        check_one(3'd1, 3'd0, 0,        "yel  cnt");

        $display("");
        $display("frame_cnt = %0d (expect 2)", frame_cnt);
        if (frame_cnt !== 32'd2) begin
            $display("  [FAIL] frame_cnt");
            errs = errs + 1;
        end

        $display("passthrough mismatches = %0d", fwd_err);
        if (fwd_err != 0) errs = errs + 1;

        $display("");
        if (errs == 0)
            $display("===== ALL PASS (stats ok + passthrough ok) =====");
        else
            $display("===== FAILED %0d item(s) =====", errs);
        $finish;
    end

    initial begin
        #50000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
