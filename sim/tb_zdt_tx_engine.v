`timescale 1ns/1ps
//=============================================================================
// tb_zdt_tx_engine.v —— 四轴发送引擎自检测试台
//   一次触发应发出 4 帧（地址 1..4），逐帧比对字节并测量总耗时
//=============================================================================
module tb_zdt_tx_engine;

    localparam integer BAUD_DIV   = 8;
    localparam integer CLK_PERIOD = 10;
    localparam real    BIT_TIME   = BAUD_DIV * CLK_PERIOD;

    reg clk = 0, rst_n = 0;
    reg        start = 0;
    wire       busy, done, tx;
    wire [2:0] cur_joint;
    wire [15:0] frame_count;

    reg [31:0] pos1 = 32'd0;
    reg [31:0] pos2 = 32'hFFFF8A3B;   // 负数 → dir=1
    reg [31:0] pos3 = 32'd1234;
    reg [31:0] pos4 = 32'hFFFFFC18;   // -1000
    reg [15:0] speed   = 16'd3000;
    reg [7:0]  accel   = 8'd240;
    reg        abs_rel = 1'b1;
    reg        sync_en = 1'b0;

    zdt_tx_engine #(.CLK_HZ(BAUD_DIV * 115200), .BAUD(115200)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done(done),
        .pos1(pos1), .pos2(pos2), .pos3(pos3), .pos4(pos4),
        .speed(speed), .accel(accel), .abs_rel(abs_rel), .sync_en(sync_en),
        .skip_en(1'b0),          // 新增端口：本组用例验证完整使能流程
        .gap_ms(8'd0),           // 新增端口：本用例不考察帧间隔
        .tx(tx), .cur_joint(cur_joint), .frame_count(frame_count)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    // UART 接收模型
    //------------------------------------------------------------------
    reg [7:0] cap [0:127];
    integer   rx_idx = 0;
    integer   bad_stop = 0;
    reg [7:0] rx_byte;
    integer   b;

    initial begin : uart_rx
        forever begin
            @(negedge tx);
            #(BIT_TIME * 1.5);
            for (b = 0; b < 8; b = b + 1) begin
                rx_byte[b] = tx;
                #(BIT_TIME);
            end
            if (tx !== 1'b1) bad_stop = bad_stop + 1;
            if (rx_idx < 128) cap[rx_idx] = rx_byte;
            rx_idx = rx_idx + 1;
        end
    end

    //------------------------------------------------------------------
    // 期望
    //------------------------------------------------------------------
    reg [7:0] exp_bytes [0:12];
    integer   i, k, errs = 0;
    integer   t0, t1;

    function [31:0] mag;
        input [31:0] v;
        begin
            mag = v[31] ? (~v + 32'd1) : v;
        end
    endfunction

    task build_expect;
        input [7:0]  a;
        input [31:0] p;
        begin
            exp_bytes[0]  = a;
            exp_bytes[1]  = 8'hFD;
            exp_bytes[2]  = {7'd0, p[31]};
            exp_bytes[3]  = speed[15:8];
            exp_bytes[4]  = speed[7:0];
            exp_bytes[5]  = accel;
            exp_bytes[6]  = mag(p) >> 24;
            exp_bytes[7]  = mag(p) >> 16;
            exp_bytes[8]  = mag(p) >> 8;
            exp_bytes[9]  = mag(p);
            exp_bytes[10] = {7'd0, abs_rel};
            exp_bytes[11] = {7'd0, sync_en};
            exp_bytes[12] = 8'h6B;
        end
    endtask

    task check_frame;
        input integer base;
        input [127:0] name;
        begin
            for (k = 0; k < 13; k = k + 1) begin
                if (cap[base + k] !== exp_bytes[k]) begin
                    $display("  [FAIL] %0s byte[%0d] got=%02X exp=%02X",
                             name, k, cap[base + k], exp_bytes[k]);
                    errs = errs + 1;
                end
            end
            $display("  %0s: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                     name, cap[base], cap[base+1], cap[base+2],  cap[base+3],  cap[base+4],
                     cap[base+5], cap[base+6], cap[base+7],  cap[base+8],  cap[base+9],
                     cap[base+10], cap[base+11], cap[base+12]);
        end
    endtask

    initial begin
        rst_n = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);

        $display("触发一次四轴发送（pos: 0 / -30149 / 1234 / -1000）");
        t0 = $time;
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        wait (done === 1'b1);
        t1 = $time;

        #(BIT_TIME * 12);   // 等最后一帧收完

        $display("  收到 %0d 字节（期望 124 = 12 使能帧 ×6 + 4 位置帧 ×13）", rx_idx);

        if (rx_idx != 124) begin
            $display("  [FAIL] 字节数不对");
            errs = errs + 1;
        end else begin
            build_expect(8'h01, pos1); check_frame(72,  "J1  ");
            build_expect(8'h02, pos2); check_frame(85, "J2  ");
            build_expect(8'h03, pos3); check_frame(98, "J3  ");
            build_expect(8'h04, pos4); check_frame(111, "J4  ");
        end

        $display("");
        $display("  一次触发总耗时: %0d ns = %.2f 个位时间 = %.1f 字符时间",
                 t1 - t0, real'(t1 - t0) / BIT_TIME, real'(t1 - t0) / BIT_TIME / 10.0);
        $display("  （位置阶段理论值 52 字节 × 10 位 = 520 位时间）");
        $display("  frame_count = %0d（期望 16 = 12 使能 + 4 位置）", frame_count);
        if (frame_count !== 16'd16) begin
            $display("  [FAIL] frame_count");
            errs = errs + 1;
        end
        if (bad_stop != 0) begin
            $display("  [FAIL] 停止位错误 %0d 次", bad_stop);
            errs = errs + 1;
        end

        $display("");
        if (errs == 0) $display("===== 全部通过 =====");
        else           $display("===== 失败 %0d 项 =====", errs);
        $finish;
    end

    initial begin
        #10000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
