`timescale 1ns/1ps
//=============================================================================
// tb_zdt_frame_tx.v —— 自检测试台
//   用 UART 接收模型把 tx 线还原成字节，与期望序列逐字节比对
//   时钟 10ns，BAUD_DIV=8 → 每位 80ns
//=============================================================================
module tb_zdt_frame_tx;

    localparam integer BAUD_DIV     = 8;
    localparam integer CLK_PERIOD   = 10;              // ns
    localparam real    BIT_TIME     = BAUD_DIV * CLK_PERIOD;   // 80 ns

    reg clk = 0, rst_n = 0;
    reg        start = 0;
    wire       busy, done, tx;

    reg [7:0]  addr    = 8'h01;
    reg        dir     = 1'b1;
    reg [15:0] speed   = 16'd3000;
    reg [7:0]  accel   = 8'd240;
    reg [31:0] pos     = 32'd0;
    reg        abs_rel = 1'b1;
    reg        sync    = 1'b0;

    zdt_frame_tx #(.CLK_HZ(BAUD_DIV * 115200), .BAUD(115200)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done(done),
        .fsel(2'd0),              // 0=位置帧（新增的帧类型选择端口）
        .addr(addr), .dir(dir), .speed(speed), .accel(accel),
        .pos(pos), .abs_rel(abs_rel), .sync(sync),
        .tx(tx)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    // UART 接收模型
    //------------------------------------------------------------------
    reg [7:0] captured [0:31];
    integer   rx_idx   = 0;
    integer   bad_stop = 0;
    reg [7:0] rx_byte;
    integer   b;

    initial begin : uart_rx
        forever begin
            @(negedge tx);                       // 起始位下降沿
            #(BIT_TIME * 1.5);                   // 移到 bit0 中心
            for (b = 0; b < 8; b = b + 1) begin
                rx_byte[b] = tx;
                #(BIT_TIME);
            end
            // 此时在 bit7 之后的停止位中点
            if (tx !== 1'b1) bad_stop = bad_stop + 1;
            captured[rx_idx] = rx_byte;
            rx_idx = rx_idx + 1;
        end
    end

    //------------------------------------------------------------------
    // 期望字节
    //------------------------------------------------------------------
    reg [7:0] exp_bytes [0:12];
    integer   i;
    integer   errs = 0;

    task setup_expect;
        begin
            exp_bytes[0]  = addr;
            exp_bytes[1]  = 8'hFD;
            exp_bytes[2]  = {7'd0, dir};
            exp_bytes[3]  = speed[15:8];
            exp_bytes[4]  = speed[7:0];
            exp_bytes[5]  = accel;
            exp_bytes[6]  = pos[31:24];
            exp_bytes[7]  = pos[23:16];
            exp_bytes[8]  = pos[15:8];
            exp_bytes[9]  = pos[7:0];
            exp_bytes[10] = {7'd0, abs_rel};
            exp_bytes[11] = {7'd0, sync};
            exp_bytes[12] = 8'h6B;
        end
    endtask

    task compare_bytes;
        begin
            for (i = 0; i < 13; i = i + 1) begin
                if (captured[i] !== exp_bytes[i]) begin
                    $display("  [FAIL] byte[%0d] got=%02X exp=%02X", i, captured[i], exp_bytes[i]);
                    errs = errs + 1;
                end
            end
        end
    endtask

    task show_bytes;
        begin
            $display("  实际: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                     captured[0], captured[1], captured[2],  captured[3],  captured[4],
                     captured[5], captured[6], captured[7],  captured[8],  captured[9],
                     captured[10], captured[11], captured[12]);
            $display("  期望: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                     exp_bytes[0], exp_bytes[1], exp_bytes[2],  exp_bytes[3],  exp_bytes[4],
                     exp_bytes[5], exp_bytes[6], exp_bytes[7],  exp_bytes[8],  exp_bytes[9],
                     exp_bytes[10], exp_bytes[11], exp_bytes[12]);
        end
    endtask

    task send_one;
        begin
            @(negedge clk); start = 1'b1;
            @(negedge clk); start = 1'b0;
            wait (done === 1'b1);
            repeat (2) @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (3) @(negedge clk);

        // ---------------- 第一帧 ----------------
        setup_expect();
        $display("第一帧: addr=01 dir=1 speed=3000 accel=240 pos=0 abs=1 sync=0");
        send_one();
        #(BIT_TIME * 12);                        // 等最后一个字节收完
        if (rx_idx != 13) begin
            $display("  [FAIL] 收到 %0d 字节，应为 13", rx_idx);
            errs = errs + 1;
        end else begin
            compare_bytes();
            show_bytes();
        end
        if (bad_stop != 0) begin
            $display("  [FAIL] 停止位错误 %0d 次", bad_stop);
            errs = errs + 1;
        end

        // ---------------- 第二帧：换参数 ----------------
        addr = 8'h02; dir = 1'b1; pos = 32'hFFFF8A3B; abs_rel = 1'b1; sync = 1'b1;
        speed = 16'd2500; accel = 8'd200;
        setup_expect();
        rx_idx = 0;
        bad_stop = 0;
        $display("");
        $display("第二帧: addr=02 speed=2500 accel=200 pos=FFFF8A3B abs=1 sync=1");
        send_one();
        #(BIT_TIME * 12);
        if (rx_idx != 13) begin
            $display("  [FAIL] 收到 %0d 字节，应为 13", rx_idx);
            errs = errs + 1;
        end else begin
            compare_bytes();
            show_bytes();
        end

        $display("");
        if (errs == 0) $display("===== 全部通过 =====");
        else           $display("===== 失败 %0d 项 =====", errs);
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
