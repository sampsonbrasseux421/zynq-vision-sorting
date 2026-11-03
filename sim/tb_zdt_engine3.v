`timescale 1ns/1ps
//=============================================================================
// tb_zdt_engine3.v —— 使能+位置两阶段（带帧间隔）自检
//   gap_ms=0 时纯功能验证：124 字节，16 帧
//   gap_ms>0 时验证帧间隔
//=============================================================================
module tb_zdt_engine3;

    localparam integer BAUD_DIV   = 8;
    localparam integer CLK_PERIOD = 10;
    localparam real    BIT_TIME   = BAUD_DIV * CLK_PERIOD;
    // 仿真时钟 = BAUD_DIV*115200 = 921600 Hz，1 ms = 921 个周期
    localparam integer MS_CYC_SIM = 921;

    reg clk = 0, rst_n = 0;
    reg        start = 0;
    wire       busy, done, tx;
    wire [2:0] cur_joint;
    wire [15:0] frame_count;

    reg [31:0] pos1 = 32'd0;
    reg [31:0] pos2 = 32'hFFFF8A3B;
    reg [31:0] pos3 = 32'd1234;
    reg [31:0] pos4 = 32'hFFFFFC18;
    reg [15:0] speed   = 16'd3000;
    reg [7:0]  accel   = 8'd240;
    reg        abs_rel = 1'b1;
    reg        sync_en = 1'b0;
    reg [7:0]  gap_ms  = 8'd0;

    zdt_tx_engine #(.CLK_HZ(BAUD_DIV * 115200), .BAUD(115200)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done(done),
        .pos1(pos1), .pos2(pos2), .pos3(pos3), .pos4(pos4),
        .speed(speed), .accel(accel), .abs_rel(abs_rel), .sync_en(sync_en),
        .skip_en(1'b0),          // 新增端口：本组用例验证完整使能流程
        .gap_ms(gap_ms),
        .tx(tx), .cur_joint(cur_joint), .frame_count(frame_count)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    reg [7:0] cap [0:255];
    integer   rx_idx = 0;
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
            if (rx_idx < 256) cap[rx_idx] = rx_byte;
            rx_idx = rx_idx + 1;
        end
    end

    integer i, k, errs = 0;
    integer t0, t1;

    function [31:0] mag;
        input [31:0] v;
        begin mag = v[31] ? (~v + 32'd1) : v; end
    endfunction

    task check_en;
        input integer base;
        input [7:0]   a;
        begin
            if (cap[base] !== a || cap[base+1] !== 8'hF3 || cap[base+2] !== 8'hAB ||
                cap[base+3] !== 8'h01 || cap[base+4] !== 8'h00 || cap[base+5] !== 8'h6B) begin
                $display("  [FAIL] 使能帧 @%0d: %02X %02X %02X %02X %02X %02X",
                         base, cap[base], cap[base+1], cap[base+2],
                         cap[base+3], cap[base+4], cap[base+5]);
                errs = errs + 1;
            end
        end
    endtask

    task check_pos;
        input integer base;
        input [7:0]   a;
        input [31:0]  p;
        reg   [31:0] m;
        begin
            m = mag(p);
            if (cap[base] !== a || cap[base+1] !== 8'hFD ||
                cap[base+2] !== {7'd0, p[31]} || cap[base+12] !== 8'h6B ||
                {cap[base+6],cap[base+7],cap[base+8],cap[base+9]} !== m) begin
                $display("  [FAIL] 位置帧 @%0d", base);
                errs = errs + 1;
            end
        end
    endtask

    initial begin
        rst_n = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);

        $display("=== gap_ms = 0（纯功能验证）===");
        t0 = $time;
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        wait (done === 1'b1);
        t1 = $time;
        repeat (BIT_TIME * 12) @(negedge clk);

        $display("  收到 %0d 字节（期望 124）", rx_idx);
        if (rx_idx != 124) begin
            $display("  [FAIL] 字节数"); errs = errs + 1;
        end else begin
            for (i = 0; i < 4; i = i + 1)
                for (k = 0; k < 3; k = k + 1)
                    check_en(i * 18 + k * 6, i + 1);
            check_pos(72,  8'h01, pos1);
            check_pos(85,  8'h02, pos2);
            check_pos(98,  8'h03, pos3);
            check_pos(111, 8'h04, pos4);
            $display("  EN  @0  : %02X %02X %02X %02X %02X %02X",
                     cap[0],cap[1],cap[2],cap[3],cap[4],cap[5]);
            $display("  POS @72 : %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                     cap[72],cap[73],cap[74],cap[75],cap[76],cap[77],cap[78],
                     cap[79],cap[80],cap[81],cap[82],cap[83],cap[84]);
        end
        $display("  总耗时 %.1f 个位时间（理论 1240）", real'(t1 - t0) / BIT_TIME);
        $display("  frame_count = %0d（期望 16）", frame_count);
        if (frame_count !== 16'd16) begin $display("  [FAIL] frame_count"); errs = errs + 1; end

        $display("");
        if (errs == 0) $display("===== 全部通过 =====");
        else           $display("===== 失败 %0d 项 =====", errs);
        $finish;
    end

    initial begin
        #50000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
