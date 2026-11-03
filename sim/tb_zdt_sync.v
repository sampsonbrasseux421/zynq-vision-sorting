`timescale 1ns/1ps
//=============================================================================
// tb_zdt_sync.v —— 同步触发模式自检
//   sync_en=1 时：12 使能帧 + 4 位置帧（带同步标志） + 1 同步触发帧
//   字节数 = 72 + 52 + 4 = 128
//   同步触发帧必须是 00 FF 66 6B
//=============================================================================
module tb_zdt_sync;

    localparam integer BAUD_DIV   = 8;
    localparam integer CLK_PERIOD = 10;
    localparam real    BIT_TIME   = BAUD_DIV * CLK_PERIOD;

    reg clk = 0, rst_n = 0;
    reg        start = 0;
    wire       busy, done, tx;
    wire [2:0] cur_joint;
    wire [15:0] frame_count;

    reg [31:0] pos1 = 32'd1000;
    reg [31:0] pos2 = 32'hFFFFF830;   // -2000
    reg [31:0] pos3 = 32'd3000;
    reg [31:0] pos4 = 32'hFFFFF448;   // -3000
    reg [15:0] speed   = 16'd3000;
    reg [7:0]  accel   = 8'd240;
    reg        abs_rel = 1'b0;
    reg        sync_en = 1'b1;        // ← 关键：开同步
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

    integer i, errs = 0;

    initial begin
        rst_n = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);

        $display("=== 同步模式（sync_en=1）===");
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        wait (done === 1'b1);
        repeat (BIT_TIME * 12) @(negedge clk);

        $display("  收到 %0d 字节（期望 128）", rx_idx);
        if (rx_idx != 128) begin
            $display("  [FAIL] 字节数"); errs = errs + 1;
        end else begin
            // 位置帧 @72..123，同步触发帧 @124..127
            $display("  POS @72  : %02X %02X %02X %02X ... sync=%02X ... %02X",
                     cap[72],cap[73],cap[74],cap[75], cap[83], cap[84]);
            $display("  SYNC @124: %02X %02X %02X %02X（期望 00 FF 66 6B）",
                     cap[124],cap[125],cap[126],cap[127]);
            if (cap[124] !== 8'h00 || cap[125] !== 8'hFF ||
                cap[126] !== 8'h66 || cap[127] !== 8'h6B) begin
                $display("  [FAIL] 同步触发帧错误");
                errs = errs + 1;
            end
            // 4 个位置帧的 sync 字节都应为 1
            for (i = 0; i < 4; i = i + 1) begin
                if (cap[72 + i*13 + 11] !== 8'h01) begin
                    $display("  [FAIL] 位置帧 %0d 的 sync 字节 = %02X（应为 01）",
                             i+1, cap[72 + i*13 + 11]);
                    errs = errs + 1;
                end
            end
        end

        $display("  frame_count = %0d（期望 17）", frame_count);
        if (frame_count !== 16'd17) begin
            $display("  [FAIL] frame_count"); errs = errs + 1;
        end

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
