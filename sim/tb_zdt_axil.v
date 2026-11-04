`timescale 1ns/1ps
//=============================================================================
// tb_zdt_axil.v —— AXI4-Lite 接口 + 引擎 联合测试
//   1) 通过 AXI 写 4 个目标位置和时序参数
//   2) 写 CTRL.start 触发
//   3) 轮询 STATUS 直到 done
//   4) 检查串口输出的 124 字节是否正确（12 使能帧 + 4 位置帧）
//=============================================================================
module tb_zdt_axil;

    localparam integer BAUD_DIV   = 8;
    localparam integer CLK_PERIOD = 10;
    localparam real    BIT_TIME   = BAUD_DIV * CLK_PERIOD;

    reg aclk = 0, aresetn = 0;

    reg  [7:0]  awaddr = 0;
    reg         awvalid = 0;
    wire        awready;
    reg  [31:0] wdata = 0;
    reg  [3:0]  wstrb = 4'hF;
    reg         wvalid = 0;
    wire        wready;
    wire [1:0]  bresp;
    wire        bvalid;
    reg         bready = 1;
    reg  [7:0]  araddr = 0;
    reg         arvalid = 0;
    wire        arready;
    wire [31:0] rdata;
    wire [1:0]  rresp;
    wire        rvalid;
    reg         rready = 1;
    wire        tx;

    zdt_axil #(.CLK_HZ(BAUD_DIV * 115200), .BAUD(115200)) dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .tx(tx)
    );

    always #(CLK_PERIOD/2) aclk = ~aclk;

    //------------------------------------------------------------------
    // AXI 事务
    //------------------------------------------------------------------
    task axi_write;
        input [5:0]  a;
        input [31:0] d;
        begin
            @(negedge aclk);
            awaddr = a; awvalid = 1'b1;
            wdata  = d; wvalid  = 1'b1;
            @(posedge aclk);
            while (!(awready && wready)) @(posedge aclk);
            @(negedge aclk);
            awvalid = 1'b0; wvalid = 1'b0;
            while (!bvalid) @(posedge aclk);
            @(posedge aclk);
        end
    endtask

    task axi_read;
        input  [5:0]  a;
        output [31:0] d;
        begin
            @(negedge aclk);
            araddr = a; arvalid = 1'b1;
            @(posedge aclk);
            while (!arready) @(posedge aclk);
            @(negedge aclk);
            arvalid = 1'b0;
            while (!rvalid) @(posedge aclk);
            d = rdata;
            @(posedge aclk);
        end
    endtask

    //------------------------------------------------------------------
    // UART 接收
    //------------------------------------------------------------------
    reg [7:0] cap [0:127];
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
            if (rx_idx < 128) cap[rx_idx] = rx_byte;
            rx_idx = rx_idx + 1;
        end
    end

    //------------------------------------------------------------------
    reg [31:0] st;
    integer    errs = 0;
    integer    k;

    initial begin
        aresetn = 0;
        repeat (5) @(negedge aclk);
        aresetn = 1;
        repeat (3) @(negedge aclk);

        $display("=== 通过 AXI 配置并触发 ===");
        axi_write(6'h04, {8'd240, 16'd3000});      // accel=240, speed=3000
        axi_write(6'h08, 32'd0);                   // J1
        axi_write(6'h0C, 32'hFFFF8A3B);            // J2 = -30149
        axi_write(6'h10, 32'd1234);                // J3
        axi_write(6'h14, 32'hFFFFFC18);            // J4 = -1000
        $display("  参数已写入");

        axi_write(6'h00, 32'h0000_0003);           // start=1, abs_rel=1
        $display("  已触发");

        // 等 done
        st = 0;
        while (!(st[1])) begin
            axi_read(6'h18, st);
            repeat (20) @(negedge aclk);
        end
        $display("  STATUS = %08X  busy=%0d done=%0d frame_count=%0d",
                 st, st[0], st[1], st[31:16]);

        #(BIT_TIME * 12);

        $display("  收到 %0d 字节（期望 124 = 12 使能帧 ×6 + 4 位置帧 ×13）", rx_idx);
        if (rx_idx != 124) begin
            $display("  [FAIL] 字节数不对");
            errs = errs + 1;
        end else begin
            // 帧序：12 帧使能（12×6 = 72 字节）+ 4 帧位置（每帧 13 字节）
            // 位置帧基址：J1=72  J2=85  J3=98  J4=111
            $display("  J1: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                cap[72],cap[73],cap[74],cap[75],cap[76],cap[77],cap[78],cap[79],cap[80],cap[81],cap[82],cap[83],cap[84]);
            $display("  J2: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                cap[85],cap[86],cap[87],cap[88],cap[89],cap[90],cap[91],cap[92],cap[93],cap[94],cap[95],cap[96],cap[97]);
            // 关键字段检查
            if (cap[72] !== 8'h01 || cap[73] !== 8'hFD || cap[84] !== 8'h6B) begin
                $display("  [FAIL] J1 帧头/校验错");
                errs = errs + 1;
            end
            if (cap[85] !== 8'h02 || cap[87] !== 8'h01) begin
                $display("  [FAIL] J2 地址或方向错（应为 02 ... 01）");
                errs = errs + 1;
            end
            if ({cap[91],cap[92],cap[93],cap[94]} !== 32'h0000_75C5) begin
                $display("  [FAIL] J2 位置字段错");
                errs = errs + 1;
            end
        end

        if (st[31:16] !== 16'd16) begin
            $display("  [FAIL] frame_count 应为 16");
            errs = errs + 1;
        end

        $display("");
        if (errs == 0) $display("===== 全部通过 =====");
        else           $display("===== 失败 %0d 项 =====", errs);
        $finish;
    end

    initial begin
        #20000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
