`timescale 1ns/1ps

module tb_zdt_axil_interp;
    localparam integer CLK_HZ = 115200;

    reg aclk = 0;
    reg aresetn = 0;
    reg [7:0] awaddr = 0;
    reg awvalid = 0;
    wire awready;
    reg [31:0] wdata = 0;
    reg [3:0] wstrb = 4'hF;
    reg wvalid = 0;
    wire wready;
    wire [1:0] bresp;
    wire bvalid;
    reg bready = 1;
    reg [7:0] araddr = 0;
    reg arvalid = 0;
    wire arready;
    wire [31:0] rdata;
    wire [1:0] rresp;
    wire rvalid;
    reg rready = 1;
    wire tx, uart_sel;
    wire [15:0] status_frames;
    wire status_busy, status_done;

    zdt_top21 #(.CLK_HZ(CLK_HZ), .BAUD(115200)) dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .tx(tx), .uart_sel(uart_sel),
        .status_frames(status_frames), .status_busy(status_busy), .status_done(status_done)
    );

    always #5 aclk = ~aclk;

    task axi_write(input [7:0] address, input [31:0] value);
        begin
            @(negedge aclk);
            awaddr = address;
            wdata = value;
            awvalid = 1;
            wvalid = 1;
            while (!(awready && wready)) @(posedge aclk);
            @(posedge aclk);
            @(negedge aclk);
            awvalid = 0;
            wvalid = 0;
            while (!bvalid) @(posedge aclk);
            @(posedge aclk);
        end
    endtask

    task axi_read(input [7:0] address, output [31:0] value);
        begin
            @(negedge aclk);
            araddr = address;
            arvalid = 1;
            while (!arready) @(posedge aclk);
            @(posedge aclk);
            @(negedge aclk);
            arvalid = 0;
            while (!rvalid) @(posedge aclk);
            value = rdata;
            @(posedge aclk);
        end
    endtask

    reg [31:0] value;
    integer cycles;
    initial begin
        repeat (5) @(negedge aclk);
        aresetn = 1;

        axi_write(8'h20, 10);
        axi_write(8'h40, 3000);
        axi_read(8'h40, value);
        if (value !== 3000) begin
            $display("FAIL: duration readback got %0d expected 3000", value);
            $fatal(1);
        end

        axi_write(8'h24, 3000);
        axi_write(8'h28, 0);
        axi_write(8'h2C, 1200);
        axi_write(8'h00, 32'h48);
        axi_write(8'h30, 1);

        cycles = 0;
        while (!(status_frames == 240 && !status_busy && status_done) && cycles < 4000000) begin
            @(posedge aclk);
            cycles = cycles + 1;
        end
        if (status_frames !== 240 || status_busy || !status_done) begin
            $display("FAIL: terminal state frames=%0d busy=%0d done=%0d (expected 240, 0, 1)",
                     status_frames, status_busy, status_done);
            $fatal(1);
        end

        $display("PASS: 8-bit AXI, duration=3000, 60 interpolation ticks, 240 position frames");
        $finish;
    end

    initial begin
        #100000000;
        $fatal(1, "TIMEOUT");
    end
endmodule
