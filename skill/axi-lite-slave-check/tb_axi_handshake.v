// tb_axi_handshake.v
// Minimal AXI4-Lite write/read driver that covers the three handshake orders
// that break hand-written slaves:
//   1. AW and W asserted in the same cycle
//   2. AW first, W one cycle later
//   3. W first, AW one cycle later
//   4. a read of a write-only register (must not hang the bus)
//
// Adapt the DUT instantiation to your slave.

`timescale 1ns/1ps

module tb_axi_handshake;

    reg clk = 0;
    reg rstn = 0;
    always #5 clk = ~clk;

    reg  [5:0]  awaddr = 0;
    reg         awvalid = 0;
    wire        awready;
    reg  [31:0] wdata = 0;
    reg  [3:0]  wstrb = 4'hF;
    reg         wvalid = 0;
    wire        wready;
    wire [1:0]  bresp;
    wire        bvalid;
    reg         bready = 1;

    reg  [5:0]  araddr = 0;
    reg         arvalid = 0;
    wire        arready;
    wire [31:0] rdata;
    wire [1:0]  rresp;
    wire        rvalid;
    reg         rready = 1;

    integer errors = 0;

    // ---- DUT ----
    // dut u_dut (.aclk(clk), .aresetn(rstn), ... );

    task write_same_cycle(input [5:0] a, input [31:0] d);
        begin
            @(posedge clk);
            awaddr <= a; awvalid <= 1'b1;
            wdata  <= d; wvalid  <= 1'b1;
            @(posedge clk);
            awvalid <= 1'b0; wvalid <= 1'b0;
            wait (bvalid);
            @(posedge clk);
        end
    endtask

    task write_aw_first(input [5:0] a, input [31:0] d);
        begin
            @(posedge clk);
            awaddr <= a; awvalid <= 1'b1;
            @(posedge clk);
            awvalid <= 1'b0;
            wdata  <= d; wvalid  <= 1'b1;
            @(posedge clk);
            wvalid <= 1'b0;
            wait (bvalid);
            @(posedge clk);
        end
    endtask

    task write_w_first(input [5:0] a, input [31:0] d);
        begin
            @(posedge clk);
            wdata <= d; wvalid <= 1'b1;
            @(posedge clk);
            wvalid <= 1'b0;
            awaddr <= a; awvalid <= 1'b1;
            @(posedge clk);
            awvalid <= 1'b0;
            wait (bvalid);
            @(posedge clk);
        end
    endtask

    task read_reg(input [5:0] a);
        begin
            @(posedge clk);
            araddr <= a; arvalid <= 1'b1;
            @(posedge clk);
            arvalid <= 1'b0;
            wait (rvalid);
            @(posedge clk);
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rstn <= 1'b1;
        repeat (2) @(posedge clk);

        write_same_cycle(6'h00, 32'h0000_0001);
        write_aw_first  (6'h00, 32'h0000_0003);
        write_w_first   (6'h00, 32'h0000_0007);
        read_reg        (6'h00);   // must return, not hang

        if (errors == 0) $display("PASS: all handshake orders completed");
        else             $display("FAIL: %0d errors", errors);
        $finish;
    end

    // watchdog: a hang is a failure
    initial begin
        #200000;
        $display("FAIL: timeout (bus hang)");
        $finish;
    end

endmodule
