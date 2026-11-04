`timescale 1ns/1ps

module tb_zdt_ik_axil;
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
    wire tx;

    integer errors = 0;
    integer cycles;
    integer variant;
    integer start_count = 0;
    integer count_before;
    reg [31:0] status;
    reg [31:0] angle;
    reg signed [31:0] difference;

    always #10 aclk = ~aclk;

    always @(posedge aclk) begin
        if (!aresetn) start_count <= 0;
        else if (dut.ik_start && !dut.ik_busy) start_count <= start_count + 1;
    end

    zdt_axil dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .tx(tx)
    );

    task axi_write_split;
        input [5:0] address;
        input [31:0] value;
        input integer aw_delay;
        input integer w_delay;
        input integer response_delay;
        integer timeout;
        begin
            bready = 0;
            fork
                begin
                    repeat (aw_delay + 1) @(negedge aclk);
                    awaddr = address;
                    awvalid = 1;
                    @(posedge aclk);
                    while (!awready) @(posedge aclk);
                    @(negedge aclk);
                    awvalid = 0;
                    awaddr = 6'h00;
                end
                begin
                    repeat (w_delay + 1) @(negedge aclk);
                    wdata = value;
                    wvalid = 1;
                    @(posedge aclk);
                    while (!wready) @(posedge aclk);
                    @(negedge aclk);
                    wvalid = 0;
                    wdata = 32'hDEADBEEF;
                end
            join
            timeout = 0;
            while (!bvalid && timeout < 32) begin
                @(negedge aclk);
                timeout = timeout + 1;
            end
            if (!bvalid) begin
                $display("[FAIL] lost AXI write at %02X (AW delay=%0d W delay=%0d)", address, aw_delay, w_delay);
                errors = errors + 1;
            end
            repeat (response_delay) begin
                @(negedge aclk);
                if (!bvalid || bresp != 0) begin
                    $display("[FAIL] write response changed under backpressure");
                    errors = errors + 1;
                end
            end
            @(negedge aclk);
            bready = 1;
            @(posedge aclk);
            @(negedge aclk);
        end
    endtask

    task axi_write;
        input [5:0] address;
        input [31:0] value;
        begin
            axi_write_split(address, value, 0, 0, 0);
        end
    endtask

    task axi_read;
        input [5:0] address;
        output [31:0] value;
        begin
            @(negedge aclk);
            araddr = address;
            arvalid = 1;
            while (!arready) @(posedge aclk);
            @(negedge aclk);
            arvalid = 0;
            while (!rvalid) @(posedge aclk);
            value = rdata;
            @(posedge aclk);
        end
    endtask

    task check_angle;
        input [31:0] index;
        input signed [31:0] expected;
        input [8*3-1:0] name;
        begin
            axi_write(6'h34, index);
            axi_read(6'h34, angle);
            difference = $signed(angle) - expected;
            if (difference < 0) difference = -difference;
            if ($signed(angle) !== expected) begin
                $display("[FAIL] %0s got %0d expected %0d", name, $signed(angle), expected);
                errors = errors + 1;
            end else begin
                $display("[PASS] %0s = %0d (error %0d deg256)", name, $signed(angle), difference);
            end
        end
    endtask

    task reset_dut;
        begin
            @(negedge aclk);
            aresetn = 0;
            awvalid = 0;
            wvalid = 0;
            repeat (5) @(negedge aclk);
            aresetn = 1;
            repeat (3) @(negedge aclk);
        end
    endtask

    initial begin
      for (variant = 0; variant < 4; variant = variant + 1) begin
        reset_dut;
        $display("=== AXI IK variant %0d ===", variant);

        // Default operation is one software-triggered solve, not a periodic workaround.
        repeat (1300) @(negedge aclk);
        axi_read(6'h3C, status);
        if (status !== 0 || start_count != 0) begin
            $display("[FAIL] unsolicited IK solve: status=%08X starts=%0d", status, start_count);
            errors = errors + 1;
        end

        axi_write(6'h24, 32'd2500);
        axi_write(6'h28, 32'd0);
        axi_write(6'h2C, 32'd1500);
        count_before = start_count;
        case (variant)
            0: axi_write_split(6'h30, 32'd1, 0, 0, 0);
            1: axi_write_split(6'h30, 32'd1, 0, 5, 0);
            2: axi_write_split(6'h30, 32'd1, 5, 0, 0);
            3: axi_write_split(6'h30, 32'd1, 0, 0, 8);
        endcase

        cycles = 0;
        status = 0;
        while (!status[1] && cycles < 256) begin
            axi_read(6'h3C, status);
            cycles = cycles + 1;
        end

        if (!status[1]) begin
            $display("[FAIL] IK done timeout; status=0x%08X", status);
            errors = errors + 1;
        end else if (status[2]) begin
            $display("[FAIL] IK reported error; status=0x%08X", status);
            errors = errors + 1;
        end else begin
            $display("[PASS] IK done after %0d status polls", cycles);
            check_angle(1, 0, "J1");
            check_angle(2, -3723, "J2");
            check_angle(3, 23386, "J3");
            check_angle(0, 19663, "J4");
        end

        if (start_count != count_before + 1) begin
            $display("[FAIL] one write must start exactly one solve");
            errors = errors + 1;
        end

        count_before = start_count;
        axi_write(6'h30, 32'd0);
        wstrb = 4'h0;
        axi_write(6'h30, 32'd1);
        wstrb = 4'hF;
        repeat (2100) @(negedge aclk);
        axi_read(6'h3C, status);
        if (start_count != count_before || status !== 32'd2) begin
            $display("[FAIL] zero/unstrobed write or timer restarted IK: starts=%0d status=%08X", start_count-count_before, status);
            errors = errors + 1;
        end
      end

        if (errors == 0) $display("[PASS] tb_zdt_ik_axil");
        else $fatal(1, "[FAIL] tb_zdt_ik_axil errors=%0d", errors);
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "[FAIL] global timeout");
    end
endmodule
