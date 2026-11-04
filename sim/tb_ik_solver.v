//=============================================================================
// tb_ik_solver.v —— 验证 ik_solver 与浮点参考的一致性
//
// 期望值来自 ik_zdt.c 的浮点 ik_solve()（已离线核对）
// 判据：|误差| < 256 deg256（1 度）
//=============================================================================
`timescale 1ns / 1ps

module tb_ik_solver;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                start = 0;
    reg signed [15:0]  x, y, z;
    wire signed [31:0] j1, j2, j3, j4;
    wire               busy, done, err;

    always #10 clk = ~clk;      // 50 MHz

    ik_solver dut (
        .clk(clk), .rst_n(rst_n),
        .start(start),
        .x_mm10(x), .y_mm10(y), .z_mm10(z),
        .j1_deg256(j1), .j2_deg256(j2), .j3_deg256(j3), .j4_deg256(j4),
        .busy(busy), .done(done), .err(err)
    );

    integer errors = 0;
    integer cases = 0;
    integer cycles;
    integer i;
    integer final_math_active = 0;
    integer final_math_cycles = 0;
    integer final_math_samples = 0;

    always @(negedge clk) begin
        if (!rst_n) begin
            final_math_active = 0;
            final_math_cycles = 0;
            final_math_samples = 0;
        end else if (dut.stage == 4'd8 && dut.cordic_done) begin
            final_math_active = 1;
            final_math_cycles = 0;
        end else if (final_math_active) begin
            final_math_cycles = final_math_cycles + 1;
            if (done) begin
                final_math_samples = final_math_cycles;
                final_math_active = 0;
            end
        end
    end

    // 测试向量：x,y,z, 期望 j1,j2,j3,j4
    reg signed [15:0] vx [0:9];
    reg signed [15:0] vy [0:9];
    reg signed [15:0] vz [0:9];
    reg signed [31:0] e1 [0:9];
    reg signed [31:0] e2 [0:9];
    reg signed [31:0] e3 [0:9];
    reg signed [31:0] e4 [0:9];

    task run_case(input integer idx);
        reg signed [31:0] d1, d2, d3, d4;
        reg signed [31:0] m;
        begin
            @(negedge clk);
            x = vx[idx]; y = vy[idx]; z = vz[idx];
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            cycles = 0;
            while (!done && cycles < 2000) begin
                @(posedge clk);
                cycles = cycles + 1;
            end

            d1 = j1 - e1[idx]; if (d1 < 0) d1 = -d1;
            d2 = j2 - e2[idx]; if (d2 < 0) d2 = -d2;
            d3 = j3 - e3[idx]; if (d3 < 0) d3 = -d3;
            d4 = j4 - e4[idx]; if (d4 < 0) d4 = -d4;
            m = d1; if (d2 > m) m = d2; if (d3 > m) m = d3; if (d4 > m) m = d4;

            cases = cases + 1;
            if (!done) begin
                $display("  [%0d] TIMEOUT after %0d cycles", idx, cycles);
                errors = errors + 1;
            end else if (final_math_samples < 3) begin
                $display("  [%0d] final math was not pipelined: %0d cycles after CORDIC", idx, final_math_samples);
                errors = errors + 1;
            end else if (m >= 256) begin
                $display("  [%0d] (%0d,%0d,%0d) got %0d %0d %0d %0d exp %0d %0d %0d %0d  ERR=%0d",
                         idx, x, y, z, j1, j2, j3, j4, e1[idx], e2[idx], e3[idx], e4[idx], m);
                errors = errors + 1;
            end else begin
                $display("  [%0d] (%0d,%0d,%0d) got %0d %0d %0d %0d  err=%0d  cycles=%0d",
                         idx, x, y, z, j1, j2, j3, j4, m, cycles);
            end
        end
    endtask

    initial begin
        vx[0]=2500;  vy[0]=0;  vz[0]=1500;  e1[0]=0;  e2[0]=-5985;  e3[0]=-4047;  e4[0]=-8752;
        vx[1]=2500;  vy[1]=1000;  vz[1]=1500;  e1[1]=5581;  e2[1]=-7127;  e3[1]=-2191;  e4[1]=-8038;
        vx[2]=2500;  vy[2]=-1000;  vz[2]=1500;  e1[2]=-5581;  e2[2]=-7127;  e3[2]=-2191;  e4[2]=-8038;
        vx[3]=2000;  vy[3]=2000;  vz[3]=1200;  e1[3]=11520;  e2[3]=-9313;  e3[3]=-1543;  e4[3]=-9576;
        vx[4]=3000;  vy[4]=-1500;  vz[4]=2000;  e1[4]=-6801;  e2[4]=-10197;  e3[4]=7620;  e4[4]=-1297;
        vx[5]=1500;  vy[5]=0;  vz[5]=2500;  e1[5]=0;  e2[5]=4397;  e3[5]=-5915;  e4[5]=-238;
        vx[6]=2800;  vy[6]=800;  vz[6]=1000;  e1[6]=4082;  e2[6]=-10780;  e3[6]=-934;  e4[6]=-10434;
        vx[7]=2200;  vy[7]=-2200;  vz[7]=1800;  e1[7]=-11520;  e2[7]=-8771;  e3[7]=3432;  e4[7]=-4059;
        vx[8]=1800;  vy[8]=1500;  vz[8]=2200;  e1[8]=10190;  e2[8]=-2426;  e3[8]=-2198;  e4[8]=-3344;
        vx[9]=2600;  vy[9]=0;  vz[9]=900;  e1[9]=0;  e2[9]=-9872;  e3[9]=-4213;  e4[9]=-12805;

        x = 0; y = 0; z = 0;
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);

        $display("=== ik_solver 定点 vs 浮点参考 ===");
        for (i = 0; i < 10; i = i + 1)
            run_case(i);

        $display("");
        $display("用例 %0d 个，失败 %0d 个", cases, errors);
        if (errors == 0)
            $display("[PASS] ik_solver");
        else
            $display("[FAIL] ik_solver");
        $finish;
    end

    initial begin
        #2000000;
        $display("[FAIL] global timeout");
        $finish;
    end

endmodule
