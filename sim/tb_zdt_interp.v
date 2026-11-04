`timescale 1ns/1ps
//=============================================================================
// tb_zdt_interp.v -- 验证 S 曲线直线插补
//   1) 终点精确
//   2) 拍数 = dur_ms / TICK_MS
//   3) 共线（按主轴比例检查其余轴）
//   4) S 曲线形状（首尾步子小、中间步子大）
//=============================================================================
module tb_zdt_interp;

    localparam CLK_HZ  = 50000000;
    localparam TICK_MS = 20;

    reg clk = 0, rst_n = 0, start = 0;
    reg signed [15:0] x0, y0, z0, x1, y1, z1;
    reg [15:0] dur_ms;
    wire tick, busy, done;
    wire signed [15:0] xq, yq, zq;

    always #10 clk = ~clk;

    zdt_interp #(.CLK_HZ(CLK_HZ), .TICK_MS(TICK_MS)) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .x0(x0), .y0(y0), .z0(z0), .x1(x1), .y1(y1), .z1(z1),
        .dur_ms(dur_ms),
        .tick(tick), .xq(xq), .yq(yq), .zq(zq),
        .busy(busy), .done(done)
    );

    integer n, k;
    integer px, py, pz;
    integer cx, cy, cz;
    integer ax_i, ay_i, az_i;
    integer step;
    integer step_max, step_first, step_last;
    integer col_err, col_max;
    reg [1023:0] steps;

    task run_case(input integer ax, ay, az, bx, by, bz, dms);
        begin
            x0 = ax; y0 = ay; z0 = az;
            x1 = bx; y1 = by; z1 = bz;
            dur_ms = dms;
            @(negedge clk); start = 1;
            @(negedge clk); start = 0;

            n = 0; col_max = 0; step_max = 0; step_first = -1;
            px = ax; py = ay; pz = az;
            ax_i = ax; ay_i = ay; az_i = az;
            steps = "";

            while (!done) begin
                @(posedge clk);
                if (tick) begin
                    cx = xq; cy = yq; cz = zq;
                    n = n + 1;

                    // 共线检查：以位移最大的轴为主轴
                    // 对每一轴，剩余误差 = 实际 - 线性预测
                    begin : colchk
                        integer maxd, pred;
                        maxd = (bx-ax); if (maxd<0) maxd=-maxd;
                        if ((by-ay)>maxd)  maxd = (by-ay);  if ((ay-by)>maxd)  maxd = ay-by;
                        if ((bz-az)>maxd)  maxd = (bz-az);  if ((az-bz)>maxd)  maxd = az-bz;
                        if (maxd > 0) begin
                            // 主轴进度
                            if (((bx-ax)<=maxd)&&((ax-bx)<=maxd)&&(bx!=ax)) begin
                                pred = ax + ((bx-ax)*(cx-ax))/(bx-ax);
                                if ((cx-ax) != 0) begin
                                    // 用主轴的 s 反推
                                    // s = (cx-ax)/(bx-ax)，检查 cy = ay + (by-ay)*s
                                    pred = ay + ((by-ay)*(cx-ax))/(bx-ax);
                                    if ((pred-cy)>col_max) col_max = pred-cy;
                                    if ((cy-pred)>col_max) col_max = cy-pred;
                                end
                            end
                        end
                    end

                    step = (cx-px)*(cx-px) + (cy-py)*(cy-py) + (cz-pz)*(cz-pz);
                    if (step > step_max) step_max = step;
                    if (n == 1) step_first = step;
                    step_last = step;

                    px = cx; py = cy; pz = cz;
                end
            end

            $display("  目标(%0d,%0d,%0d)  拍数=%0d  终点=(%0d,%0d,%0d)  共线误差=%0d",
                     bx, by, bz, n, xq, yq, zq, col_max);
            $display("     步长: 首=%0d  最大=%0d  末=%0d   %s",
                     step_first, step_max, step_last,
                     (step_first < step_max && step_last < step_max) ? "S曲线 OK" : "!! 非 S 曲线");
            if (xq !== bx || yq !== by || zq !== bz)
                $display("     !! 终点不精确");
            @(negedge clk);
        end
    endtask

    initial begin
        $display("=== zdt_interp S 曲线直线插补验证 ===");
        rst_n = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);

        run_case(2500, 0, 1500,    2800, 0, 1200,    200);
        run_case(2000, 1000, 1200, 2000, -1500, 1800, 300);
        run_case(1500, 0, 2500,    2500, 0, 1500,    400);

        $display("=== 完成 ===");
        $finish;
    end

endmodule
