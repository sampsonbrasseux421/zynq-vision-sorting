//=============================================================================
// zdt_interp.v -- 笛卡尔直线插补 + 五次 S 曲线速度规划
//
// 从 (x0,y0,z0) 走到 (x1,y1,z1)，共 N 个节拍，每拍输出中间点 + tick 脉冲。
//
//   位置：p(t) = p0 + (p1-p0) * s(u),  u = n/N   (Q16)
//   S 曲线：s(u) = 10u^3 - 15u^4 + 6u^5
//           s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
//           首尾速度与加速度均为 0，无冲击
//=============================================================================
`default_nettype none

module zdt_interp #(
    parameter CLK_HZ  = 50000000,
    parameter TICK_MS = 20
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire               start,
    input  wire signed [15:0] x0, y0, z0,
    input  wire signed [15:0] x1, y1, z1,
    input  wire        [15:0] dur_ms,

    output reg                tick,
    output reg  signed [15:0] xq, yq, zq,
    output reg                busy,
    output reg                done
);

    localparam integer TICK_CYC = (CLK_HZ / 1000) * TICK_MS;

    //------------------------------------------------------------------
    reg [31:0] tick_cnt;
    wire       tick_now = (tick_cnt == TICK_CYC - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tick_cnt <= 32'd0;
        else if (busy) begin
            if (tick_now) tick_cnt <= 32'd0;
            else          tick_cnt <= tick_cnt + 32'd1;
        end else
            tick_cnt <= 32'd0;
    end

    //------------------------------------------------------------------
    reg [15:0] n_total;
    reg [15:0] n_cur;

    reg signed [15:0] sx0, sy0, sz0;
    reg signed [16:0] dx, dy, dz;

    // 下一拍的 u（33 位中间量，避免溢出）
    wire [32:0] n_next  = {17'd0, n_cur} + 33'd1;
    wire [32:0] u_next  = (33'd65536 * n_next) / n_total;
    wire        u_last  = (n_next >= {17'd0, n_total});

    // S 曲线（用 u_next 计算，与本拍输出的坐标同步）
    wire [16:0] u_q16 = u_last ? 17'd65536 : u_next[16:0];

    wire [63:0] u2 = u_q16 * u_q16;                       // Q32
    wire [63:0] u3 = ((u2 >> 16) * u_q16) >> 16;          // Q16
    wire [63:0] u4 = ((u3 * u_q16) >> 16);                // Q16
    wire [63:0] u5 = ((u4 * u_q16) >> 16);                // Q16

    wire signed [63:0] s_full =
        64'sd10 * $signed({1'b0, u3}) -
        64'sd15 * $signed({1'b0, u4}) +
        64'sd6  * $signed({1'b0, u5});

    wire signed [63:0] s_clamp =
        (s_full < 0)     ? 64'sd0 :
        (s_full > 65536) ? 64'sd65536 : s_full;

    // s 最大为 65536，17 位有符号装不下（会变成 -65536），用 18 位
    wire signed [17:0] s_q16 = s_clamp[17:0];

    // 中间点
    wire signed [35:0] px = $signed(dx) * s_q16;
    wire signed [35:0] py = $signed(dy) * s_q16;
    wire signed [35:0] pz = $signed(dz) * s_q16;

    wire signed [16:0] nx17 = $signed({sx0[15], sx0}) + $signed(px[32:16]);
    wire signed [16:0] ny17 = $signed({sy0[15], sy0}) + $signed(py[32:16]);
    wire signed [16:0] nz17 = $signed({sz0[15], sz0}) + $signed(pz[32:16]);
    wire signed [15:0] nx = nx17[15:0];
    wire signed [15:0] ny = ny17[15:0];
    wire signed [15:0] nz = nz17[15:0];

    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy    <= 1'b0;
            done    <= 1'b0;
            tick    <= 1'b0;
            n_cur   <= 16'd0;
            n_total <= 16'd1;
            xq <= 16'sd0; yq <= 16'sd0; zq <= 16'sd0;
            sx0 <= 16'sd0; sy0 <= 16'sd0; sz0 <= 16'sd0;
            dx  <= 17'sd0; dy  <= 17'sd0; dz  <= 17'sd0;
        end else begin
            tick <= 1'b0;

            if (start && !busy) begin
                sx0 <= x0; sy0 <= y0; sz0 <= z0;
                dx  <= $signed({x1[15], x1}) - $signed({x0[15], x0});
                dy  <= $signed({y1[15], y1}) - $signed({y0[15], y0});
                dz  <= $signed({z1[15], z1}) - $signed({z0[15], z0});
                n_total <= (dur_ms < TICK_MS) ? 16'd1 : (dur_ms / TICK_MS);
                n_cur   <= 16'd0;
                xq <= x0; yq <= y0; zq <= z0;
                busy  <= 1'b1;
                done  <= 1'b0;
            end
            else if (busy && tick_now) begin
                n_cur <= n_cur + 16'd1;
                xq <= nx; yq <= ny; zq <= nz;
                tick <= 1'b1;
                if (u_last) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
