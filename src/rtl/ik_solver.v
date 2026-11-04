//=============================================================================
// ik_solver.v —— 四轴机械臂逆运动学硬件求解器
//
// 算法与 ik_zdt.c 的 ik_solve() 一致；定点模型已用 Python 验证（误差 0.04 度）。
//
// 阶段序列（stage）：
//   0 : 启动 sqrt(x^2+y^2)
//   1 : 得 rr；发起 atan2(y, x)                      -> ang_base
//   2 : 得 ang_base；算 dx/dz/d2/c2；启动 sqrt(1-c2^2)
//   3 : 得 s2；发起 atan2(s2, c2)                    -> ang_th2
//   4 : 得 ang_th2；发起 atan2(k2, k1)               -> ang_k
//   5 : 得 ang_k；发起 atan2(dz, dx)                 -> ang_dz
//   6 : 得 ang_dz；进入分拍角度换算与输出整理
//   9 : 角度转 deg256
//  10 : 计算 j1 环绕值、j2 差值和 j3 归一化值
//  11 : 计算 j2 环绕值和 j4 差值
//  12 : 计算 j4 环绕值
//  13 : 同拍提交 j1..j4，置 done，回 0
//
// 定点：输入 0.1mm (s16)，输出 deg256 (s32)，内部角度 Q16 弧度
//=============================================================================
`default_nettype none

module ik_solver #(
    parameter signed [31:0] P_TOOL_X = 350,      // TOOL_X = 35.0 mm
    parameter signed [31:0] P_BASE_H = 1120,     // BASE_H = 112.0 mm
    parameter signed [31:0] P_TOOL_Z = 350,      // TOOL_Z = 35.0 mm
    parameter signed [31:0] P_L1     = 2000,     // LINK1 = 200.0 mm
    parameter signed [31:0] P_L2     = 1750,     // LINK2 = 175.0 mm
    parameter signed [31:0] P_J1_OFF = 0,
    parameter signed [31:0] P_J2_OFF = -23040,   // J2Z
    parameter signed [31:0] P_J3_OFF = 23040,    // J3Z
    parameter signed [31:0] P_J4_OFF = 1280,    // J4Z
    parameter signed [31:0] P_ELBOW  = -1       // ELBOW = -1
) (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                start,
    input  wire signed [15:0]  x_mm10,
    input  wire signed [15:0]  y_mm10,
    input  wire signed [15:0]  z_mm10,
    output reg  signed [31:0]  j1_deg256,
    output reg  signed [31:0]  j2_deg256,
    output reg  signed [31:0]  j3_deg256,
    output reg  signed [31:0]  j4_deg256,
    output reg                 busy,
    output reg                 done,
    output reg                 err
);

    localparam signed [31:0] LSUMSQ  = P_L1 * P_L1 + P_L2 * P_L2;
    localparam signed [31:0] BASE_HZ = P_BASE_H - P_TOOL_Z;
    localparam signed [31:0] PI_Q16  = 205888;
    localparam signed [31:0] FULL_D  = 92160;

    //------------------------------------------------------------------
    // 整数开方
    //------------------------------------------------------------------
    reg  [31:0] sq_arg;          // 被开方数（由 FSM 提供）
    reg         sq_start;        // 1 拍脉冲
    reg  [31:0] sq_x;
    reg  [17:0] sq_rem;
    reg  [15:0] sq_root;
    reg  [4:0]  sq_i;
    reg         sq_run, sq_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sq_run <= 1'b0; sq_done <= 1'b0;
            sq_x <= 0; sq_rem <= 0; sq_root <= 0; sq_i <= 0;
        end else begin
            sq_done <= 1'b0;
            if (sq_start) begin
                sq_x    <= sq_arg;
                sq_rem  <= 18'd0;
                sq_root <= 16'd0;
                sq_i    <= 5'd0;
                sq_run  <= 1'b1;
            end else if (sq_run) begin
                if (sq_i == 5'd16) begin
                    sq_run  <= 1'b0;
                    sq_done <= 1'b1;
                end else begin
                    sq_i <= sq_i + 1'b1;
                    if ({sq_rem[15:0], sq_x[31:30]} >= {sq_root, 2'b01}) begin
                        sq_rem  <= {sq_rem[15:0], sq_x[31:30]} - {sq_root, 2'b01};
                        sq_root <= {sq_root[14:0], 1'b1};
                    end else begin
                        sq_rem  <= {sq_rem[15:0], sq_x[31:30]};
                        sq_root <= {sq_root[14:0], 1'b0};
                    end
                    sq_x <= {sq_x[29:0], 2'b00};
                end
            end
        end
    end

    //------------------------------------------------------------------
    // CORDIC atan2
    //------------------------------------------------------------------
    function signed [23:0] atan_tbl(input [4:0] i);
        case (i)
            5'd0: atan_tbl = 24'sd51472;  5'd1: atan_tbl = 24'sd30386;
            5'd2: atan_tbl = 24'sd16055;  5'd3: atan_tbl = 24'sd8149;
            5'd4: atan_tbl = 24'sd4091;   5'd5: atan_tbl = 24'sd2047;
            5'd6: atan_tbl = 24'sd1024;   5'd7: atan_tbl = 24'sd512;
            5'd8: atan_tbl = 24'sd256;    5'd9: atan_tbl = 24'sd128;
            5'd10: atan_tbl = 24'sd64;    5'd11: atan_tbl = 24'sd32;
            5'd12: atan_tbl = 24'sd16;    5'd13: atan_tbl = 24'sd8;
            5'd14: atan_tbl = 24'sd4;     default: atan_tbl = 24'sd2;
        endcase
    endfunction

    reg  signed [23:0] req_x, req_y;

    // 17 位绝对值（先扩位，再进 24 位，避免 16 位左移溢出）
    wire signed [16:0] y_abs17 = y_mm10[15] ? -$signed(y_mm10) : $signed(y_mm10);
    reg                cordic_req;

    wire signed [23:0] pr_x = req_x[23] ? -req_x : req_x;
    wire signed [23:0] pr_y = req_x[23] ? -req_y : req_y;
    wire signed [31:0] pr_z = req_x[23] ? (req_y[23] ? -PI_Q16 : PI_Q16) : 32'sd0;

    reg  signed [23:0] cx, cy;
    reg  signed [31:0] cz;
    reg  [4:0]         ci;
    reg                cordic_run, cordic_done;
    reg  signed [31:0] cordic_ang;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cx <= 0; cy <= 0; cz <= 0; ci <= 0;
            cordic_run <= 1'b0; cordic_done <= 1'b0; cordic_ang <= 0;
        end else begin
            cordic_done <= 1'b0;
            if (cordic_req) begin
                cx <= pr_x; cy <= pr_y; cz <= pr_z;
                ci <= 5'd0;
                cordic_run <= 1'b1;
            end else if (cordic_run) begin
                if (ci == 5'd16) begin
                    cordic_run  <= 1'b0;
                    cordic_done <= 1'b1;
                    if (cz > PI_Q16)       cordic_ang <= cz - 2 * PI_Q16;
                    else if (cz < -PI_Q16) cordic_ang <= cz + 2 * PI_Q16;
                    else                   cordic_ang <= cz;
                end else begin
                    if (cy[23]) cz <= cz - atan_tbl(ci);
                    else        cz <= cz + atan_tbl(ci);
                    if (cy[23]) begin
                        cx <= cx - (cy >>> ci);
                        cy <= cy + (cx >>> ci);
                    end else begin
                        cx <= cx + (cy >>> ci);
                        cy <= cy - (cx >>> ci);
                    end
                    ci <= ci + 1'b1;
                end
            end
        end
    end

    //------------------------------------------------------------------
    // 换算函数
    //------------------------------------------------------------------
    function signed [31:0] to_deg(input signed [31:0] q16);
        reg signed [63:0] m;
        begin
            m = q16 * 14668;
            to_deg = m >>> 16;
        end
    endfunction

    // 绕回到 [-180,180) 度（deg256）
    // 不用取模：负数取模在 Verilog / 综合工具间语义不一致，
    // 曾导致 y<0 时角度全部塌成 ~0。这里只用加减。
    function signed [31:0] wrap_deg(input signed [31:0] d);
        reg signed [31:0] t;
        begin
            t = d;
            if (t >= FULL_D)       t = t - FULL_D;
            else if (t < -FULL_D)  t = t + FULL_D;
            if (t >= 46080)        t = t - FULL_D;
            else if (t < -46080)   t = t + FULL_D;
            wrap_deg = t;
        end
    endfunction

    //------------------------------------------------------------------
    // 主序列
    //------------------------------------------------------------------
    reg [3:0]  stage;
    reg        ik_yneg;         // y < 0：镜像到正半轴，结果取反
    reg        ik_dzneg;        // dz < 0：同上
    reg signed [31:0] rr, dx, dz;
    reg signed [63:0] d2prod;
    reg signed [31:0] c2, s2;
    reg               sat;
    reg signed [31:0] ang_base, ang_th2, ang_k, ang_dz;
    reg signed [31:0] a1_v, a2p_v;      // 肩角（有符号）、肘角（规范到 0..180）
    reg signed [31:0] deg_base, deg_th2, deg_k, deg_dz;
    reg signed [31:0] j1_wrapped, j2_delta, j2_wrapped;
    reg signed [31:0] j3_normalized, j4_delta, j4_wrapped;

    // c2 定标：c2_q15 = (d2 - L1^2 - L2^2) / (2*L1*L2) * 32768
    //            = (num * C2K) >> 16 ，其中 C2K = round(32768*65536/(2*L1*L2))
    localparam signed [63:0] C2K = (64'sd2147483648 + (P_L1 * P_L2)) / (P_L1 * P_L2 * 2);
    wire signed [63:0] c2_prod = (d2prod - LSUMSQ) * C2K;
    wire signed [31:0] c2_w    = c2_prod >>> 16;
    wire               sat_w   = (c2_w >= 32767) || (c2_w <= -32767);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage <= 3'd0; busy <= 1'b0; done <= 1'b0; err <= 1'b0;
            cordic_req <= 1'b0;
            j1_deg256 <= 0; j2_deg256 <= 0; j3_deg256 <= 0; j4_deg256 <= 0;
            deg_base <= 0; deg_th2 <= 0; deg_k <= 0; deg_dz <= 0;
            j1_wrapped <= 0; j2_delta <= 0; j2_wrapped <= 0;
            j3_normalized <= 0; j4_delta <= 0; j4_wrapped <= 0;
        end else begin
            done       <= 1'b0;
            cordic_req <= 1'b0;
            sq_start   <= 1'b0;

            case (stage)
            // 0 空闲
            4'd0: if (start) begin
                busy  <= 1'b1; err <= 1'b0;
                sq_arg   <= (x_mm10 * x_mm10) + (y_mm10 * y_mm10);
                sq_start <= 1'b1;
                stage <= 4'd1;
            end

            // 1 得 rr；发起 atan2(y,x)
            4'd1: if (sq_done) begin
                rr    <= $signed(sq_root);
                ik_yneg <= y_mm10[15];
                req_x   <= {{8{x_mm10[15]}}, x_mm10} <<< 6;
                req_y   <= {{7{y_abs17[16]}}, y_abs17} <<< 6;
                cordic_req <= 1'b1;
                stage <= 4'd2;
            end

            // 2 得 ang_base；登记 dx/dz/d2prod
            4'd2: if (cordic_done) begin
                ang_base <= ik_yneg ? -cordic_ang : cordic_ang;
                dx       <= rr - P_TOOL_X;
                dz       <= z_mm10 - BASE_HZ;
                d2prod   <= (rr - P_TOOL_X) * (rr - P_TOOL_X)
                          + (z_mm10 - BASE_HZ) * (z_mm10 - BASE_HZ);
                stage <= 4'd3;
            end

            // 3 用已登记的 d2prod 算 c2（此时 d2prod 已稳定）
            4'd3: begin
                c2  <= c2_w;
                sat <= sat_w;
                if (sat_w) begin
                    s2    <= 0;
                    stage <= 4'd5;
                end else begin
                    sq_arg   <= 32'd1073741824 - (c2_w * c2_w);
                    sq_start <= 1'b1;
                    stage <= 4'd4;
                end
            end

            // 4 得 s2
            4'd4: if (sq_done) begin
                s2    <= $signed(sq_root);
                stage <= 4'd5;
            end

            // 5 发起 atan2(s2, c2)（acos）
            4'd5: begin
                req_x <= c2 <<< 6;
                req_y <= s2 <<< 6;
                cordic_req <= 1'b1;
                stage <= 4'd6;
            end

            // 6 得 th2；发起 atan2(k2, k1)
            4'd6: if (cordic_done) begin
                ang_th2 <= cordic_ang;
                req_x <= (P_L1 + ((P_L2 * c2) >>> 15)) <<< 6;
                req_y <= ((P_ELBOW * P_L2 * s2) >>> 15) <<< 6;
                cordic_req <= 1'b1;
                stage <= 4'd7;
            end

            // 7 得 ang_k；发起 atan2(dz, dx)
            4'd7: if (cordic_done) begin
                ang_k    <= cordic_ang;
                ik_dzneg <= dz[31];
                req_x    <= dx <<< 6;
                req_y    <= (dz[31] ? -$signed(dz) : $signed(dz)) <<< 6;
                cordic_req <= 1'b1;
                stage <= 4'd8;
            end

            // 8 得 ang_dz；后续算术分拍，缩短单周期组合路径
            4'd8: if (cordic_done) begin
                ang_dz <= ik_dzneg ? -cordic_ang : cordic_ang;
                stage <= 4'd9;
            end

            // 9 独立寄存每个角度换算，避免串接到输出加法与环绕
            4'd9: begin
                deg_base <= to_deg(ang_base);
                deg_th2  <= to_deg(ang_th2);
                deg_k    <= to_deg(ang_k);
                deg_dz   <= to_deg(ang_dz);
                stage <= 4'd10;
            end

            // 10 j1 环绕、j2 差值、j3 角度归一化
            4'd10: begin
                j1_wrapped    <= wrap_deg(deg_base);
                j2_delta      <= deg_dz - deg_k;
                j3_normalized <= (deg_th2 < 0) ? (deg_th2 + FULL_D) : deg_th2;
                stage <= 4'd11;
            end

            // 11 j2 环绕与 j4 的未环绕角度和
            4'd11: begin
                j2_wrapped <= wrap_deg(j2_delta);
                j4_delta   <= j2_delta + P_ELBOW * j3_normalized;
                stage <= 4'd12;
            end

            // 12 j4 环绕
            4'd12: begin
                j4_wrapped <= wrap_deg(j4_delta);
                stage <= 4'd13;
            end

            // 13 原子提交全部关节角
            4'd13: begin
                j1_deg256 <= j1_wrapped + P_J1_OFF;
                j2_deg256 <= j2_wrapped + P_J2_OFF;
                j3_deg256 <= P_ELBOW * j3_normalized + P_J3_OFF;
                j4_deg256 <= j4_wrapped + P_J4_OFF;
                busy  <= 1'b0;
                done  <= 1'b1;
                stage <= 4'd0;
            end
            default: stage <= 4'd0;
            endcase
        end
    end

endmodule

`default_nettype wire
