//=============================================================================
// color_blob_detector.v -- 彩色物块硬件检测核心 (AXI4-Stream passthrough)
//
// VDMA(M_AXIS_MM2S) --> [本模块] --> v_axi4s_vid_out
//   数据零延迟直通, 显示不受影响; 内部对每帧做颜色统计供 PS 读取.
//
// 每色输出 7 项: cnt, sum_x, sum_y, min_x, max_x, min_y, max_y
// 质心 = sum / cnt (由 PS 计算, 硬件无除法器)
//
// 实测资源 (xc7z020clg400-2): LUT 943 (1.77%)  FF 1440 (1.35%)
//                            BRAM 0  DSP 0  WNS +2.76ns @100MHz
//=============================================================================
`default_nettype none

module color_blob_detector #(
    parameter integer H_DIV = 4,
    parameter integer V_DIV = 4,
    // ROI: 只统计这个矩形内的像素（显示坐标，1024x600）
    parameter integer ROI_X0 = 120,
    parameter integer ROI_X1 = 1016,
    parameter integer ROI_Y0 = 200,
    parameter integer ROI_Y1 = 450
)(
    input  wire        aclk,
    input  wire        aresetn,

    // ---------------- 视频输入 (来自 VDMA) ----------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TDATA" *)
    input  wire [23:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TREADY" *)
    output wire        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TLAST" *)
    input  wire        s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TUSER" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME s_axis,TDATA_NUM_BYTES 3,TDEST_WIDTH 0,TID_WIDTH 0,TUSER_WIDTH 1,HAS_TKEEP 0,HAS_TLAST 1,HAS_TREADY 1,HAS_TSTRB 0" *)
    input  wire        s_axis_tuser,

    // ---------------- 视频输出 (到 vid_out, 透传) ----------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TDATA" *)
    output wire [23:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TVALID" *)
    output wire        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TREADY" *)
    input  wire        m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TLAST" *)
    output wire        m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TUSER" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME m_axis,TDATA_NUM_BYTES 3,TDEST_WIDTH 0,TID_WIDTH 0,TUSER_WIDTH 1,HAS_TKEEP 0,HAS_TLAST 1,HAS_TREADY 1,HAS_TSTRB 0" *)
    output wire        m_axis_tuser,

    // ---------------- 结果读取 (接 AXI GPIO 双通道) ----------------
    input  wire [7:0]  rd_sel,      // [2:0]=color  [5:3]=field
    output reg  [31:0] rd_data,

    // ---------------- 状态 ----------------
    output reg  [31:0] frame_cnt,
    output reg         done_toggle
);

    //=========================================================================
    // 0. 零延迟直通
    //=========================================================================
    assign m_axis_tvalid = s_axis_tvalid;
    assign m_axis_tuser  = s_axis_tuser;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tdata  = s_axis_tdata;
    assign s_axis_tready = m_axis_tready;

    wire xfer = s_axis_tvalid && m_axis_tready;

    //=========================================================================
    // 1. 像素坐标计数
    //=========================================================================
    reg [15:0] x_cnt, y_cnt;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            x_cnt <= 16'd0;
            y_cnt <= 16'd0;
        end else if (xfer) begin
            if (s_axis_tuser) begin
                x_cnt <= 16'd0;
                y_cnt <= 16'd0;
            end else if (s_axis_tlast) begin
                x_cnt <= 16'd0;
                y_cnt <= y_cnt + 16'd1;
            end else begin
                x_cnt <= x_cnt + 16'd1;
            end
        end
    end

    //=========================================================================
    // 2. 颜色分类
    //=========================================================================
    wire [7:0] R = s_axis_tdata[23:16];
    wire [7:0] G = s_axis_tdata[15:8];
    wire [7:0] B = s_axis_tdata[7:0];

    wire [7:0] mx = (R > G) ? ((R > B) ? R : B) : ((G > B) ? G : B);
    wire [7:0] mn = (R < G) ? ((R < B) ? R : B) : ((G < B) ? G : B);
    wire [7:0] satura = mx - mn;

    reg [2:0] cls;      // 0=R 1=Y 2=G 3=B 4=K 5=background
    always @* begin
        if (mx < 8'd90 && satura < 8'd40)
            cls = 3'd4;
        else if (R > G + 8'd45 && R > B + 8'd35)
            cls = 3'd0;
        else if (R > 8'd110 && G > 8'd90 && B < 8'd110 && R >= G)
            cls = 3'd1;
        else if (G > R + 8'd25 && G > B + 8'd20)
            cls = 3'd2;
        else if (B > R + 8'd35 && B > G + 8'd15)
            cls = 3'd3;
        else
            cls = 3'd5;
    end

    //=========================================================================
    // 3. 采样判定 (打一拍, 切断长路径)
    //=========================================================================
    reg [2:0]  cls_s;
    reg        samp_s;
    reg        sof_s;
    reg [15:0] xs, ys;

    wire in_roi = (x_cnt >= ROI_X0) && (x_cnt <= ROI_X1)
               && (y_cnt >= ROI_Y0) && (y_cnt <= ROI_Y1);

    wire samp_c = xfer && (cls != 3'd5) && in_roi
                  && ((x_cnt % H_DIV) == 0) && ((y_cnt % V_DIV) == 0);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            cls_s  <= 3'd5;
            samp_s <= 1'b0;
            sof_s  <= 1'b0;
            xs     <= 16'd0;
            ys     <= 16'd0;
        end else begin
            cls_s  <= cls;
            samp_s <= samp_c;
            sof_s  <= xfer && s_axis_tuser;
            xs     <= x_cnt;
            ys     <= y_cnt;
        end
    end

    //=========================================================================
    // 4. 统计累加器
    //=========================================================================
    reg [15:0] acc_cnt [0:4];
    reg [23:0] acc_sx  [0:4];
    reg [23:0] acc_sy  [0:4];
    reg [15:0] acc_x0  [0:4];
    reg [15:0] acc_x1  [0:4];
    reg [15:0] acc_y0  [0:4];
    reg [15:0] acc_y1  [0:4];

    reg [15:0] res_cnt [0:4];
    reg [23:0] res_sx  [0:4];
    reg [23:0] res_sy  [0:4];
    reg [15:0] res_x0  [0:4];
    reg [15:0] res_x1  [0:4];
    reg [15:0] res_y0  [0:4];
    reg [15:0] res_y1  [0:4];

    integer i;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            frame_cnt   <= 32'd0;
            done_toggle <= 1'b0;
            for (i = 0; i < 5; i = i + 1) begin
                acc_cnt[i] <= 16'd0;    acc_sx[i] <= 24'd0;  acc_sy[i] <= 24'd0;
                acc_x0[i]  <= 16'hFFFF; acc_x1[i] <= 16'd0;
                acc_y0[i]  <= 16'hFFFF; acc_y1[i] <= 16'd0;
                res_cnt[i] <= 16'd0;    res_sx[i] <= 24'd0;  res_sy[i] <= 24'd0;
                res_x0[i]  <= 16'd0;    res_x1[i] <= 16'd0;
                res_y0[i]  <= 16'd0;    res_y1[i] <= 16'd0;
            end
        end else begin
            if (sof_s) begin
                frame_cnt   <= frame_cnt + 32'd1;
                done_toggle <= ~done_toggle;
                for (i = 0; i < 5; i = i + 1) begin
                    res_cnt[i] <= acc_cnt[i];
                    res_sx[i]  <= acc_sx[i];
                    res_sy[i]  <= acc_sy[i];
                    res_x0[i]  <= (acc_cnt[i] == 16'd0) ? 16'd0 : acc_x0[i];
                    res_x1[i]  <= (acc_cnt[i] == 16'd0) ? 16'd0 : acc_x1[i];
                    res_y0[i]  <= (acc_cnt[i] == 16'd0) ? 16'd0 : acc_y0[i];
                    res_y1[i]  <= (acc_cnt[i] == 16'd0) ? 16'd0 : acc_y1[i];
                    acc_cnt[i] <= 16'd0;
                    acc_sx[i]  <= 24'd0;
                    acc_sy[i]  <= 24'd0;
                    acc_x0[i]  <= 16'hFFFF;
                    acc_x1[i]  <= 16'd0;
                    acc_y0[i]  <= 16'hFFFF;
                    acc_y1[i]  <= 16'd0;
                end
            end else if (samp_s) begin
                if (acc_cnt[cls_s] == 16'd0) begin
                    acc_x0[cls_s] <= xs;  acc_x1[cls_s] <= xs;
                    acc_y0[cls_s] <= ys;  acc_y1[cls_s] <= ys;
                end else begin
                    if (xs < acc_x0[cls_s]) acc_x0[cls_s] <= xs;
                    if (xs > acc_x1[cls_s]) acc_x1[cls_s] <= xs;
                    if (ys < acc_y0[cls_s]) acc_y0[cls_s] <= ys;
                    if (ys > acc_y1[cls_s]) acc_y1[cls_s] <= ys;
                end
                acc_cnt[cls_s] <= acc_cnt[cls_s] + 16'd1;
                acc_sx[cls_s]  <= acc_sx[cls_s] + {8'd0, xs};
                acc_sy[cls_s]  <= acc_sy[cls_s] + {8'd0, ys};
            end
        end
    end

    //=========================================================================
    // 5. 读取通道
    //=========================================================================
    wire [2:0] rd_color = rd_sel[2:0];
    wire [2:0] rd_field = rd_sel[5:3];

    always @* begin
        rd_data = 32'd0;
        if (rd_color < 3'd5) begin
            case (rd_field)
                3'd0: rd_data = {16'd0, res_cnt[rd_color]};
                3'd1: rd_data = {8'd0,  res_sx[rd_color]};
                3'd2: rd_data = {8'd0,  res_sy[rd_color]};
                3'd3: rd_data = {16'd0, res_x0[rd_color]};
                3'd4: rd_data = {16'd0, res_x1[rd_color]};
                3'd5: rd_data = {16'd0, res_y0[rd_color]};
                3'd6: rd_data = {16'd0, res_y1[rd_color]};
                default: rd_data = 32'd0;
            endcase
        end
    end

endmodule

`default_nettype wire
