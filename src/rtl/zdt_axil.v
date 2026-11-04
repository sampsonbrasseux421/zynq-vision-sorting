//=============================================================================
// zdt_axil.v —— 四轴 ZDT 发送引擎的 AXI4-Lite 寄存器接口
//
// 寄存器映射（字地址）：
//   0x00  CTRL    [rw]  bit0 start（写 1 触发，单拍自清）
//                       bit1 abs_rel
//                       bit2 sync_en
//   0x04  TIMING  [rw]  [15:0] speed, [23:16] accel
//   0x08  POS1    [rw]  signed 32
//   0x0C  POS2    [rw]
//   0x10  POS3    [rw]
//   0x14  POS4    [rw]
//   0x18  STATUS  [ro]  bit0 busy, bit1 done(粘滞), [5:3] cur_joint, [31:16] frame_count
//   0x1C  CLR     [wo]  写 1 清 done
//   0x20  GAP     [rw]  帧间隔 ms
//   0x24/28/2C   [rw]  IK X/Y/Z，signed 16，单位 0.1mm
//   0x30  IK_START [wo] bit0 写 1 且空闲时执行一次解算
//   0x34  IK_ANGLE [rw] 写关节索引 1/2/3/4（4 截为 0），读 signed deg256
//   0x38  IK_ID    [ro] 0x494B0002，识别本次 AXI/IK 固件
//   0x3C  IK_STATUS[ro] bit0 busy, bit1 done(粘滞), bit2 err
//   0x40  INTERP_DUR[rw] 插补总时长 ms
//
// AW/W 两个独立通道分别缓存；配对提交后等待 B 响应取走。
//=============================================================================
`default_nettype none

module zdt_axil #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200,
    parameter integer INTERP_TICK_MS = 50
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk,ASSOCIATED_BUSIF S_AXI,ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn,POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    // ---------------- AXI4-Lite 从机 ----------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI,PROTOCOL AXI4LITE,DATA_WIDTH 32,ADDR_WIDTH 8,NUM_READ_OUTSTANDING 1,NUM_WRITE_OUTSTANDING 1,READ_WRITE_MODE READ_WRITE,HAS_BURST 0,HAS_LOCK 0,HAS_CACHE 0,HAS_REGION 0,HAS_QOS 0,HAS_PROT 0" *)
    input  wire [7:0]  s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input  wire        s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output wire        s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input  wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input  wire [3:0]  s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input  wire        s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output wire        s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output reg  [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output reg         s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire        s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [7:0]  s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire        s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output reg  [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output reg  [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output reg         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire        s_axi_rready,

    // ---------------- 串行输出 ----------------
    output wire        tx,
    // 串口通路选择：0=软件 axi_uartlite，1=PL 硬件引擎（CTRL bit3）
    output wire        uart_sel,
    // OSD 状态输出
    output wire [15:0] status_frames,
    output wire        status_busy,
    output wire        status_done
);

    assign uart_sel      = reg_ctrl[3];
    assign status_frames = frame_count;
    assign status_busy   = busy;
    assign status_done   = done_sticky;

    reg aw_pending, w_pending;
    reg [7:0] wr_addr;
    reg [31:0] wr_data;
    reg [3:0] wr_strb;

    assign s_axi_awready = aresetn && !aw_pending && !s_axi_bvalid;
    assign s_axi_wready  = aresetn && !w_pending && !s_axi_bvalid;
    assign s_axi_arready = ~s_axi_rvalid;

    wire wr_fire = aw_pending && w_pending && !s_axi_bvalid;
    wire rd_fire = s_axi_arvalid && !s_axi_rvalid;

    function [31:0] merge_wstrb;
        input [31:0] previous;
        input [31:0] value;
        input [3:0] strobes;
        integer byte_index;
        begin
            merge_wstrb = previous;
            for (byte_index = 0; byte_index < 4; byte_index = byte_index + 1)
                if (strobes[byte_index])
                    merge_wstrb[byte_index*8 +: 8] = value[byte_index*8 +: 8];
        end
    endfunction

    //------------------------------------------------------------------
    // 寄存器
    //------------------------------------------------------------------
    reg [31:0] reg_ctrl;
    reg [31:0] reg_timing;
    reg [31:0] reg_pos1, reg_pos2, reg_pos3, reg_pos4;
    reg [31:0] reg_gap;          /* 0x20: 帧间隔 ms */
    reg        done_sticky;
    reg        clr_pulse;

    // ---------------- 逆运动学硬件求解器 ----------------
    reg  signed [15:0] ik_x, ik_y, ik_z;
    wire               ik_start;
    reg  [1:0]         ik_idx;
    wire signed [31:0] ik_j1, ik_j2, ik_j3, ik_j4;
    wire               ik_busy, ik_done, ik_err;
    reg                ik_done_sticky;

    ik_solver u_ik (
        .clk       (aclk),
        .rst_n     (aresetn),
        .start     (ik_start),
        .x_mm10    (ik_x),
        .y_mm10    (ik_y),
        .z_mm10    (ik_z),
        .j1_deg256 (ik_j1),
        .j2_deg256 (ik_j2),
        .j3_deg256 (ik_j3),
        .j4_deg256 (ik_j4),
        .busy      (ik_busy),
        .done      (ik_done),
        .err       (ik_err)
    );

    // ================== 笛卡尔直线插补（S 曲线）==================
    reg  signed [15:0] itp_x0, itp_y0, itp_z0;   // 起点（上一次命令的位置）

    // 起点跟随：普通 PL 移动(IK 自动) 或 插补段结束后，更新为当前目标
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            itp_x0 <= 16'sd2500; itp_y0 <= 16'sd0; itp_z0 <= 16'sd1500;
        end else if (ik_auto_req || itp_seg_end) begin
            itp_x0 <= ik_x; itp_y0 <= ik_y; itp_z0 <= ik_z;
        end
    end
    reg         [15:0] reg_dur;                  // 0x40 插补总时长 ms
    wire signed [15:0] itp_xq, itp_yq, itp_zq;   // 插补输出的中间点
    wire               itp_tick, itp_busy, itp_done;

    zdt_interp #(.CLK_HZ(CLK_HZ), .TICK_MS(INTERP_TICK_MS)) u_interp (
        .clk(aclk), .rst_n(aresetn),
        .start(reg_ctrl[6] & ~itp_busy & ik_start),
        .x0(itp_x0), .y0(itp_y0), .z0(itp_z0),
        .x1(ik_x), .y1(ik_y), .z1(ik_z),
        .dur_ms(reg_dur),
        .tick(itp_tick), .xq(itp_xq), .yq(itp_yq), .zq(itp_zq),
        .busy(itp_busy), .done(itp_done)
    );

    // 插补点 -> 逆解（第二个 ik_solver 实例）
    reg  ik2_start;
    wire ik2_busy, ik2_done, ik2_err;
    wire signed [31:0] ik2_j1, ik2_j2, ik2_j3, ik2_j4;

    ik_solver u_ik_interp (
        .clk(aclk), .rst_n(aresetn), .start(ik2_start),
        .x_mm10(itp_xq), .y_mm10(itp_yq), .z_mm10(itp_zq),
        .j1_deg256(ik2_j1), .j2_deg256(ik2_j2),
        .j3_deg256(ik2_j3), .j4_deg256(ik2_j4),
        .busy(ik2_busy), .done(ik2_done), .err(ik2_err)
    );

    // 插补节拍状态机：tick -> 逆解 -> 装载 -> 启动引擎
    localparam [1:0] IS_IDLE = 2'd0, IS_IK = 2'd1, IS_SEND = 2'd2;
    reg [1:0] istate;

    reg signed [31:0] ipos1, ipos2, ipos3, ipos4;
    reg               eng_go;
    reg               itp_seg_end;   // 插补段结束（1 拍）

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            istate  <= IS_IDLE;
            itp_seg_end <= 1'b0;
            ik2_start <= 1'b0;
            eng_go  <= 1'b0;
            ipos1 <= 32'd0; ipos2 <= 32'd0; ipos3 <= 32'd0; ipos4 <= 32'd0;
        end else begin
            ik2_start  <= 1'b0;
            eng_go     <= 1'b0;
            itp_seg_end <= itp_done;
            itp_seg_end <= 1'b0;
            case (istate)
                IS_IDLE: if (itp_tick && !busy) begin
                    ik2_start <= 1'b1;
                    istate    <= IS_IK;
                end
                IS_IK: if (ik2_done) begin
                    ipos1 <= deg2pulse(ik2_j1, PPR1);
                    ipos2 <= deg2pulse(ik2_j2, PPR2);
                    ipos3 <= deg2pulse(ik2_j3, PPR3);
                    ipos4 <= deg2pulse(ik2_j4, PPR4);
                    eng_go <= 1'b1;
                    istate <= IS_IDLE;      // 不等引擎，按节拍推进
                end
                default: istate <= IS_IDLE;
            endcase
        end
    end

    // ---------------- 角度 -> 脉冲（PL 内完成）----------------
    // pulse = round(deg256 * PPR / 92160)
    //       = (deg256 * PPR * K) >>> 32   ，K = round(2^32/92160) = 46603
    localparam signed [31:0] PPR1 = 418400;   // /etc/ik.conf P1
    localparam signed [31:0] PPR2 = 406600;
    localparam signed [31:0] PPR3 = 223400;
    localparam signed [31:0] PPR4 = 27550;
    localparam signed [63:0] PULSE_K = 64'sd46603;

    function signed [31:0] deg2pulse(input signed [31:0] d, input signed [31:0] ppr);
        reg signed [63:0] m;
        begin
            m = ($signed(d) * $signed(ppr) * PULSE_K + 64'sd2147483648) >>> 32;
            deg2pulse = m[31:0];
        end
    endfunction

    wire signed [31:0] pulse1 = deg2pulse(ik_j1, PPR1);
    wire signed [31:0] pulse2 = deg2pulse(ik_j2, PPR2);
    wire signed [31:0] pulse3 = deg2pulse(ik_j3, PPR3);
    wire signed [31:0] pulse4 = deg2pulse(ik_j4, PPR4);

    // CTRL bit5 = IK 自动模式：求解完成自动装载 POS 并启动组帧
    reg ik_auto_req;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn)
            ik_auto_req <= 1'b0;
        else if (ik_done && reg_ctrl[5])
            ik_auto_req <= 1'b1;
        else
            ik_auto_req <= 1'b0;
    end

    // 送给引擎的 POS：IK 自动模式用 PL 算的脉冲，否则用 PS 写的
    // CTRL bit5=1 时常驻使用 PL 算出的脉冲。
    // 不能只用 1 拍的 ik_auto_req：第一帧之后会退回旧 POS，后续帧就发错了。
    wire signed [31:0] pos1_eff = reg_ctrl[6] ? ipos1 :
                                  reg_ctrl[5] ? pulse1 : reg_pos1;
    wire signed [31:0] pos2_eff = reg_ctrl[6] ? ipos2 :
                                  reg_ctrl[5] ? pulse2 : reg_pos2;
    wire signed [31:0] pos3_eff = reg_ctrl[6] ? ipos3 :
                                  reg_ctrl[5] ? pulse3 : reg_pos3;
    wire signed [31:0] pos4_eff = reg_ctrl[6] ? ipos4 :
                                  reg_ctrl[5] ? pulse4 : reg_pos4;

    // 引擎启动：PS 主动 或 IK 自动
    wire eng_start = reg_ctrl[0] | ik_auto_req | eng_go;

    // 索引 1~4 读关节角度(deg256)，5~8 读脉冲
    wire signed [31:0] ik_sel =
        (ik_idx == 2'd1) ? ik_j1 :
        (ik_idx == 2'd2) ? ik_j2 :
        (ik_idx == 2'd3) ? ik_j3 : ik_j4;

    wire busy, done;
    wire [2:0]  cur_joint;
    wire [15:0] frame_count;

    zdt_tx_engine #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_eng (
        .clk(aclk), .rst_n(aresetn),
        .start(eng_start), .busy(busy), .done(done),
        .pos1(pos1_eff), .pos2(pos2_eff), .pos3(pos3_eff), .pos4(pos4_eff),
        .speed(reg_timing[15:0]), .accel(reg_timing[23:16]),
        .abs_rel(reg_ctrl[1] | reg_ctrl[5] | reg_ctrl[6]), .sync_en(reg_ctrl[2]),   // IK 自动模式一律绝对位置
        .gap_ms(reg_gap[7:0]),
        .skip_en(reg_ctrl[4] | reg_ctrl[6]),
        .tx(tx), .cur_joint(cur_joint), .frame_count(frame_count)
    );

    //------------------------------------------------------------------
    // 写通道
    //------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            reg_ctrl     <= 32'd0;
            reg_timing   <= 32'h00F00BB8;
            reg_pos1     <= 32'd0;
            reg_pos2     <= 32'd0;
            reg_pos3     <= 32'd0;
            reg_pos4     <= 32'd0;
            reg_gap      <= 32'd60;
            reg_dur      <= INTERP_TICK_MS;
            ik_x         <= 16'sd2500;
            ik_y         <= 16'sd0;
            ik_z         <= 16'sd1500;
            ik_idx       <= 2'd1;
            aw_pending   <= 1'b0;
            w_pending    <= 1'b0;
            wr_addr      <= 8'd0;
            wr_data      <= 32'd0;
            wr_strb      <= 4'd0;
            clr_pulse    <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
        end else begin
            clr_pulse <= 1'b0;

            if (s_axi_awvalid && s_axi_awready) begin
                wr_addr <= s_axi_awaddr;
                aw_pending <= 1'b1;
            end
            if (s_axi_wvalid && s_axi_wready) begin
                wr_data <= s_axi_wdata;
                wr_strb <= s_axi_wstrb;
                w_pending <= 1'b1;
            end

            if (wr_fire) begin
                aw_pending <= 1'b0;
                w_pending <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
                case (wr_addr[7:2])
                    4'd0: reg_ctrl   <= merge_wstrb(reg_ctrl, wr_data, wr_strb) & 32'h0000007E;   // bit1..6（bit6=插补模式）   // bit1..6
                    4'd1: reg_timing <= merge_wstrb(reg_timing, wr_data, wr_strb);
                    4'd2: reg_pos1   <= merge_wstrb(reg_pos1, wr_data, wr_strb);
                    4'd3: reg_pos2   <= merge_wstrb(reg_pos2, wr_data, wr_strb);
                    4'd4: reg_pos3   <= merge_wstrb(reg_pos3, wr_data, wr_strb);
                    4'd5: reg_pos4   <= merge_wstrb(reg_pos4, wr_data, wr_strb);
                    4'd7: clr_pulse  <= wr_strb[0] && wr_data[0];
                    4'd8: reg_gap    <= merge_wstrb(reg_gap, wr_data, wr_strb);
                    4'd9:  ik_x      <= merge_wstrb({16'd0, ik_x}, wr_data, wr_strb);
                    4'd10: ik_y      <= merge_wstrb({16'd0, ik_y}, wr_data, wr_strb);
                    4'd11: ik_z      <= merge_wstrb({16'd0, ik_z}, wr_data, wr_strb);
                    4'd13: if (wr_strb[0]) ik_idx <= wr_data[1:0];
                    6'd16: reg_dur   <= merge_wstrb({16'd0, reg_dur}, wr_data, wr_strb);
                    default: ;
                endcase
                if (wr_addr[5:2] == 4'd0 && wr_strb[0] && wr_data[0])
                    reg_ctrl[0] <= 1'b1;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (reg_ctrl[0] && !wr_fire)
                reg_ctrl[0] <= 1'b0;
        end
    end

    //------------------------------------------------------------------
    // 读通道
    //------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'd0;
            s_axi_rresp  <= 2'b00;
        end else begin
            if (rd_fire) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
                case (s_axi_araddr[7:2])
                    4'd0: s_axi_rdata <= {27'd0, reg_ctrl[4:1], 1'b0};
                    4'd1: s_axi_rdata <= reg_timing;
                    4'd2: s_axi_rdata <= reg_pos1;
                    4'd3: s_axi_rdata <= reg_pos2;
                    4'd4: s_axi_rdata <= reg_pos3;
                    4'd5: s_axi_rdata <= reg_pos4;
                    4'd6: s_axi_rdata <= {frame_count, 10'd0, cur_joint, 1'b0, done_sticky, busy};
                    4'd8: s_axi_rdata <= reg_gap;
                    4'd9:  s_axi_rdata <= {{16{ik_x[15]}}, ik_x};
                    4'd10: s_axi_rdata <= {{16{ik_y[15]}}, ik_y};
                    4'd11: s_axi_rdata <= {{16{ik_z[15]}}, ik_z};
                    4'd13: s_axi_rdata <= ik_sel;
                    6'd16: s_axi_rdata <= {16'd0, reg_dur};
                    4'd15: s_axi_rdata <= {29'd0, ik_err, ik_done_sticky, ik_busy};
                    default: s_axi_rdata <= 32'd0;
                endcase
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    // IK 启动脉冲与 done 粘滞
    //------------------------------------------------------------------
    // 使用已握手缓存的地址/数据；一次有效写提交只产生一拍启动。
    // busy 时不接受新求解，已完成结果保持到下次有效启动。
    assign ik_start = wr_fire && (wr_addr[5:2] == 4'd12) &&
                      wr_strb[0] && wr_data[0] && !ik_busy;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn)
            ik_done_sticky <= 1'b0;
        else if (ik_start)
            ik_done_sticky <= 1'b0;
        else if (ik_done)
            ik_done_sticky <= 1'b1;
    end

    //------------------------------------------------------------------
    // done 粘滞
    //------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn)       done_sticky <= 1'b0;
        else if (clr_pulse) done_sticky <= 1'b0;
        else if (done)      done_sticky <= 1'b1;
    end

endmodule

`default_nettype wire
