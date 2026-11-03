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
//
// 握手：ready 常高（除非响应未取走），写数据在 awvalid&wvalid 同拍采样。
// 资源：231 LUT / 322 FF / 0 BRAM / 0 DSP，WNS +4.94ns @100MHz
//=============================================================================
`default_nettype none

module zdt_axil #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk,ASSOCIATED_BUSIF S_AXI,ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn,POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    // ---------------- AXI4-Lite 从机 ----------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI,PROTOCOL AXI4LITE,DATA_WIDTH 32,ADDR_WIDTH 6,NUM_READ_OUTSTANDING 1,NUM_WRITE_OUTSTANDING 1,READ_WRITE_MODE READ_WRITE,HAS_BURST 0,HAS_LOCK 0,HAS_CACHE 0,HAS_REGION 0,HAS_QOS 0,HAS_PROT 0" *)
    input  wire [5:0]  s_axi_awaddr,
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
    input  wire [5:0]  s_axi_araddr,
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

    assign s_axi_awready = ~s_axi_bvalid;
    assign s_axi_wready  = ~s_axi_bvalid;
    assign s_axi_arready = ~s_axi_rvalid;

    wire wr_fire = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
    wire rd_fire = s_axi_arvalid && !s_axi_rvalid;

    //------------------------------------------------------------------
    // 寄存器
    //------------------------------------------------------------------
    reg [31:0] reg_ctrl;
    reg [31:0] reg_timing;
    reg [31:0] reg_pos1, reg_pos2, reg_pos3, reg_pos4;
    reg [31:0] reg_gap;          /* 0x20: 帧间隔 ms */
    reg        done_sticky;
    reg        clr_pulse;

    wire busy, done;
    wire [2:0]  cur_joint;
    wire [15:0] frame_count;

    zdt_tx_engine #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_eng (
        .clk(aclk), .rst_n(aresetn),
        .start(reg_ctrl[0]), .busy(busy), .done(done),
        .pos1(reg_pos1), .pos2(reg_pos2), .pos3(reg_pos3), .pos4(reg_pos4),
        .speed(reg_timing[15:0]), .accel(reg_timing[23:16]),
        .abs_rel(reg_ctrl[1]), .sync_en(reg_ctrl[2]),
        .gap_ms(reg_gap[7:0]),
        .skip_en(reg_ctrl[4]),
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
            reg_gap      <= 32'd60;    /* 默认 60ms，与 ik_zdt 一致 */
            clr_pulse    <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
        end else begin
            clr_pulse <= 1'b0;

            if (wr_fire) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
                case (s_axi_awaddr[5:2])
                    4'd0: reg_ctrl   <= {27'd0, s_axi_wdata[4:1], 1'b0};
                    4'd1: reg_timing <= s_axi_wdata;
                    4'd2: reg_pos1   <= s_axi_wdata;
                    4'd3: reg_pos2   <= s_axi_wdata;
                    4'd4: reg_pos3   <= s_axi_wdata;
                    4'd5: reg_pos4   <= s_axi_wdata;
                    4'd7: clr_pulse  <= s_axi_wdata[0];
                    4'd8: reg_gap    <= s_axi_wdata;
                    default: ;
                endcase
                if (s_axi_awaddr[5:2] == 4'd0 && s_axi_wdata[0])
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
                case (s_axi_araddr[5:2])
                    4'd0: s_axi_rdata <= {27'd0, reg_ctrl[4:1], 1'b0};
                    4'd1: s_axi_rdata <= reg_timing;
                    4'd2: s_axi_rdata <= reg_pos1;
                    4'd3: s_axi_rdata <= reg_pos2;
                    4'd4: s_axi_rdata <= reg_pos3;
                    4'd5: s_axi_rdata <= reg_pos4;
                    4'd6: s_axi_rdata <= {frame_count, 10'd0, cur_joint, 1'b0, done_sticky, busy};
                    4'd8: s_axi_rdata <= reg_gap;
                    default: s_axi_rdata <= 32'd0;
                endcase
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
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
