//=============================================================================
// zdt_frame_tx.v —— ZDT 帧发送器（支持两种帧）
//
//   帧类型  is_enable=0  位置帧（13 字节）
//     [0] addr  [1] 0xFD  [2] dir  [3] speedH  [4] speedL  [5] accel
//     [6..9] pos32  [10] abs_rel  [11] sync  [12] 0x6B
//     例：01 FD 01 0B B8 F0 00 00 00 00 01 00 6B
//
//   帧类型  is_enable=1  使能帧（6 字节）
//     [0] addr  [1] 0xF3  [2] 0xAB  [3] 0x01  [4] 0x00  [5] 0x6B
//     例：01 F3 AB 01 00 6B
//
// 说明：ZDT 驱动器上电默认未使能（轴是松的），必须先发使能帧，
//       否则位置帧会被忽略。ik_zdt 每次动作前都发 3 次使能帧。
//=============================================================================
`default_nettype none

module zdt_frame_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,
    output reg         busy,
    output reg         done,

    input  wire [1:0]  fsel,          // 0=位置帧(13B) 1=使能帧(6B) 2=同步触发(4B)
    input  wire [7:0]  addr,
    input  wire        dir,
    input  wire [15:0] speed,
    input  wire [7:0]  accel,
    input  wire [31:0] pos,
    input  wire        abs_rel,
    input  wire        sync,

    output reg         tx
);

    localparam integer BAUD_DIV = (CLK_HZ + BAUD / 2) / BAUD;

    // 帧长度：使能 6 字节（末字节索引 5），位置 13 字节（末字节索引 12）
    wire [3:0] last_idx = (fsel == 2'd2) ? 4'd3 :
                      (fsel == 2'd1) ? 4'd5 : 4'd12;

    //------------------------------------------------------------------
    // 第 i 字节内容
    //------------------------------------------------------------------
    function [7:0] fbyte;
        input [3:0] i;
        input [1:0] en;
        begin
            if (en == 2'd2) begin
                case (i)
                    4'd0:    fbyte = 8'h00;   /* 广播地址 */
                    4'd1:    fbyte = 8'hFF;   /* SYNC_MOVE */
                    4'd2:    fbyte = 8'h66;   /* 协议码 */
                    4'd3:    fbyte = 8'h6B;   /* 校验 */
                    default: fbyte = 8'h00;
                endcase
            end else if (en == 2'd1) begin
                case (i)
                    4'd0:    fbyte = addr;
                    4'd1:    fbyte = 8'hF3;
                    4'd2:    fbyte = 8'hAB;
                    4'd3:    fbyte = 8'h01;
                    4'd4:    fbyte = 8'h00;
                    4'd5:    fbyte = 8'h6B;
                    default: fbyte = 8'h00;
                endcase
            end else begin
                case (i)
                    4'd0:    fbyte = addr;
                    4'd1:    fbyte = 8'hFD;
                    4'd2:    fbyte = {7'd0, dir};
                    4'd3:    fbyte = speed[15:8];
                    4'd4:    fbyte = speed[7:0];
                    4'd5:    fbyte = accel;
                    4'd6:    fbyte = pos[31:24];
                    4'd7:    fbyte = pos[23:16];
                    4'd8:    fbyte = pos[15:8];
                    4'd9:    fbyte = pos[7:0];
                    4'd10:   fbyte = {7'd0, abs_rel};
                    4'd11:   fbyte = {7'd0, sync};
                    4'd12:   fbyte = 8'h6B;
                    default: fbyte = 8'h00;
                endcase
            end
        end
    endfunction

    //------------------------------------------------------------------
    // 状态机
    //------------------------------------------------------------------
    localparam S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;

    reg [1:0]  st;
    reg [3:0]  byte_idx;
    reg [3:0]  bit_idx;
    reg [15:0] baud_cnt;
    reg [7:0]  shift_reg;
    reg  [1:0] en_lat;          // 锁存帧类型

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            byte_idx  <= 4'd0;
            bit_idx   <= 4'd0;
            baud_cnt  <= 16'd0;
            shift_reg <= 8'd0;
            tx        <= 1'b1;
            busy      <= 1'b0;
            done      <= 1'b0;
            en_lat    <= 1'b0;
        end else begin
            done <= 1'b0;

            case (st)
            S_IDLE: begin
                tx <= 1'b1;
                if (start) begin
                    busy      <= 1'b1;
                    en_lat    <= fsel;
                    byte_idx  <= 4'd0;
                    bit_idx   <= 4'd0;
                    baud_cnt  <= 16'd0;
                    shift_reg <= fbyte(4'd0, fsel);
                    st        <= S_START;
                end
            end

            S_START: begin
                tx <= 1'b0;
                if (baud_cnt == BAUD_DIV - 1) begin
                    baud_cnt <= 16'd0;
                    bit_idx  <= 4'd0;
                    st       <= S_DATA;
                end else begin
                    baud_cnt <= baud_cnt + 16'd1;
                end
            end

            S_DATA: begin
                tx <= shift_reg[0];
                if (baud_cnt == BAUD_DIV - 1) begin
                    baud_cnt  <= 16'd0;
                    shift_reg <= {1'b0, shift_reg[7:1]};
                    if (bit_idx == 4'd7) begin
                        st <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 4'd1;
                    end
                end else begin
                    baud_cnt <= baud_cnt + 16'd1;
                end
            end

            S_STOP: begin
                tx <= 1'b1;
                if (baud_cnt == BAUD_DIV - 1) begin
                    baud_cnt <= 16'd0;
                    if (byte_idx == last_idx) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        st   <= S_IDLE;
                    end else begin
                        byte_idx  <= byte_idx + 4'd1;
                        shift_reg <= fbyte(byte_idx + 4'd1, en_lat);
                        st        <= S_START;
                    end
                end else begin
                    baud_cnt <= baud_cnt + 16'd1;
                end
            end
            endcase
        end
    end

endmodule

`default_nettype wire
