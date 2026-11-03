//=============================================================================
// zdt_tx_engine.v —— 四轴 ZDT 指令引擎（使能 + 位置，帧间隔可配置）
//
// 重要：ZDT 驱动器串口处理慢，背靠背连发会被丢弃。
//       ik_zdt 源码注释："the drive loses the first frames after idle"
//       其做法：每帧之间 usleep(60000)，使能阶段结束后 usleep(120000)
//
// 本引擎把帧间隔做成输入 gap_ms（毫秒），由 PS 经 AXI 配置，现场可调：
//   gap_ms = 60 → 与 ik_zdt 行为一致（可靠）
//   gap_ms 越小越快，但可能丢帧
//
// 序号 seq 0..15：
//   0..11  使能帧（轴 = seq/3 + 1，每轴 3 次）
//   12..15 位置帧（轴 = seq - 11）
//   发完第 11 帧后额外等待 2×gap_ms
//=============================================================================
`default_nettype none

module zdt_tx_engine #(
    parameter integer CLK_HZ   = 50_000_000,
    parameter integer BAUD     = 115200,
    parameter integer EN_TRIES = 3
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,
    output reg         busy,
    output reg         done,

    input  wire [31:0] pos1,
    input  wire [31:0] pos2,
    input  wire [31:0] pos3,
    input  wire [31:0] pos4,
    input  wire [15:0] speed,
    input  wire [7:0]  accel,
    input  wire        abs_rel,
    input  wire        sync_en,
    input  wire [7:0]  gap_ms,          // 帧间隔（毫秒）
    input  wire        skip_en,         // 1=跳过使能阶段（仅发位置帧）

    output wire        tx,
    output reg  [2:0]  cur_joint,
    output reg  [15:0] frame_count
);

    localparam integer MS_CYC  = CLK_HZ / 1000;
    localparam integer EN_FRMS = 4 * EN_TRIES;   // 12
    localparam integer SEQ_MAX      = EN_FRMS + 4;      // 16 (无同步)
    localparam integer SEQ_MAX_SYNC = EN_FRMS + 5;      // 17 (带同步触发)

    //------------------------------------------------------------------
    // 帧发送器
    //------------------------------------------------------------------
    reg         f_start;
    wire        f_busy, f_done;
    reg  [1:0]  f_fsel;
    reg  [7:0]  f_addr;
    reg         f_dir;
    reg  [31:0] f_pos;
    reg  [15:0] f_speed;
    reg  [7:0]  f_accel;
    reg         f_absrel;
    reg         f_sync;

    zdt_frame_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(f_start), .busy(f_busy), .done(f_done),
        .fsel(f_fsel),
        .addr(f_addr), .dir(f_dir), .speed(f_speed), .accel(f_accel),
        .pos(f_pos), .abs_rel(f_absrel), .sync(f_sync),
        .tx(tx)
    );

    //------------------------------------------------------------------
    // 序号 → 轴号 / 阶段
    //------------------------------------------------------------------
    reg [4:0] seq;
    wire      is_en_phase  = (seq < EN_FRMS);
    wire      is_sync_frm  = (seq == SEQ_MAX);              // 第 17 帧：同步触发
    wire [2:0] axis_en     = (seq / EN_TRIES) + 3'd1;
    wire [2:0] axis_pos    = seq - EN_FRMS + 5'd1;
    wire [2:0] axis_now    = is_en_phase ? axis_en : axis_pos;
    wire [4:0] seq_last    = sync_en ? SEQ_MAX_SYNC - 1 : SEQ_MAX - 1;

    reg [31:0] cur_pos;
    always @* begin
        case (axis_now)
            3'd1:    cur_pos = pos1;
            3'd2:    cur_pos = pos2;
            3'd3:    cur_pos = pos3;
            3'd4:    cur_pos = pos4;
            default: cur_pos = 32'd0;
        endcase
    end

    wire [31:0] cur_mag = cur_pos[31] ? (~cur_pos + 32'd1) : cur_pos;
    wire        cur_dir = cur_pos[31];

    //------------------------------------------------------------------
    // 延时
    //------------------------------------------------------------------
    reg [31:0] dly_cnt;
    reg [31:0] dly_tgt;
    reg        dly_run;
    wire       dly_done = dly_run && (dly_cnt >= dly_tgt);

    //------------------------------------------------------------------
    // 状态机
    //------------------------------------------------------------------
    localparam S_IDLE  = 3'd0,
               S_LOAD  = 3'd1,
               S_SEND  = 3'd2,
               S_WAIT  = 3'd3,
               S_DELAY = 3'd4,
               S_FIN   = 3'd5;

    reg [2:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            seq         <= 5'd0;
            busy        <= 1'b0;
            done        <= 1'b0;
            cur_joint   <= 3'd1;
            frame_count <= 16'd0;
            f_start     <= 1'b0;
            f_fsel      <= 2'd0;
            f_addr      <= 8'd1;
            f_dir       <= 1'b0;
            f_pos       <= 32'd0;
            f_speed     <= 16'd3000;
            f_accel     <= 8'd240;
            f_absrel    <= 1'b1;
            f_sync      <= 1'b0;
            dly_cnt     <= 32'd0;
            dly_tgt     <= 32'd0;
            dly_run     <= 1'b0;
        end else begin
            done    <= 1'b0;
            f_start <= 1'b0;

            if (dly_run) dly_cnt <= dly_cnt + 32'd1;

            case (st)
            S_IDLE: begin
                seq     <= 5'd0;
                dly_run <= 1'b0;
                if (start) begin
                    busy <= 1'b1;
                    seq  <= skip_en ? EN_FRMS[4:0] : 5'd0;
                    st   <= S_LOAD;
                end
            end

            S_LOAD: begin
                cur_joint <= axis_now;
                f_addr    <= {5'd0, axis_now};
                f_dir     <= cur_dir;
                f_pos     <= cur_mag;
                f_speed   <= speed;
                f_accel   <= accel;
                f_absrel  <= abs_rel;
                f_sync    <= (~is_en_phase) && sync_en;
                f_fsel    <= is_sync_frm ? 2'd2 :
                             (is_en_phase ? 2'd1 : 2'd0);
                st        <= S_SEND;
            end

            S_SEND: begin
                if (!f_busy) begin
                    f_start <= 1'b1;
                    st      <= S_WAIT;
                end
            end

            S_WAIT: begin
                if (f_done) begin
                    frame_count <= frame_count + 16'd1;

                    // 计算本帧之后的延时
                    dly_cnt <= 32'd0;
                    dly_run <= 1'b1;
                    if (seq == EN_FRMS - 1)
                        dly_tgt <= (2 * gap_ms) * MS_CYC;   // 使能后长延时
                    else if (seq == seq_last)
                        dly_tgt <= 32'd0;                   // 最后一帧不等
                    else
                        dly_tgt <= gap_ms * MS_CYC;

                    if (seq == seq_last)
                        st <= S_FIN;
                    else
                        st <= S_DELAY;
                end
            end

            S_DELAY: begin
                if (!dly_run || dly_done) begin
                    dly_run <= 1'b0;
                    seq     <= seq + 5'd1;
                    st      <= S_LOAD;
                end
            end

            S_FIN: begin
                if (!dly_run || dly_done) begin
                    dly_run <= 1'b0;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    st      <= S_IDLE;
                end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
