//=============================================================================
// video_osd_text16.v —— 硬件字符 OSD（16x16 字库，PS 写文字缓冲）
//
// 面板：32 列 x 8 行，每字符 16x16 像素 = 512 x 128
// PS 通过 AXI4-Lite 写 ASCII 到 256 字节字符 RAM（0x40001000）
//=============================================================================
`default_nettype none

module video_osd_text21 #(
    parameter integer X0    = 16,
    parameter integer Y0    = 64,
    parameter integer COLS  = 24,
    parameter integer ROWS  = 6,
    parameter integer SCALE = 2,
    parameter integer PITCH = 40,         // 行距（像素）
    parameter integer ALPHA_BG = 70       // 背景不透明度 0~255（70=很透）
)(
    input  wire        aclk,
    input  wire        aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI,PROTOCOL AXI4LITE,DATA_WIDTH 32,ADDR_WIDTH 8,NUM_READ_OUTSTANDING 1,NUM_WRITE_OUTSTANDING 1,READ_WRITE_MODE WRITE_ONLY,HAS_BURST 0,HAS_LOCK 0,HAS_CACHE 0,HAS_REGION 0,HAS_QOS 0,HAS_PROT 0" *)
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
    output wire [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output wire        s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire        s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [7:0]  s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire        s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output wire [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output wire [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output wire        s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire        s_axi_rready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input  wire [23:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire        s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input  wire        s_axis_tuser,
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXIS,TDATA_NUM_BYTES 3,TUSER_WIDTH 1,HAS_TLAST 1,HAS_TREADY 1" *)

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output wire        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire        m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    output wire [23:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output wire        m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TUSER" *)
    output wire        m_axis_tuser,
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M_AXIS,TDATA_NUM_BYTES 3,TUSER_WIDTH 1,HAS_TLAST 1,HAS_TREADY 1" *)

    input  wire        enable
);

    assign s_axis_tready = m_axis_tready;
    assign m_axis_tvalid = s_axis_tvalid;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tuser  = s_axis_tuser;
    wire beat = s_axis_tvalid && m_axis_tready;

    // ---------------- AXI 写 ----------------
    (* ram_style = "distributed" *) reg [7:0] text_ram [0:255];

    // ---- 标准 AXI4-Lite 写握手（AW/W 独立锁存）----
    reg        aw_lat, w_lat, bvalid_r;
    reg [7:0]  awaddr_r;
    reg [31:0] wdata_r;
    reg [3:0]  wstrb_r;

    assign s_axi_awready = ~aw_lat & ~bvalid_r;
    assign s_axi_wready  = ~w_lat  & ~bvalid_r;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_bvalid  = bvalid_r;
    assign s_axi_arready = 1'b0;
    assign s_axi_rdata   = 32'd0;
    assign s_axi_rresp   = 2'b00;
    assign s_axi_rvalid  = 1'b0;

    wire [7:0] wbase = {awaddr_r[7:2], 2'b00};

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            aw_lat <= 1'b0; w_lat <= 1'b0; bvalid_r <= 1'b0;
            awaddr_r <= 8'd0; wdata_r <= 32'd0; wstrb_r <= 4'd0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin
                aw_lat   <= 1'b1;
                awaddr_r <= s_axi_awaddr;
            end
            if (s_axi_wvalid && s_axi_wready) begin
                w_lat   <= 1'b1;
                wdata_r <= s_axi_wdata;
                wstrb_r <= s_axi_wstrb;
            end
            if (aw_lat && w_lat && !bvalid_r) begin
                if (wstrb_r[0]) text_ram[wbase + 8'd0] <= wdata_r[7:0];
                if (wstrb_r[1]) text_ram[wbase + 8'd1] <= wdata_r[15:8];
                if (wstrb_r[2]) text_ram[wbase + 8'd2] <= wdata_r[23:16];
                if (wstrb_r[3]) text_ram[wbase + 8'd3] <= wdata_r[31:24];
                aw_lat   <= 1'b0;
                w_lat    <= 1'b0;
                bvalid_r <= 1'b1;
            end
            if (bvalid_r && s_axi_bready) bvalid_r <= 1'b0;
        end
    end
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) text_ram[i] = 8'd32;
    end

    // ---------------- 16x16 字库 ----------------
    (* rom_style = "distributed" *) reg [15:0] font_rom [0:1023];
    initial begin
        font_rom[  0] = 16'h0000; font_rom[  1] = 16'h0000; font_rom[  2] = 16'h0000; font_rom[  3] = 16'h0000;
        font_rom[  4] = 16'h0000; font_rom[  5] = 16'h0000; font_rom[  6] = 16'h0000; font_rom[  7] = 16'h0000;
        font_rom[  8] = 16'h0000; font_rom[  9] = 16'h0000; font_rom[ 10] = 16'h0000; font_rom[ 11] = 16'h0000;
        font_rom[ 12] = 16'h0000; font_rom[ 13] = 16'h0000; font_rom[ 14] = 16'h0000; font_rom[ 15] = 16'h0000;
        font_rom[ 16] = 16'h0780; font_rom[ 17] = 16'h07C0; font_rom[ 18] = 16'h0FC0; font_rom[ 19] = 16'h0CC0;
        font_rom[ 20] = 16'h0CE0; font_rom[ 21] = 16'h1CE0; font_rom[ 22] = 16'h1CE0; font_rom[ 23] = 16'h1870;
        font_rom[ 24] = 16'h3870; font_rom[ 25] = 16'h3C70; font_rom[ 26] = 16'h3FF8; font_rom[ 27] = 16'h3FF8;
        font_rom[ 28] = 16'h7878; font_rom[ 29] = 16'hF01C; font_rom[ 30] = 16'hE01C; font_rom[ 31] = 16'hE01C;
        font_rom[ 32] = 16'h3FE0; font_rom[ 33] = 16'h3FF0; font_rom[ 34] = 16'h3870; font_rom[ 35] = 16'h3070;
        font_rom[ 36] = 16'h3070; font_rom[ 37] = 16'h38F0; font_rom[ 38] = 16'h3FE0; font_rom[ 39] = 16'h3FE0;
        font_rom[ 40] = 16'h38F0; font_rom[ 41] = 16'h3078; font_rom[ 42] = 16'h3038; font_rom[ 43] = 16'h3038;
        font_rom[ 44] = 16'h3078; font_rom[ 45] = 16'h3870; font_rom[ 46] = 16'h3FE0; font_rom[ 47] = 16'h3FE0;
        font_rom[ 48] = 16'h03FC; font_rom[ 49] = 16'h07FC; font_rom[ 50] = 16'h0F00; font_rom[ 51] = 16'h1E00;
        font_rom[ 52] = 16'h1C00; font_rom[ 53] = 16'h3800; font_rom[ 54] = 16'h3800; font_rom[ 55] = 16'h3800;
        font_rom[ 56] = 16'h3800; font_rom[ 57] = 16'h3800; font_rom[ 58] = 16'h3800; font_rom[ 59] = 16'h1C00;
        font_rom[ 60] = 16'h1E00; font_rom[ 61] = 16'h1F00; font_rom[ 62] = 16'h07FC; font_rom[ 63] = 16'h07FC;
        font_rom[ 64] = 16'h3FE0; font_rom[ 65] = 16'h3FF0; font_rom[ 66] = 16'h3CF8; font_rom[ 67] = 16'h383C;
        font_rom[ 68] = 16'h381C; font_rom[ 69] = 16'h381E; font_rom[ 70] = 16'h381E; font_rom[ 71] = 16'h381E;
        font_rom[ 72] = 16'h381E; font_rom[ 73] = 16'h381E; font_rom[ 74] = 16'h381E; font_rom[ 75] = 16'h381C;
        font_rom[ 76] = 16'h3838; font_rom[ 77] = 16'h3CF8; font_rom[ 78] = 16'h3FF0; font_rom[ 79] = 16'h3FE0;
        font_rom[ 80] = 16'h1FF8; font_rom[ 81] = 16'h1FF8; font_rom[ 82] = 16'h1E00; font_rom[ 83] = 16'h1C00;
        font_rom[ 84] = 16'h1C00; font_rom[ 85] = 16'h1E00; font_rom[ 86] = 16'h1FF8; font_rom[ 87] = 16'h1FF8;
        font_rom[ 88] = 16'h1E00; font_rom[ 89] = 16'h1C00; font_rom[ 90] = 16'h1C00; font_rom[ 91] = 16'h1C00;
        font_rom[ 92] = 16'h1C00; font_rom[ 93] = 16'h1E00; font_rom[ 94] = 16'h1FF8; font_rom[ 95] = 16'h1FF8;
        font_rom[ 96] = 16'h1FF8; font_rom[ 97] = 16'h1FF8; font_rom[ 98] = 16'h1E00; font_rom[ 99] = 16'h1C00;
        font_rom[100] = 16'h1C00; font_rom[101] = 16'h1E00; font_rom[102] = 16'h1FF8; font_rom[103] = 16'h1FF8;
        font_rom[104] = 16'h1E00; font_rom[105] = 16'h1C00; font_rom[106] = 16'h1C00; font_rom[107] = 16'h1C00;
        font_rom[108] = 16'h1C00; font_rom[109] = 16'h1C00; font_rom[110] = 16'h1C00; font_rom[111] = 16'h1C00;
        font_rom[112] = 16'h07FC; font_rom[113] = 16'h07FC; font_rom[114] = 16'h1F00; font_rom[115] = 16'h1E00;
        font_rom[116] = 16'h3C00; font_rom[117] = 16'h3800; font_rom[118] = 16'h78FC; font_rom[119] = 16'h78FE;
        font_rom[120] = 16'h783E; font_rom[121] = 16'h781C; font_rom[122] = 16'h381C; font_rom[123] = 16'h3C1E;
        font_rom[124] = 16'h1C1E; font_rom[125] = 16'h1F3C; font_rom[126] = 16'h0FFC; font_rom[127] = 16'h07F8;
        font_rom[128] = 16'h381E; font_rom[129] = 16'h381E; font_rom[130] = 16'h381E; font_rom[131] = 16'h381E;
        font_rom[132] = 16'h381E; font_rom[133] = 16'h3C3E; font_rom[134] = 16'h3FFE; font_rom[135] = 16'h3FFE;
        font_rom[136] = 16'h3C3E; font_rom[137] = 16'h381E; font_rom[138] = 16'h381E; font_rom[139] = 16'h381E;
        font_rom[140] = 16'h381E; font_rom[141] = 16'h381E; font_rom[142] = 16'h381E; font_rom[143] = 16'h381E;
        font_rom[144] = 16'h3FF0; font_rom[145] = 16'h3FF0; font_rom[146] = 16'h0780; font_rom[147] = 16'h0300;
        font_rom[148] = 16'h0380; font_rom[149] = 16'h0380; font_rom[150] = 16'h0380; font_rom[151] = 16'h0380;
        font_rom[152] = 16'h0380; font_rom[153] = 16'h0380; font_rom[154] = 16'h0380; font_rom[155] = 16'h0380;
        font_rom[156] = 16'h0300; font_rom[157] = 16'h0780; font_rom[158] = 16'h3FF0; font_rom[159] = 16'h3FF0;
        font_rom[160] = 16'h1FF8; font_rom[161] = 16'h1FF8; font_rom[162] = 16'h0078; font_rom[163] = 16'h0078;
        font_rom[164] = 16'h0078; font_rom[165] = 16'h0078; font_rom[166] = 16'h0078; font_rom[167] = 16'h0078;
        font_rom[168] = 16'h0078; font_rom[169] = 16'h0078; font_rom[170] = 16'h0078; font_rom[171] = 16'h0078;
        font_rom[172] = 16'h1870; font_rom[173] = 16'h1CF0; font_rom[174] = 16'h1FE0; font_rom[175] = 16'h0FC0;
        font_rom[176] = 16'h1C3C; font_rom[177] = 16'h1C38; font_rom[178] = 16'h1C70; font_rom[179] = 16'h1CF0;
        font_rom[180] = 16'h19E0; font_rom[181] = 16'h19C0; font_rom[182] = 16'h1F80; font_rom[183] = 16'h1F80;
        font_rom[184] = 16'h1F80; font_rom[185] = 16'h1F80; font_rom[186] = 16'h1DC0; font_rom[187] = 16'h19E0;
        font_rom[188] = 16'h18F0; font_rom[189] = 16'h1C78; font_rom[190] = 16'h1C38; font_rom[191] = 16'h1C1C;
        font_rom[192] = 16'h3C00; font_rom[193] = 16'h3C00; font_rom[194] = 16'h3C00; font_rom[195] = 16'h3C00;
        font_rom[196] = 16'h3C00; font_rom[197] = 16'h3C00; font_rom[198] = 16'h3C00; font_rom[199] = 16'h3C00;
        font_rom[200] = 16'h3C00; font_rom[201] = 16'h3C00; font_rom[202] = 16'h3C00; font_rom[203] = 16'h3C00;
        font_rom[204] = 16'h3C00; font_rom[205] = 16'h3E00; font_rom[206] = 16'h3FF0; font_rom[207] = 16'h3FF8;
        font_rom[208] = 16'h3C1C; font_rom[209] = 16'h3C3E; font_rom[210] = 16'h3C3E; font_rom[211] = 16'h3E7E;
        font_rom[212] = 16'h7E7E; font_rom[213] = 16'h7E7E; font_rom[214] = 16'h7E7E; font_rom[215] = 16'h7BCE;
        font_rom[216] = 16'h71CE; font_rom[217] = 16'h718E; font_rom[218] = 16'h700E; font_rom[219] = 16'h700E;
        font_rom[220] = 16'h700E; font_rom[221] = 16'h700E; font_rom[222] = 16'h700E; font_rom[223] = 16'h700E;
        font_rom[224] = 16'h3C1C; font_rom[225] = 16'h3C1C; font_rom[226] = 16'h3E1C; font_rom[227] = 16'h3E1C;
        font_rom[228] = 16'h3E1C; font_rom[229] = 16'h3F1C; font_rom[230] = 16'h399C; font_rom[231] = 16'h399C;
        font_rom[232] = 16'h399E; font_rom[233] = 16'h39FE; font_rom[234] = 16'h38FE; font_rom[235] = 16'h387E;
        font_rom[236] = 16'h387E; font_rom[237] = 16'h387C; font_rom[238] = 16'h383C; font_rom[239] = 16'h383C;
        font_rom[240] = 16'h07F0; font_rom[241] = 16'h0FF0; font_rom[242] = 16'h1E78; font_rom[243] = 16'h3C3C;
        font_rom[244] = 16'h381E; font_rom[245] = 16'h781E; font_rom[246] = 16'h780E; font_rom[247] = 16'h780E;
        font_rom[248] = 16'h780E; font_rom[249] = 16'h780E; font_rom[250] = 16'h781E; font_rom[251] = 16'h381E;
        font_rom[252] = 16'h3C3C; font_rom[253] = 16'h1E78; font_rom[254] = 16'h0FF0; font_rom[255] = 16'h07E0;
        font_rom[256] = 16'h3FE0; font_rom[257] = 16'h3FF0; font_rom[258] = 16'h3870; font_rom[259] = 16'h3078;
        font_rom[260] = 16'h3038; font_rom[261] = 16'h3038; font_rom[262] = 16'h3078; font_rom[263] = 16'h38F0;
        font_rom[264] = 16'h3FE0; font_rom[265] = 16'h3FC0; font_rom[266] = 16'h3800; font_rom[267] = 16'h3000;
        font_rom[268] = 16'h3000; font_rom[269] = 16'h3800; font_rom[270] = 16'h3800; font_rom[271] = 16'h3800;
        font_rom[272] = 16'h0FC0; font_rom[273] = 16'h1FE0; font_rom[274] = 16'h38E0; font_rom[275] = 16'h3870;
        font_rom[276] = 16'h7030; font_rom[277] = 16'h7030; font_rom[278] = 16'h7030; font_rom[279] = 16'h7030;
        font_rom[280] = 16'h7070; font_rom[281] = 16'h3870; font_rom[282] = 16'h38E0; font_rom[283] = 16'h1FE0;
        font_rom[284] = 16'h0F80; font_rom[285] = 16'h0300; font_rom[286] = 16'h03F0; font_rom[287] = 16'h01F0;
        font_rom[288] = 16'h3FC0; font_rom[289] = 16'h3FE0; font_rom[290] = 16'h38F0; font_rom[291] = 16'h3070;
        font_rom[292] = 16'h3870; font_rom[293] = 16'h3870; font_rom[294] = 16'h3070; font_rom[295] = 16'h39F0;
        font_rom[296] = 16'h3FC0; font_rom[297] = 16'h3FC0; font_rom[298] = 16'h39E0; font_rom[299] = 16'h30F0;
        font_rom[300] = 16'h38F0; font_rom[301] = 16'h3870; font_rom[302] = 16'h3838; font_rom[303] = 16'h383C;
        font_rom[304] = 16'h07F8; font_rom[305] = 16'h0FF8; font_rom[306] = 16'h1E00; font_rom[307] = 16'h1C00;
        font_rom[308] = 16'h1C00; font_rom[309] = 16'h1C00; font_rom[310] = 16'h1F80; font_rom[311] = 16'h0FE0;
        font_rom[312] = 16'h03F0; font_rom[313] = 16'h00F8; font_rom[314] = 16'h003C; font_rom[315] = 16'h001C;
        font_rom[316] = 16'h003C; font_rom[317] = 16'h0038; font_rom[318] = 16'h1FF0; font_rom[319] = 16'h3FF0;
        font_rom[320] = 16'h7FFE; font_rom[321] = 16'h3FFE; font_rom[322] = 16'h03C0; font_rom[323] = 16'h0180;
        font_rom[324] = 16'h01C0; font_rom[325] = 16'h01C0; font_rom[326] = 16'h01C0; font_rom[327] = 16'h01C0;
        font_rom[328] = 16'h01C0; font_rom[329] = 16'h01C0; font_rom[330] = 16'h01C0; font_rom[331] = 16'h01C0;
        font_rom[332] = 16'h01C0; font_rom[333] = 16'h01C0; font_rom[334] = 16'h01C0; font_rom[335] = 16'h01C0;
        font_rom[336] = 16'h381E; font_rom[337] = 16'h381E; font_rom[338] = 16'h381E; font_rom[339] = 16'h381E;
        font_rom[340] = 16'h381E; font_rom[341] = 16'h381E; font_rom[342] = 16'h381E; font_rom[343] = 16'h381E;
        font_rom[344] = 16'h381E; font_rom[345] = 16'h381E; font_rom[346] = 16'h381E; font_rom[347] = 16'h381C;
        font_rom[348] = 16'h3C3C; font_rom[349] = 16'h1E38; font_rom[350] = 16'h0FF0; font_rom[351] = 16'h0FF0;
        font_rom[352] = 16'hE01C; font_rom[353] = 16'hF01C; font_rom[354] = 16'hF01C; font_rom[355] = 16'h7038;
        font_rom[356] = 16'h7038; font_rom[357] = 16'h3838; font_rom[358] = 16'h3870; font_rom[359] = 16'h3C70;
        font_rom[360] = 16'h1C70; font_rom[361] = 16'h1CE0; font_rom[362] = 16'h1CE0; font_rom[363] = 16'h0EC0;
        font_rom[364] = 16'h0FC0; font_rom[365] = 16'h0FC0; font_rom[366] = 16'h07C0; font_rom[367] = 16'h0780;
        font_rom[368] = 16'h7006; font_rom[369] = 16'h700E; font_rom[370] = 16'h700E; font_rom[371] = 16'h700E;
        font_rom[372] = 16'h780E; font_rom[373] = 16'h780E; font_rom[374] = 16'h798E; font_rom[375] = 16'h79CE;
        font_rom[376] = 16'h79CE; font_rom[377] = 16'h3FFE; font_rom[378] = 16'h3E7E; font_rom[379] = 16'h3E7E;
        font_rom[380] = 16'h1E7E; font_rom[381] = 16'h1E7C; font_rom[382] = 16'h3C3C; font_rom[383] = 16'h3C3C;
        font_rom[384] = 16'h381E; font_rom[385] = 16'h3C1C; font_rom[386] = 16'h1C38; font_rom[387] = 16'h0E78;
        font_rom[388] = 16'h0FF0; font_rom[389] = 16'h07E0; font_rom[390] = 16'h03E0; font_rom[391] = 16'h03C0;
        font_rom[392] = 16'h03E0; font_rom[393] = 16'h07E0; font_rom[394] = 16'h07F0; font_rom[395] = 16'h0E70;
        font_rom[396] = 16'h1E78; font_rom[397] = 16'h1C3C; font_rom[398] = 16'h781E; font_rom[399] = 16'h781E;
        font_rom[400] = 16'hF01C; font_rom[401] = 16'hF03C; font_rom[402] = 16'h7038; font_rom[403] = 16'h3878;
        font_rom[404] = 16'h3C70; font_rom[405] = 16'h1CF0; font_rom[406] = 16'h0FE0; font_rom[407] = 16'h0FC0;
        font_rom[408] = 16'h07C0; font_rom[409] = 16'h0780; font_rom[410] = 16'h0380; font_rom[411] = 16'h0380;
        font_rom[412] = 16'h0380; font_rom[413] = 16'h0380; font_rom[414] = 16'h0380; font_rom[415] = 16'h0380;
        font_rom[416] = 16'h3FFC; font_rom[417] = 16'h1FF8; font_rom[418] = 16'h0078; font_rom[419] = 16'h0030;
        font_rom[420] = 16'h0070; font_rom[421] = 16'h00E0; font_rom[422] = 16'h01E0; font_rom[423] = 16'h01C0;
        font_rom[424] = 16'h0180; font_rom[425] = 16'h0380; font_rom[426] = 16'h0700; font_rom[427] = 16'h0600;
        font_rom[428] = 16'h0C00; font_rom[429] = 16'h1E00; font_rom[430] = 16'h1FFC; font_rom[431] = 16'h1FFE;
        font_rom[432] = 16'h07F0; font_rom[433] = 16'h0FF0; font_rom[434] = 16'h1E38; font_rom[435] = 16'h1C3C;
        font_rom[436] = 16'h3C3C; font_rom[437] = 16'h387E; font_rom[438] = 16'h38FE; font_rom[439] = 16'h39FE;
        font_rom[440] = 16'h7F9E; font_rom[441] = 16'h3F1E; font_rom[442] = 16'h3E1E; font_rom[443] = 16'h3C1C;
        font_rom[444] = 16'h1C38; font_rom[445] = 16'h1C78; font_rom[446] = 16'h0FF0; font_rom[447] = 16'h07E0;
        font_rom[448] = 16'h07C0; font_rom[449] = 16'h0FC0; font_rom[450] = 16'h1FC0; font_rom[451] = 16'h1DC0;
        font_rom[452] = 16'h01C0; font_rom[453] = 16'h01C0; font_rom[454] = 16'h01C0; font_rom[455] = 16'h01C0;
        font_rom[456] = 16'h01C0; font_rom[457] = 16'h01C0; font_rom[458] = 16'h01C0; font_rom[459] = 16'h01C0;
        font_rom[460] = 16'h01C0; font_rom[461] = 16'h03E0; font_rom[462] = 16'h1FFC; font_rom[463] = 16'h1FFC;
        font_rom[464] = 16'h07E0; font_rom[465] = 16'h0FF0; font_rom[466] = 16'h1CF8; font_rom[467] = 16'h1838;
        font_rom[468] = 16'h0038; font_rom[469] = 16'h0038; font_rom[470] = 16'h0038; font_rom[471] = 16'h0070;
        font_rom[472] = 16'h00F0; font_rom[473] = 16'h01E0; font_rom[474] = 16'h03C0; font_rom[475] = 16'h0780;
        font_rom[476] = 16'h0E00; font_rom[477] = 16'h1F00; font_rom[478] = 16'h1FFC; font_rom[479] = 16'h1FFE;
        font_rom[480] = 16'h3FC0; font_rom[481] = 16'h3FE0; font_rom[482] = 16'h01F0; font_rom[483] = 16'h0070;
        font_rom[484] = 16'h00F0; font_rom[485] = 16'h01E0; font_rom[486] = 16'h0FC0; font_rom[487] = 16'h0FE0;
        font_rom[488] = 16'h01F0; font_rom[489] = 16'h0070; font_rom[490] = 16'h0038; font_rom[491] = 16'h0038;
        font_rom[492] = 16'h0070; font_rom[493] = 16'h00F0; font_rom[494] = 16'h3FE0; font_rom[495] = 16'h3FC0;
        font_rom[496] = 16'h00F0; font_rom[497] = 16'h01F0; font_rom[498] = 16'h01F0; font_rom[499] = 16'h03F0;
        font_rom[500] = 16'h07F0; font_rom[501] = 16'h0E70; font_rom[502] = 16'h1E70; font_rom[503] = 16'h1C70;
        font_rom[504] = 16'h3870; font_rom[505] = 16'h7CF8; font_rom[506] = 16'h7FFE; font_rom[507] = 16'h7FFE;
        font_rom[508] = 16'h00F8; font_rom[509] = 16'h0070; font_rom[510] = 16'h0070; font_rom[511] = 16'h0070;
        font_rom[512] = 16'h3FF0; font_rom[513] = 16'h3FF0; font_rom[514] = 16'h3C00; font_rom[515] = 16'h3000;
        font_rom[516] = 16'h3000; font_rom[517] = 16'h3C00; font_rom[518] = 16'h3FC0; font_rom[519] = 16'h3FE0;
        font_rom[520] = 16'h01F0; font_rom[521] = 16'h0070; font_rom[522] = 16'h0038; font_rom[523] = 16'h0070;
        font_rom[524] = 16'h0070; font_rom[525] = 16'h01F0; font_rom[526] = 16'h3FC0; font_rom[527] = 16'h3FC0;
        font_rom[528] = 16'h03F8; font_rom[529] = 16'h07F8; font_rom[530] = 16'h0F80; font_rom[531] = 16'h1E00;
        font_rom[532] = 16'h1C00; font_rom[533] = 16'h1C00; font_rom[534] = 16'h1DF0; font_rom[535] = 16'h1FF8;
        font_rom[536] = 16'h3E3C; font_rom[537] = 16'h3C1C; font_rom[538] = 16'h181E; font_rom[539] = 16'h181C;
        font_rom[540] = 16'h1C1C; font_rom[541] = 16'h1E38; font_rom[542] = 16'h0FF0; font_rom[543] = 16'h07F0;
        font_rom[544] = 16'h3FFC; font_rom[545] = 16'h1FFC; font_rom[546] = 16'h003C; font_rom[547] = 16'h0018;
        font_rom[548] = 16'h0038; font_rom[549] = 16'h0070; font_rom[550] = 16'h0070; font_rom[551] = 16'h00E0;
        font_rom[552] = 16'h00E0; font_rom[553] = 16'h01C0; font_rom[554] = 16'h01C0; font_rom[555] = 16'h0380;
        font_rom[556] = 16'h0380; font_rom[557] = 16'h0700; font_rom[558] = 16'h0700; font_rom[559] = 16'h0E00;
        font_rom[560] = 16'h0FF0; font_rom[561] = 16'h0FF8; font_rom[562] = 16'h1C38; font_rom[563] = 16'h1C3C;
        font_rom[564] = 16'h1C38; font_rom[565] = 16'h1FF8; font_rom[566] = 16'h07F0; font_rom[567] = 16'h07F0;
        font_rom[568] = 16'h0FF8; font_rom[569] = 16'h1C38; font_rom[570] = 16'h1C1C; font_rom[571] = 16'h381C;
        font_rom[572] = 16'h3C1C; font_rom[573] = 16'h1E38; font_rom[574] = 16'h0FF8; font_rom[575] = 16'h0FF0;
        font_rom[576] = 16'h07E0; font_rom[577] = 16'h0FF0; font_rom[578] = 16'h1E78; font_rom[579] = 16'h3C38;
        font_rom[580] = 16'h381C; font_rom[581] = 16'h381C; font_rom[582] = 16'h383C; font_rom[583] = 16'h1C7C;
        font_rom[584] = 16'h1FFC; font_rom[585] = 16'h0FFC; font_rom[586] = 16'h0018; font_rom[587] = 16'h0038;
        font_rom[588] = 16'h0078; font_rom[589] = 16'h01F0; font_rom[590] = 16'h1FE0; font_rom[591] = 16'h1FC0;
        font_rom[592] = 16'h03C0; font_rom[593] = 16'h03E0; font_rom[594] = 16'h03E0; font_rom[595] = 16'h03E0;
        font_rom[596] = 16'h03C0; font_rom[597] = 16'h0180; font_rom[598] = 16'h0000; font_rom[599] = 16'h0000;
        font_rom[600] = 16'h0000; font_rom[601] = 16'h0000; font_rom[602] = 16'h0180; font_rom[603] = 16'h03C0;
        font_rom[604] = 16'h03E0; font_rom[605] = 16'h03E0; font_rom[606] = 16'h03E0; font_rom[607] = 16'h03C0;
        font_rom[608] = 16'h0000; font_rom[609] = 16'h0000; font_rom[610] = 16'h0000; font_rom[611] = 16'h0000;
        font_rom[612] = 16'h0000; font_rom[613] = 16'h0000; font_rom[614] = 16'h3FFF; font_rom[615] = 16'h3FFF;
        font_rom[616] = 16'h3FFF; font_rom[617] = 16'h0000; font_rom[618] = 16'h0000; font_rom[619] = 16'h0000;
        font_rom[620] = 16'h0000; font_rom[621] = 16'h0000; font_rom[622] = 16'h0000; font_rom[623] = 16'h0000;
        font_rom[624] = 16'h0000; font_rom[625] = 16'h0000; font_rom[626] = 16'h1FF8; font_rom[627] = 16'h1FF8;
        font_rom[628] = 16'h1FF8; font_rom[629] = 16'h1FF8; font_rom[630] = 16'h1FF8; font_rom[631] = 16'h1FF8;
        font_rom[632] = 16'h1FF8; font_rom[633] = 16'h1FF8; font_rom[634] = 16'h1FF8; font_rom[635] = 16'h1FF8;
        font_rom[636] = 16'h1FF8; font_rom[637] = 16'h0000; font_rom[638] = 16'h0000; font_rom[639] = 16'h0000;
        font_rom[640] = 16'h0070; font_rom[641] = 16'h0060; font_rom[642] = 16'h00E0; font_rom[643] = 16'h00C0;
        font_rom[644] = 16'h00C0; font_rom[645] = 16'h01C0; font_rom[646] = 16'h0180; font_rom[647] = 16'h0380;
        font_rom[648] = 16'h0300; font_rom[649] = 16'h0700; font_rom[650] = 16'h0600; font_rom[651] = 16'h0600;
        font_rom[652] = 16'h0E00; font_rom[653] = 16'h0C00; font_rom[654] = 16'h1C00; font_rom[655] = 16'h1800;
        font_rom[656] = 16'h0000; font_rom[657] = 16'h381E; font_rom[658] = 16'h3C1C; font_rom[659] = 16'h1E3C;
        font_rom[660] = 16'h0E78; font_rom[661] = 16'h0FF0; font_rom[662] = 16'h07E0; font_rom[663] = 16'h03E0;
        font_rom[664] = 16'h07E0; font_rom[665] = 16'h07F0; font_rom[666] = 16'h0F70; font_rom[667] = 16'h1E78;
        font_rom[668] = 16'h3C3C; font_rom[669] = 16'h381E; font_rom[670] = 16'h381E; font_rom[671] = 16'h0000;
        font_rom[672] = 16'h03C0; font_rom[673] = 16'h03C0; font_rom[674] = 16'h03C0; font_rom[675] = 16'h03C0;
        font_rom[676] = 16'h01C0; font_rom[677] = 16'h01C0; font_rom[678] = 16'h1FF8; font_rom[679] = 16'hFFFF;
        font_rom[680] = 16'hFFFF; font_rom[681] = 16'h1FF8; font_rom[682] = 16'h01C0; font_rom[683] = 16'h01C0;
        font_rom[684] = 16'h03C0; font_rom[685] = 16'h03C0; font_rom[686] = 16'h03C0; font_rom[687] = 16'h03C0;
        font_rom[688] = 16'h0000; font_rom[689] = 16'h0000; font_rom[690] = 16'h0000; font_rom[691] = 16'h0000;
        font_rom[692] = 16'h0000; font_rom[693] = 16'h0000; font_rom[694] = 16'h0000; font_rom[695] = 16'h0000;
        font_rom[696] = 16'h0000; font_rom[697] = 16'h0000; font_rom[698] = 16'h0000; font_rom[699] = 16'h0000;
        font_rom[700] = 16'h0000; font_rom[701] = 16'h0000; font_rom[702] = 16'h0000; font_rom[703] = 16'h0000;
        font_rom[704] = 16'h0000; font_rom[705] = 16'h0000; font_rom[706] = 16'h0000; font_rom[707] = 16'h0000;
        font_rom[708] = 16'h0000; font_rom[709] = 16'h0000; font_rom[710] = 16'h0000; font_rom[711] = 16'h0000;
        font_rom[712] = 16'h0000; font_rom[713] = 16'h0000; font_rom[714] = 16'h0000; font_rom[715] = 16'h0000;
        font_rom[716] = 16'h0000; font_rom[717] = 16'h0000; font_rom[718] = 16'h0000; font_rom[719] = 16'h0000;
        font_rom[720] = 16'h0000; font_rom[721] = 16'h0000; font_rom[722] = 16'h0000; font_rom[723] = 16'h0000;
        font_rom[724] = 16'h0000; font_rom[725] = 16'h0000; font_rom[726] = 16'h0000; font_rom[727] = 16'h0000;
        font_rom[728] = 16'h0000; font_rom[729] = 16'h0000; font_rom[730] = 16'h0000; font_rom[731] = 16'h0000;
        font_rom[732] = 16'h0000; font_rom[733] = 16'h0000; font_rom[734] = 16'h0000; font_rom[735] = 16'h0000;
        font_rom[736] = 16'h0000; font_rom[737] = 16'h0000; font_rom[738] = 16'h0000; font_rom[739] = 16'h0000;
        font_rom[740] = 16'h0000; font_rom[741] = 16'h0000; font_rom[742] = 16'h0000; font_rom[743] = 16'h0000;
        font_rom[744] = 16'h0000; font_rom[745] = 16'h0000; font_rom[746] = 16'h0000; font_rom[747] = 16'h0000;
        font_rom[748] = 16'h0000; font_rom[749] = 16'h0000; font_rom[750] = 16'h0000; font_rom[751] = 16'h0000;
        font_rom[752] = 16'h0000; font_rom[753] = 16'h0000; font_rom[754] = 16'h0000; font_rom[755] = 16'h0000;
        font_rom[756] = 16'h0000; font_rom[757] = 16'h0000; font_rom[758] = 16'h0000; font_rom[759] = 16'h0000;
        font_rom[760] = 16'h0000; font_rom[761] = 16'h0000; font_rom[762] = 16'h0000; font_rom[763] = 16'h0000;
        font_rom[764] = 16'h0000; font_rom[765] = 16'h0000; font_rom[766] = 16'h0000; font_rom[767] = 16'h0000;
        font_rom[768] = 16'h0000; font_rom[769] = 16'h0000; font_rom[770] = 16'h0000; font_rom[771] = 16'h0000;
        font_rom[772] = 16'h0000; font_rom[773] = 16'h0000; font_rom[774] = 16'h0000; font_rom[775] = 16'h0000;
        font_rom[776] = 16'h0000; font_rom[777] = 16'h0000; font_rom[778] = 16'h0000; font_rom[779] = 16'h0000;
        font_rom[780] = 16'h0000; font_rom[781] = 16'h0000; font_rom[782] = 16'h0000; font_rom[783] = 16'h0000;
        font_rom[784] = 16'h0000; font_rom[785] = 16'h0000; font_rom[786] = 16'h0000; font_rom[787] = 16'h0000;
        font_rom[788] = 16'h0000; font_rom[789] = 16'h0000; font_rom[790] = 16'h0000; font_rom[791] = 16'h0000;
        font_rom[792] = 16'h0000; font_rom[793] = 16'h0000; font_rom[794] = 16'h0000; font_rom[795] = 16'h0000;
        font_rom[796] = 16'h0000; font_rom[797] = 16'h0000; font_rom[798] = 16'h0000; font_rom[799] = 16'h0000;
        font_rom[800] = 16'h0000; font_rom[801] = 16'h0000; font_rom[802] = 16'h0000; font_rom[803] = 16'h0000;
        font_rom[804] = 16'h0000; font_rom[805] = 16'h0000; font_rom[806] = 16'h0000; font_rom[807] = 16'h0000;
        font_rom[808] = 16'h0000; font_rom[809] = 16'h0000; font_rom[810] = 16'h0000; font_rom[811] = 16'h0000;
        font_rom[812] = 16'h0000; font_rom[813] = 16'h0000; font_rom[814] = 16'h0000; font_rom[815] = 16'h0000;
        font_rom[816] = 16'h0000; font_rom[817] = 16'h0000; font_rom[818] = 16'h0000; font_rom[819] = 16'h0000;
        font_rom[820] = 16'h0000; font_rom[821] = 16'h0000; font_rom[822] = 16'h0000; font_rom[823] = 16'h0000;
        font_rom[824] = 16'h0000; font_rom[825] = 16'h0000; font_rom[826] = 16'h0000; font_rom[827] = 16'h0000;
        font_rom[828] = 16'h0000; font_rom[829] = 16'h0000; font_rom[830] = 16'h0000; font_rom[831] = 16'h0000;
        font_rom[832] = 16'h0000; font_rom[833] = 16'h0000; font_rom[834] = 16'h0000; font_rom[835] = 16'h0000;
        font_rom[836] = 16'h0000; font_rom[837] = 16'h0000; font_rom[838] = 16'h0000; font_rom[839] = 16'h0000;
        font_rom[840] = 16'h0000; font_rom[841] = 16'h0000; font_rom[842] = 16'h0000; font_rom[843] = 16'h0000;
        font_rom[844] = 16'h0000; font_rom[845] = 16'h0000; font_rom[846] = 16'h0000; font_rom[847] = 16'h0000;
        font_rom[848] = 16'h0000; font_rom[849] = 16'h0000; font_rom[850] = 16'h0000; font_rom[851] = 16'h0000;
        font_rom[852] = 16'h0000; font_rom[853] = 16'h0000; font_rom[854] = 16'h0000; font_rom[855] = 16'h0000;
        font_rom[856] = 16'h0000; font_rom[857] = 16'h0000; font_rom[858] = 16'h0000; font_rom[859] = 16'h0000;
        font_rom[860] = 16'h0000; font_rom[861] = 16'h0000; font_rom[862] = 16'h0000; font_rom[863] = 16'h0000;
        font_rom[864] = 16'h0000; font_rom[865] = 16'h0000; font_rom[866] = 16'h0000; font_rom[867] = 16'h0000;
        font_rom[868] = 16'h0000; font_rom[869] = 16'h0000; font_rom[870] = 16'h0000; font_rom[871] = 16'h0000;
        font_rom[872] = 16'h0000; font_rom[873] = 16'h0000; font_rom[874] = 16'h0000; font_rom[875] = 16'h0000;
        font_rom[876] = 16'h0000; font_rom[877] = 16'h0000; font_rom[878] = 16'h0000; font_rom[879] = 16'h0000;
        font_rom[880] = 16'h0000; font_rom[881] = 16'h0000; font_rom[882] = 16'h0000; font_rom[883] = 16'h0000;
        font_rom[884] = 16'h0000; font_rom[885] = 16'h0000; font_rom[886] = 16'h0000; font_rom[887] = 16'h0000;
        font_rom[888] = 16'h0000; font_rom[889] = 16'h0000; font_rom[890] = 16'h0000; font_rom[891] = 16'h0000;
        font_rom[892] = 16'h0000; font_rom[893] = 16'h0000; font_rom[894] = 16'h0000; font_rom[895] = 16'h0000;
        font_rom[896] = 16'h0000; font_rom[897] = 16'h0000; font_rom[898] = 16'h0000; font_rom[899] = 16'h0000;
        font_rom[900] = 16'h0000; font_rom[901] = 16'h0000; font_rom[902] = 16'h0000; font_rom[903] = 16'h0000;
        font_rom[904] = 16'h0000; font_rom[905] = 16'h0000; font_rom[906] = 16'h0000; font_rom[907] = 16'h0000;
        font_rom[908] = 16'h0000; font_rom[909] = 16'h0000; font_rom[910] = 16'h0000; font_rom[911] = 16'h0000;
        font_rom[912] = 16'h0000; font_rom[913] = 16'h0000; font_rom[914] = 16'h0000; font_rom[915] = 16'h0000;
        font_rom[916] = 16'h0000; font_rom[917] = 16'h0000; font_rom[918] = 16'h0000; font_rom[919] = 16'h0000;
        font_rom[920] = 16'h0000; font_rom[921] = 16'h0000; font_rom[922] = 16'h0000; font_rom[923] = 16'h0000;
        font_rom[924] = 16'h0000; font_rom[925] = 16'h0000; font_rom[926] = 16'h0000; font_rom[927] = 16'h0000;
        font_rom[928] = 16'h0000; font_rom[929] = 16'h0000; font_rom[930] = 16'h0000; font_rom[931] = 16'h0000;
        font_rom[932] = 16'h0000; font_rom[933] = 16'h0000; font_rom[934] = 16'h0000; font_rom[935] = 16'h0000;
        font_rom[936] = 16'h0000; font_rom[937] = 16'h0000; font_rom[938] = 16'h0000; font_rom[939] = 16'h0000;
        font_rom[940] = 16'h0000; font_rom[941] = 16'h0000; font_rom[942] = 16'h0000; font_rom[943] = 16'h0000;
        font_rom[944] = 16'h0000; font_rom[945] = 16'h0000; font_rom[946] = 16'h0000; font_rom[947] = 16'h0000;
        font_rom[948] = 16'h0000; font_rom[949] = 16'h0000; font_rom[950] = 16'h0000; font_rom[951] = 16'h0000;
        font_rom[952] = 16'h0000; font_rom[953] = 16'h0000; font_rom[954] = 16'h0000; font_rom[955] = 16'h0000;
        font_rom[956] = 16'h0000; font_rom[957] = 16'h0000; font_rom[958] = 16'h0000; font_rom[959] = 16'h0000;
        font_rom[960] = 16'h0000; font_rom[961] = 16'h0000; font_rom[962] = 16'h0000; font_rom[963] = 16'h0000;
        font_rom[964] = 16'h0000; font_rom[965] = 16'h0000; font_rom[966] = 16'h0000; font_rom[967] = 16'h0000;
        font_rom[968] = 16'h0000; font_rom[969] = 16'h0000; font_rom[970] = 16'h0000; font_rom[971] = 16'h0000;
        font_rom[972] = 16'h0000; font_rom[973] = 16'h0000; font_rom[974] = 16'h0000; font_rom[975] = 16'h0000;
        font_rom[976] = 16'h0000; font_rom[977] = 16'h0000; font_rom[978] = 16'h0000; font_rom[979] = 16'h0000;
        font_rom[980] = 16'h0000; font_rom[981] = 16'h0000; font_rom[982] = 16'h0000; font_rom[983] = 16'h0000;
        font_rom[984] = 16'h0000; font_rom[985] = 16'h0000; font_rom[986] = 16'h0000; font_rom[987] = 16'h0000;
        font_rom[988] = 16'h0000; font_rom[989] = 16'h0000; font_rom[990] = 16'h0000; font_rom[991] = 16'h0000;
        font_rom[992] = 16'h0000; font_rom[993] = 16'h0000; font_rom[994] = 16'h0000; font_rom[995] = 16'h0000;
        font_rom[996] = 16'h0000; font_rom[997] = 16'h0000; font_rom[998] = 16'h0000; font_rom[999] = 16'h0000;
        font_rom[1000] = 16'h0000; font_rom[1001] = 16'h0000; font_rom[1002] = 16'h0000; font_rom[1003] = 16'h0000;
        font_rom[1004] = 16'h0000; font_rom[1005] = 16'h0000; font_rom[1006] = 16'h0000; font_rom[1007] = 16'h0000;
        font_rom[1008] = 16'h0000; font_rom[1009] = 16'h0000; font_rom[1010] = 16'h0000; font_rom[1011] = 16'h0000;
        font_rom[1012] = 16'h0000; font_rom[1013] = 16'h0000; font_rom[1014] = 16'h0000; font_rom[1015] = 16'h0000;
        font_rom[1016] = 16'h0000; font_rom[1017] = 16'h0000; font_rom[1018] = 16'h0000; font_rom[1019] = 16'h0000;
        font_rom[1020] = 16'h0000; font_rom[1021] = 16'h0000; font_rom[1022] = 16'h0000; font_rom[1023] = 16'h0000;
    end

    // ---------------- 像素坐标 ----------------
    reg [10:0] px = 11'd0, py = 11'd0;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin px <= 11'd0; py <= 11'd0; end
        else if (beat) begin
            if (s_axis_tlast) begin px <= 11'd0; py <= py + 11'd1; end
            else                px <= px + 11'd1;
            if (s_axis_tuser) py <= 11'd1;
        end
    end

    localparam integer SH  = (SCALE == 1) ? 0 : (SCALE == 2) ? 1 : 2;
    localparam integer CW  = 16 * SCALE;

    wire in_panel = (px >= X0) && (px < X0 + COLS*CW) &&
                    (py >= Y0) && (py <= Y0 + (ROWS-1)*PITCH + CW);
    wire [10:0] rx = px - X0;
    wire [10:0] ry = py - Y0;

    wire [4:0]  cx    = rx >> (4 + SH);
    wire [3:0]  sub_x = (rx >> SH);

    // 行：按 PITCH 划分（支持非 2 的幂），限定 ROWS 行
    reg [2:0] cy;
    reg [10:0] ry_in;
    always @* begin
        cy = 3'd0; ry_in = ry;
        if      (ry <  PITCH*1) begin cy = 3'd0; ry_in = ry;             end
        else if (ry <  PITCH*2) begin cy = 3'd1; ry_in = ry - PITCH*1;   end
        else if (ry <  PITCH*3) begin cy = 3'd2; ry_in = ry - PITCH*2;   end
        else if (ry <  PITCH*4) begin cy = 3'd3; ry_in = ry - PITCH*3;   end
        else if (ry <  PITCH*5) begin cy = 3'd4; ry_in = ry - PITCH*4;   end
        else if (ry <  PITCH*6) begin cy = 3'd5; ry_in = ry - PITCH*5;   end
        else if (ry <  PITCH*7) begin cy = 3'd6; ry_in = ry - PITCH*6;   end
        else                    begin cy = 3'd7; ry_in = ry - PITCH*7;   end
    end

    wire [3:0] sub_y = (ry_in >> SH);
    wire glyph_ok = (ry_in < CW);              // 行内有效（超出行高留空）

    wire [10:0] cell_addr = {cy, cx[4:0]};
    reg  [7:0]  ascii_ch;
    always @* ascii_ch = text_ram[cell_addr[7:0]];

    reg [5:0] fidx;
    always @* begin
        if (ascii_ch >= 8'd65 && ascii_ch <= 8'd90)       fidx = ascii_ch - 8'd64;
        else if (ascii_ch >= 8'd97 && ascii_ch <= 8'd122) fidx = ascii_ch - 8'd96;
        else if (ascii_ch >= 8'd48 && ascii_ch <= 8'd57)  fidx = ascii_ch - 8'd48 + 6'd27;
        else if (ascii_ch == 8'd58)  fidx = 6'd37;
        else if (ascii_ch == 8'd45)  fidx = 6'd38;
        else if (ascii_ch == 8'd46)  fidx = 6'd39;
        else if (ascii_ch == 8'd47)  fidx = 6'd40;
        else if (ascii_ch == 8'd120) fidx = 6'd41;
        else if (ascii_ch == 8'd43)  fidx = 6'd42;
        else fidx = 6'd0;
    end

    wire [9:0]  faddr = {fidx, sub_y};
    reg  [15:0] frow;
    always @* frow = font_rom[faddr];
    wire pix_on = in_panel && enable && glyph_ok && frow[15 - sub_x];

    localparam [23:0] BG_COL  = 24'h00_00_00;   // 黑色底
    localparam [23:0] TXT_COL = 24'h70_FF_90;   // 绿色字

    // 半透明混合：out = (src*(255-a) + bg*a) / 256
    function [7:0] mix;
        input [7:0] src;
        input [7:0] bg;
        input [8:0] a;
        reg   [16:0] t;
        begin
            t = src * (9'd255 - a) + bg * a;
            mix = t[15:8];
        end
    endfunction

    reg [23:0] bg_mix;
    always @* begin
        bg_mix[7:0]   = mix(s_axis_tdata[7:0],   BG_COL[7:0],   ALPHA_BG);
        bg_mix[15:8]  = mix(s_axis_tdata[15:8],  BG_COL[15:8],  ALPHA_BG);
        bg_mix[23:16] = mix(s_axis_tdata[23:16], BG_COL[23:16], ALPHA_BG);
    end

    reg [23:0] mdata_r;
    always @* begin
        if (!enable || !in_panel) mdata_r = s_axis_tdata;
        else if (pix_on)          mdata_r = TXT_COL;
        else                      mdata_r = bg_mix;
    end
    assign m_axis_tdata = mdata_r;

endmodule
`default_nettype wire
