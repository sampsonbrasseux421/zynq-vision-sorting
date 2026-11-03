//=============================================================================
// color_blob_detector_roi.v —— 带固定 ROI 的检测器包装
//
// 为什么需要这一层：
//   Vivado 的 Block Design 会为每个 module reference 缓存一份模块定义
//   (xxx.gen/sources_1/bd/mref/<模块名>/component.xml)。
//   直接修改底层 RTL 后，BD 仍按旧缓存例化，新增的参数不会生效。
//   这里用一个新模块名 + 显式参数例化，强制 Vivado 重新生成。
//
// ROI 依据实测画面（1024x600 显示坐标）：
//   传送带白色区域  y 约 196..462，左右留边 x 60..1000
//   最终取          x 120..1016, y 200..450
//   作用：排除上方绿色桌布、下方蓝色夹爪、四周黑色框架
//=============================================================================
`default_nettype none

module color_blob_detector_roi (
    input  wire        aclk,
    input  wire        aresetn,

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

    input  wire [7:0]  rd_sel,
    output wire [31:0] rd_data,
    output wire [31:0] frame_cnt,
    output wire        done_toggle
);

    color_blob_detector #(
        .H_DIV  (4),
        .V_DIV  (4),
        .ROI_X0 (130),
        .ROI_X1 (1010),
        .ROI_Y0 (215),
        .ROI_Y1 (435)
    ) u_det (
        .aclk          (aclk),
        .aresetn       (aresetn),
        .s_axis_tdata  (s_axis_tdata),
        .s_axis_tvalid (s_axis_tvalid),
        .s_axis_tready (s_axis_tready),
        .s_axis_tlast  (s_axis_tlast),
        .s_axis_tuser  (s_axis_tuser),
        .m_axis_tdata  (m_axis_tdata),
        .m_axis_tvalid (m_axis_tvalid),
        .m_axis_tready (m_axis_tready),
        .m_axis_tlast  (m_axis_tlast),
        .m_axis_tuser  (m_axis_tuser),
        .rd_sel        (rd_sel),
        .rd_data       (rd_data),
        .frame_cnt     (frame_cnt),
        .done_toggle   (done_toggle)
    );

endmodule

`default_nettype wire
