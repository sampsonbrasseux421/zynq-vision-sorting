`timescale 1ns / 1ps

module tmds_serializer (
    input  wire       clk_pixel,
    input  wire       clk_5x,
    input  wire       rst,
    input  wire [9:0] data10,

    output wire       out_p,
    output wire       out_n
);

    wire shift1;
    wire shift2;
    wire serial_data;

    // A 10-bit serializer uses one MASTER and one SLAVE.
    // Bits are transmitted least significant bit first.
    OSERDESE2 #(
        .DATA_RATE_OQ   ("DDR"),
        .DATA_RATE_TQ   ("SDR"),
        .DATA_WIDTH     (10),
        .SERDES_MODE    ("MASTER"),
        .TRISTATE_WIDTH (1),
        .INIT_OQ        (1'b0),
        .INIT_TQ        (1'b0),
        .SRVAL_OQ       (1'b0),
        .SRVAL_TQ       (1'b0),
        .TBYTE_CTL      ("FALSE"),
        .TBYTE_SRC      ("FALSE")
    ) serializer_master (
        .CLK       (clk_5x),
        .CLKDIV    (clk_pixel),
        .RST       (rst),
        .OCE       (1'b1),

        .D1        (data10[0]),
        .D2        (data10[1]),
        .D3        (data10[2]),
        .D4        (data10[3]),
        .D5        (data10[4]),
        .D6        (data10[5]),
        .D7        (data10[6]),
        .D8        (data10[7]),

        .SHIFTIN1  (shift1),
        .SHIFTIN2  (shift2),
        .SHIFTOUT1 (),
        .SHIFTOUT2 (),

        .OQ        (serial_data),
        .OFB       (),

        .T1        (1'b0),
        .T2        (1'b0),
        .T3        (1'b0),
        .T4        (1'b0),
        .TCE       (1'b0),
        .TBYTEIN   (1'b0),
        .TBYTEOUT  (),
        .TFB       (),
        .TQ        ()
    );

    // In 10-bit mode, bits 8 and 9 enter SLAVE D3 and D4.
    OSERDESE2 #(
        .DATA_RATE_OQ   ("DDR"),
        .DATA_RATE_TQ   ("SDR"),
        .DATA_WIDTH     (10),
        .SERDES_MODE    ("SLAVE"),
        .TRISTATE_WIDTH (1),
        .INIT_OQ        (1'b0),
        .INIT_TQ        (1'b0),
        .SRVAL_OQ       (1'b0),
        .SRVAL_TQ       (1'b0),
        .TBYTE_CTL      ("FALSE"),
        .TBYTE_SRC      ("FALSE")
    ) serializer_slave (
        .CLK       (clk_5x),
        .CLKDIV    (clk_pixel),
        .RST       (rst),
        .OCE       (1'b1),

        .D1        (1'b0),
        .D2        (1'b0),
        .D3        (data10[8]),
        .D4        (data10[9]),
        .D5        (1'b0),
        .D6        (1'b0),
        .D7        (1'b0),
        .D8        (1'b0),

        .SHIFTIN1  (1'b0),
        .SHIFTIN2  (1'b0),
        .SHIFTOUT1 (shift1),
        .SHIFTOUT2 (shift2),

        .OQ        (),
        .OFB       (),

        .T1        (1'b0),
        .T2        (1'b0),
        .T3        (1'b0),
        .T4        (1'b0),
        .TCE       (1'b0),
        .TBYTEIN   (1'b0),
        .TBYTEOUT  (),
        .TFB       (),
        .TQ        ()
    );

    // Bank 35 uses a 3.3 V supply on this board.
    OBUFDS #(
        .IOSTANDARD ("TMDS_33"),
        .SLEW       ("FAST")
    ) output_buffer (
        .I  (serial_data),
        .O  (out_p),
        .OB (out_n)
    );

endmodule