`timescale 1ns / 1ps

module tmds_encoder (
    input  wire       clk,
    input  wire       rst,
    input  wire       de,
    input  wire [1:0] control,
    input  wire [7:0] data,

    output reg  [9:0] symbol
);

    reg [8:0] q_m;
    reg       use_xnor;

    integer i;
    integer ones_data;
    integer ones_q;
    integer balance;

    // Running disparity: accumulated difference between ones and zeros.
    reg signed [5:0] disparity;

    // Stage 1: minimize transitions in the eight data bits.
    always @(*) begin
        ones_data = 0;

        for (i = 0; i < 8; i = i + 1)
            ones_data = ones_data + data[i];

        use_xnor = (ones_data > 4) ||
                   ((ones_data == 4) && (data[0] == 1'b0));

        q_m = 9'd0;
        q_m[0] = data[0];

        for (i = 1; i < 8; i = i + 1) begin
            if (use_xnor)
                q_m[i] = ~(q_m[i-1] ^ data[i]);
            else
                q_m[i] = q_m[i-1] ^ data[i];
        end

        q_m[8] = !use_xnor;

        ones_q = 0;

        for (i = 0; i < 8; i = i + 1)
            ones_q = ones_q + q_m[i];

        balance = 2 * ones_q - 8;
    end

    // Stage 2: balance ones and zeros across consecutive symbols.
    // Output is registered, with one pixel-clock latency.
    always @(posedge clk) begin
        if (rst) begin
            symbol    <= 10'b1101010100;
            disparity <= 6'sd0;
        end else if (!de) begin
            // During blanking, transmit control symbols.
            // Blue channel: control = {vsync, hsync}.
            // Red/green channels: control = 2'b00.
            disparity <= 6'sd0;

            case (control)
                2'b00: symbol <= 10'b1101010100;
                2'b01: symbol <= 10'b0010101011;
                2'b10: symbol <= 10'b0101010100;
                2'b11: symbol <= 10'b1010101011;
                default: symbol <= 10'b1101010100;
            endcase
        end else if ((disparity == 0) || (balance == 0)) begin
            symbol[9] <= ~q_m[8];
            symbol[8] <=  q_m[8];

            if (q_m[8]) begin
                symbol[7:0] <= q_m[7:0];
                disparity   <= disparity + balance;
            end else begin
                symbol[7:0] <= ~q_m[7:0];
                disparity   <= disparity - balance;
            end
        end else if (((disparity > 0) && (balance > 0)) ||
                     ((disparity < 0) && (balance < 0))) begin
            symbol <= {1'b1, q_m[8], ~q_m[7:0]};

            if (q_m[8])
                disparity <= disparity - balance + 2;
            else
                disparity <= disparity - balance;
        end else begin
            symbol <= {1'b0, q_m[8], q_m[7:0]};

            if (q_m[8])
                disparity <= disparity + balance;
            else
                disparity <= disparity + balance - 2;
        end
    end

endmodule