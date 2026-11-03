`timescale 1ns/1ps
// Command and counter share FCLK0. Zero disables; updates latch at frame boundaries.
module servo_pwm #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer PERIOD_US = 20000,
    parameter integer MIN_W_US = 500,
    parameter integer MAX_W_US = 2500
) (
    input wire clk,
    input wire rst_n,
    input wire [15:0] pulse_us,
    output reg pwm,
    output wire [15:0] active_us
);
    localparam integer TICKS_PER_US = CLK_FREQ_HZ / 1000000;
    localparam integer DIV_WIDTH = (TICKS_PER_US > 1) ? $clog2(TICKS_PER_US) : 1;
    reg [DIV_WIDTH-1:0] div_cnt;
    reg [15:0] us_cnt;
    reg [15:0] pulse_lat;
    wire [15:0] bounded_pulse = (pulse_us == 0) ? 16'd0 :
                               (pulse_us < MIN_W_US) ? MIN_W_US :
                               (pulse_us > MAX_W_US) ? MAX_W_US : pulse_us;
    assign active_us = pulse_lat;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 0;
            us_cnt <= 0;
            pulse_lat <= 0;
            pwm <= 0;
        end else if (div_cnt == TICKS_PER_US - 1) begin
            div_cnt <= 0;
            if (us_cnt == PERIOD_US - 1) begin
                us_cnt <= 0;
                pulse_lat <= bounded_pulse;
                pwm <= (bounded_pulse != 0);
            end else begin
                us_cnt <= us_cnt + 1'b1;
                if (us_cnt + 16'd1 >= pulse_lat)
                    pwm <= 0;
            end
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end
endmodule
