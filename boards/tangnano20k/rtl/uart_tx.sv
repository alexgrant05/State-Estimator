`timescale 1ns/1ps
`default_nettype none

// uart_tx
//
// Minimal 8N1 UART transmitter (1 start bit, 8 data bits LSB-first,
// 1 stop bit, no parity) with a ready/valid byte input. Used on the
// Tang Nano 20K bring-up board to stream debug data to the host over the
// on-board USB-serial bridge. Not part of the KR260 build.
//
// The whole frame lives in one 10-bit shift register {stop, data, start}:
// tx is always shift[0], and every bit period the register shifts right
// with 1s filling in from the top, so after the stop bit the line is
// naturally left idle-high. in_ready is high exactly when no bits are
// left to send, so a new byte can be accepted on the cycle right after
// the previous stop bit finishes (back-to-back frames, no idle gap).
//
// CLK_HZ / BAUD must be >= 2 and is truncated, so pick a pair whose
// rounding error is small: 27 MHz / 115200 = 234.375 -> 234 clocks per
// bit, a 0.16% baud error (UART receivers tolerate roughly 2-3%).
//
// Inputs:  clk, rst_n, in_data, in_valid
// Outputs: in_ready, tx
module uart_tx #(
    parameter int CLK_HZ = 27_000_000,
    parameter int BAUD   = 115_200
) (
    input  logic       clk,
    input  logic       rst_n,

    input  logic [7:0] in_data,
    input  logic       in_valid,
    output logic       in_ready,

    output logic       tx
);

    localparam int CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam int COUNT_WIDTH  = $clog2(CLKS_PER_BIT);
    localparam logic [COUNT_WIDTH-1:0] LAST_COUNT = COUNT_WIDTH'(CLKS_PER_BIT - 1);

    logic [9:0]             shift;      // {stop, data[7:0], start}; tx = shift[0]
    logic [3:0]             bits_left;  // bits of the current frame not yet finished
    logic [COUNT_WIDTH-1:0] baud_count; // clocks elapsed in the current bit

    always_comb in_ready = (bits_left == 4'd0);
    always_comb tx       = shift[0];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            shift      <= '1; // line idles high
            bits_left  <= 4'd0;
            baud_count <= '0;
        end else if (bits_left == 4'd0) begin
            if (in_valid) begin
                shift      <= {1'b1, in_data, 1'b0};
                bits_left  <= 4'd10;
                baud_count <= '0;
            end
        end else if (baud_count == LAST_COUNT) begin
            shift      <= {1'b1, shift[9:1]};
            bits_left  <= bits_left - 4'd1;
            baud_count <= '0;
        end else begin
            baud_count <= baud_count + 1'b1;
        end
    end

endmodule

`default_nettype wire
