`timescale 1ns/1ps
`default_nettype none

// raw_byte_tap
//
// Bring-up debug path: squeezes the raw SPI byte stream from the receiver
// (~118 kB/s at 1 MHz SCLK, nearly all of it 0xFF idle filler) into a
// 115200-baud UART (~11.5 kB/s) by dropping the filler and buffering bursts.
//
// A plain "drop every 0xFF" filter would corrupt messages, because 0xFF is
// also a legal payload byte (the u-blox integration manual, SPI section, says
// the host should ignore 0xFF bytes "which are not part of a message"). A
// byte stream alone can't say which case it is, so this uses a heuristic: a
// 0xFF is only dropped once FF_KEEP of them have arrived in a row. Any payload
// with a shorter run of 0xFF passes through intact; the cost is FF_KEEP filler
// bytes on the wire after each burst, which the host discards.
//
// If the UART can't keep up, sync_fifo drops bytes and overflow_sticky is set.
// Anything decoded from the stream after that point can't be trusted.
//
// Inputs:  clk, rst_n, rx_byte, rx_byte_valid, out_ready
// Outputs: out_data, out_valid, overflow_sticky
module raw_byte_tap #(
    parameter int unsigned FIFO_DEPTH = 1024,
    parameter int unsigned FF_KEEP    = 32
) (
    input  logic       clk,
    input  logic       rst_n,

    input  logic [7:0] rx_byte,
    input  logic       rx_byte_valid,

    output logic [7:0] out_data,
    output logic       out_valid,
    input  logic       out_ready,

    output logic       overflow_sticky
);

    localparam int unsigned RUN_WIDTH = $clog2(FF_KEEP + 1);
    localparam logic [RUN_WIDTH-1:0] RUN_FULL = RUN_WIDTH'(FF_KEEP);

    // Consecutive 0xFF bytes seen, saturating at FF_KEEP. Starts full so the
    // filler before the first real byte is dropped too.
    logic [RUN_WIDTH-1:0] ff_run;
    logic                 is_filler;
    logic                 keep_byte;

    always_comb is_filler = (rx_byte == 8'hFF);
    always_comb keep_byte = !is_filler || (ff_run != RUN_FULL);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ff_run <= RUN_FULL;
        end else if (rx_byte_valid) begin
            if (!is_filler)              ff_run <= '0;
            else if (ff_run != RUN_FULL) ff_run <= ff_run + 1'b1;
        end
    end

    /* verilator lint_off UNUSEDSIGNAL */
    logic                          fifo_input_ready; // overflow is reported via overflow_sticky
    logic [$clog2(FIFO_DEPTH+1)-1:0] fifo_level;     // not needed by this debug path
    logic                          fifo_underflow_sticky_unused;
    /* verilator lint_on UNUSEDSIGNAL */

    sync_fifo #(
        .DATA_WIDTH (8),
        .DEPTH      (FIFO_DEPTH)
    ) u_fifo (
        .clk              (clk),
        .rst_n            (rst_n),
        .input_data       (rx_byte),
        .input_valid      (rx_byte_valid && keep_byte),
        .input_ready      (fifo_input_ready),
        .output_data      (out_data),
        .output_valid     (out_valid),
        .output_ready     (out_ready),
        .level            (fifo_level),
        .overflow_sticky  (overflow_sticky),
        .underflow_sticky (fifo_underflow_sticky_unused)
    );

endmodule

`default_nettype wire
