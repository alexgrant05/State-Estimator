`timescale 1ns/1ps
`default_nettype none

module led_blinker #(
    parameter int unsigned CLOCK_HZ = 100_000_000,
    parameter int unsigned BLINK_HZ = 1,
    parameter int unsigned HALF_PERIOD_CYCLES = CLOCK_HZ / (2 * BLINK_HZ),
    parameter int unsigned COUNTER_WIDTH = $clog2(HALF_PERIOD_CYCLES)
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK",
       X_INTERFACE_PARAMETER = "FREQ_HZ 99999001, ASSOCIATED_RESET rst_n" *)
    input  wire  clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST",
       X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire  rst_n,
    output logic led = 1'b0
);

    // The JTAG bring-up design ties rst_n high. Both state registers must
    // therefore have configuration-time initial values, even before clk runs.
    logic [COUNTER_WIDTH-1:0] counter = '0;

    initial begin
        if (CLOCK_HZ == 0) begin
            $error("CLOCK_HZ must be greater than zero");
        end
        if (BLINK_HZ == 0) begin
            $error("BLINK_HZ must be greater than zero");
        end
        if (HALF_PERIOD_CYCLES < 2) begin
            $error("HALF_PERIOD_CYCLES must be at least two");
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            counter <= '0;
            led <= 1'b0;
        end else if (counter == COUNTER_WIDTH'(HALF_PERIOD_CYCLES - 1)) begin
            counter <= '0;
            led <= ~led;
        end else begin
            counter <= counter + 1'b1;
        end
    end

endmodule

`default_nettype wire
