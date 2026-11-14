`timescale 1ns/1ps
`default_nettype none

module tb_led_blinker;
    localparam int unsigned HALF_PERIOD_CYCLES = 4;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic led;

    always #5 clk <= ~clk;

    led_blinker #(
        .CLOCK_HZ(8),
        .BLINK_HZ(1),
        .HALF_PERIOD_CYCLES(HALF_PERIOD_CYCLES)
    ) dut (
        .clk,
        .rst_n,
        .led
    );

    task automatic check_led(input logic expected, input string message);
        #1;
        assert (!$isunknown(led)) else $fatal(1, "LED output is unknown: %s", message);
        assert (led == expected) else $fatal(1, "%s", message);
    endtask

    initial begin
        repeat (3) begin
            @(posedge clk);
            check_led(1'b0, "LED was not off during reset");
        end

        @(negedge clk);
        rst_n = 1'b1;

        repeat (HALF_PERIOD_CYCLES - 1) begin
            @(posedge clk);
            check_led(1'b0, "LED toggled before the first half-period elapsed");
        end
        @(posedge clk);
        check_led(1'b1, "LED did not toggle at the first half-period");

        repeat (HALF_PERIOD_CYCLES - 1) begin
            @(posedge clk);
            check_led(1'b1, "LED toggled before the second half-period elapsed");
        end
        @(posedge clk);
        check_led(1'b0, "LED did not toggle at the second half-period");

        repeat (2) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b0;
        @(posedge clk);
        check_led(1'b0, "reset did not clear the LED output");

        @(negedge clk);
        rst_n = 1'b1;
        repeat (HALF_PERIOD_CYCLES - 1) begin
            @(posedge clk);
            check_led(1'b0, "reset did not clear the half-period counter");
        end
        @(posedge clk);
        check_led(1'b1, "LED did not restart with a complete half-period");

        $display("PASS tb_led_blinker");
        $finish;
    end
endmodule

`default_nettype wire
