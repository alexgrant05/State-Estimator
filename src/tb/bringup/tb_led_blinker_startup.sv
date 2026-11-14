`timescale 1ns/1ps
`default_nettype none

module tb_led_blinker_startup;
    localparam int unsigned HALF_PERIOD_CYCLES = 4;
    logic clk = 1'b0;
    logic delayed_clk = 1'b0;
    wire led;
    wire delayed_led;

    always #5 clk = ~clk;
    initial begin
        #100;
        forever #5 delayed_clk = ~delayed_clk;
    end

    led_blinker #(.HALF_PERIOD_CYCLES(HALF_PERIOD_CYCLES)) immediate_dut (
        .clk(clk), .rst_n(1'b1), .led(led)
    );
    led_blinker #(.HALF_PERIOD_CYCLES(HALF_PERIOD_CYCLES)) delayed_dut (
        .clk(delayed_clk), .rst_n(1'b1), .led(delayed_led)
    );

    task automatic check_led(input logic actual, input logic expected);
        assert (!$isunknown(actual)) else $fatal(1, "Unknown LED without startup reset");
        assert (actual == expected) else $fatal(1, "Incorrect startup blink phase");
    endtask

    initial begin
        fork
            begin
                #1;
                check_led(led, 1'b0);
                for (int cycle = 1; cycle <= 12; cycle++) begin
                    @(posedge clk);
                    #1;
                    check_led(led, (cycle / HALF_PERIOD_CYCLES) % 2 == 1);
                end
            end
            begin
                #1;
                check_led(delayed_led, 1'b0);
                #90;
                check_led(delayed_led, 1'b0);
                for (int cycle = 1; cycle <= 12; cycle++) begin
                    @(posedge delayed_clk);
                    #1;
                    check_led(delayed_led, (cycle / HALF_PERIOD_CYCLES) % 2 == 1);
                end
            end
        join
        $display("PASS tb_led_blinker_startup");
        $finish;
    end

    initial begin
        #1000;
        $fatal(1, "Startup test timed out");
    end
endmodule

`default_nettype wire
