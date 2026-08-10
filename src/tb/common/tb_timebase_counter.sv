`timescale 1ns/1ps
`default_nettype none

module tb_timebase_counter;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic [63:0] time_ticks;

    always #5 clk <= ~clk;

    timebase_counter dut (
        .clk,
        .rst_n,
        .time_ticks
    );

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        @(posedge clk);
        #1;
        assert (time_ticks == 64'd1) else $fatal(1, "first tick was incorrect");
        repeat (9) @(posedge clk);
        #1;
        assert (time_ticks == 64'd10) else $fatal(1, "counter did not advance");

        @(negedge clk);
        rst_n = 1'b0;
        @(posedge clk);
        #1;
        assert (time_ticks == 64'd0) else $fatal(1, "reset did not clear counter");

        $display("PASS tb_timebase_counter");
        $finish;
    end
endmodule

`default_nettype wire
