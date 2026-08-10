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

`ifdef TRACE
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_timebase_counter);
    end
`endif

    initial begin
        repeat (3) @(posedge clk);
        #1;
        if (time_ticks !== 64'd0) $fatal(1, "counter changed during reset");

        @(negedge clk);
        rst_n = 1'b1;
        for (int unsigned expected = 1; expected <= 20; expected++) begin
            @(posedge clk);
            #1;
            if (time_ticks !== 64'(expected)) begin
                $fatal(1, "expected tick %0d, received %0d", expected, time_ticks);
            end
        end

        @(negedge clk);
        rst_n = 1'b0;
        @(posedge clk);
        #1;
        if (time_ticks !== 64'd0) $fatal(1, "reset did not clear counter");

        $display("PASS tb_timebase_counter");
        $finish;
    end
endmodule

`default_nettype wire
