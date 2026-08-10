`timescale 1ns/1ps
`default_nettype none

module tb_async_event_capture;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic async_event = 1'b0;
    logic [63:0] time_ticks;
    logic event_valid;
    logic event_ready = 1'b0;
    logic [63:0] event_timestamp;
    logic overflow_sticky;
    logic [31:0] overflow_count;
    logic [63:0] held_timestamp;

    always #5 clk <= ~clk;

    timebase_counter timebase (
        .clk,
        .rst_n,
        .time_ticks
    );

    async_event_capture dut (
        .clk,
        .rst_n,
        .async_event,
        .time_ticks,
        .event_valid,
        .event_ready,
        .event_timestamp,
        .overflow_sticky,
        .overflow_count
    );

    task automatic pulse_event;
        #2;
        async_event = 1'b1;
        #12;
        async_event = 1'b0;
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        pulse_event();
        wait (event_valid);
        held_timestamp = event_timestamp;
        repeat (2) begin
            @(posedge clk);
            #1;
            assert (event_valid) else $fatal(1, "event was not held under backpressure");
            assert (event_timestamp == held_timestamp) else $fatal(1, "timestamp changed while held");
        end

        pulse_event();
        wait (overflow_count == 32'd1);
        assert (overflow_sticky) else $fatal(1, "overflow sticky flag was not set");

        @(negedge clk);
        event_ready = 1'b1;
        @(posedge clk);
        @(negedge clk);
        event_ready = 1'b0;
        #1;
        assert (!event_valid) else $fatal(1, "accepted event remained valid");

        pulse_event();
        wait (event_valid);
        assert (event_timestamp > held_timestamp) else $fatal(1, "new timestamp did not advance");

        $display("PASS tb_async_event_capture");
        $finish;
    end
endmodule

`default_nettype wire
