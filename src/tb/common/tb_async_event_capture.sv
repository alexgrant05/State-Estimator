`timescale 1ns/1ps
`default_nettype none

module tb_async_event_capture;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic async_event = 1'b0;
    logic [63:0] time_ticks;
    logic event_ready = 1'b0;
    logic clear_status = 1'b0;
    logic event_valid;
    logic [63:0] event_timestamp;
    logic overflow_pulse;
    logic overflow_sticky;
    logic [31:0] overflow_count;

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
        .event_ready,
        .clear_status,
        .event_valid,
        .event_timestamp,
        .overflow_pulse,
        .overflow_sticky,
        .overflow_count
    );

`ifdef TRACE
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_async_event_capture);
    end
`endif

    task automatic wait_for_valid;
        for (int unsigned cycle = 0; cycle < 12; cycle++) begin
            @(posedge clk);
            #1;
            if (event_valid) return;
        end
        $fatal(1, "timed out waiting for captured event");
    endtask

    task automatic wait_for_overflow;
        for (int unsigned cycle = 0; cycle < 12; cycle++) begin
            @(posedge clk);
            #1;
            if (overflow_pulse) return;
        end
        $fatal(1, "timed out waiting for overflow");
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        #2 async_event = 1'b1;
        wait_for_valid();
        if (event_timestamp == 0 || event_timestamp > time_ticks) begin
            $fatal(1, "invalid captured timestamp");
        end

        @(negedge clk);
        async_event = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        async_event = 1'b1;
        wait_for_overflow();
        if (!overflow_sticky || overflow_count !== 32'd1) begin
            $fatal(1, "overflow status mismatch");
        end

        @(negedge clk);
        event_ready = 1'b1;
        @(posedge clk);
        #1;
        if (event_valid) $fatal(1, "ready handshake did not consume event");

        @(negedge clk);
        event_ready = 1'b0;
        clear_status = 1'b1;
        @(posedge clk);
        #1;
        if (overflow_sticky || overflow_count != 0) $fatal(1, "clear_status failed");

        $display("PASS tb_async_event_capture");
        $finish;
    end
endmodule

`default_nettype wire
