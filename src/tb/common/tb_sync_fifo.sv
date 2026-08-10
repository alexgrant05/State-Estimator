`timescale 1ns/1ps
`default_nettype none

module tb_sync_fifo;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic clear_status = 1'b0;
    logic [7:0] input_data = '0;
    logic input_valid = 1'b0;
    logic input_ready;
    logic [7:0] output_data;
    logic output_valid;
    logic output_ready = 1'b0;
    logic [2:0] level;
    logic overflow_pulse;
    logic underflow_pulse;
    logic overflow_sticky;
    logic underflow_sticky;

    always #5 clk <= ~clk;

    sync_fifo #(
        .DATA_WIDTH(8),
        .DEPTH(4)
    ) dut (
        .clk,
        .rst_n,
        .clear_status,
        .input_data,
        .input_valid,
        .input_ready,
        .output_data,
        .output_valid,
        .output_ready,
        .level,
        .overflow_pulse,
        .underflow_pulse,
        .overflow_sticky,
        .underflow_sticky
    );

`ifdef TRACE
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_sync_fifo);
    end
`endif

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        for (int unsigned value = 0; value < 4; value++) begin
            input_data = 8'(value);
            input_valid = 1'b1;
            @(posedge clk);
            #1;
            if (level !== 3'(value + 1)) $fatal(1, "fill level mismatch");
            @(negedge clk);
        end
        input_valid = 1'b0;
        #1;
        if (input_ready !== 1'b0) $fatal(1, "full FIFO reported ready");

        input_data = 8'hff;
        input_valid = 1'b1;
        @(posedge clk);
        #1;
        if (!overflow_pulse || !overflow_sticky) $fatal(1, "overflow was not reported");

        @(negedge clk);
        input_data = 8'd99;
        output_ready = 1'b1;
        #1;
        if (!input_ready || output_data !== 8'd0) $fatal(1, "full pop/push setup failed");
        @(posedge clk);
        #1;
        if (level !== 3'd4) $fatal(1, "simultaneous pop/push changed level");

        @(negedge clk);
        input_valid = 1'b0;
        for (int unsigned expected = 1; expected <= 3; expected++) begin
            if (!output_valid || output_data !== 8'(expected)) $fatal(1, "FIFO ordering failed");
            @(posedge clk);
            @(negedge clk);
        end
        if (!output_valid || output_data !== 8'd99) $fatal(1, "replacement item missing");
        @(posedge clk);
        #1;
        if (level !== 0 || output_valid) $fatal(1, "FIFO did not empty");

        @(negedge clk);
        output_ready = 1'b1;
        @(posedge clk);
        #1;
        if (!underflow_pulse || !underflow_sticky) $fatal(1, "underflow was not reported");

        @(negedge clk);
        output_ready = 1'b0;
        clear_status = 1'b1;
        @(posedge clk);
        #1;
        if (overflow_sticky || underflow_sticky) $fatal(1, "clear_status failed");

        $display("PASS tb_sync_fifo");
        $finish;
    end
endmodule

`default_nettype wire
