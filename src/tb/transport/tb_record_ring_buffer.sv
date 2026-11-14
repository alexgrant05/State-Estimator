`timescale 1ns/1ps
`default_nettype none

module tb_record_ring_buffer;
    logic clk = 1'b0;
    logic resetn = 1'b0;
    logic flush = 1'b0;
    logic write_start = 1'b0;
    logic write_valid = 1'b0;
    logic [7:0] write_data = 8'b0;
    logic write_ready;
    logic write_commit = 1'b0;
    logic write_abort = 1'b0;
    logic write_active;
    logic write_overflow;
    logic read_valid;
    logic [7:0] read_data;
    logic read_ready = 1'b0;
    logic [4:0] used_bytes;

    always #5 clk <= ~clk;

    record_ring_buffer #(.ADDR_WIDTH(4)) dut (.*);

    task automatic pulse(input integer selection);
        begin
            @(negedge clk);
            write_start = selection == 0;
            write_commit = selection == 1;
            write_abort = selection == 2;
            @(negedge clk);
            write_start = 1'b0;
            write_commit = 1'b0;
            write_abort = 1'b0;
        end
    endtask

    task automatic put(input logic [7:0] value);
        begin
            @(negedge clk);
            if (!write_ready) $fatal(1, "writer unexpectedly not ready");
            write_valid = 1'b1;
            write_data = value;
            @(negedge clk);
            write_valid = 1'b0;
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        resetn = 1'b1;
        pulse(0);
        if (!write_active) $fatal(1, "write transaction did not start");
        put(8'h11);
        put(8'h22);
        if (read_valid || used_bytes != 0) $fatal(1, "uncommitted bytes became visible");
        pulse(1);
        if (!read_valid || used_bytes != 2 || read_data != 8'h11) $fatal(1, "commit was not atomic");
        @(negedge clk); read_ready = 1'b1;
        @(negedge clk); read_ready = 1'b0;
        if (read_data != 8'h22 || used_bytes != 1) $fatal(1, "read pointer mismatch");
        pulse(0);
        put(8'h33);
        pulse(2);
        if (used_bytes != 1) $fatal(1, "abort changed committed data");
        if (write_overflow) $fatal(1, "unexpected overflow");
        $display("PASS tb_record_ring_buffer");
        $finish;
    end
endmodule

`default_nettype wire
