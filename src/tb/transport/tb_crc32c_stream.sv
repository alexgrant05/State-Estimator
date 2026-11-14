`timescale 1ns/1ps
`default_nettype none

module tb_crc32c_stream;
    logic clk = 1'b0;
    logic resetn = 1'b0;
    logic start = 1'b0;
    logic data_valid = 1'b0;
    logic [7:0] data_byte = 8'b0;
    logic data_last = 1'b0;
    logic [31:0] crc;
    logic crc_valid;

    always #5 clk <= ~clk;

    crc32c_stream dut (.*);

    task automatic send_byte(input logic [7:0] value, input logic first, input logic last);
        begin
            @(negedge clk);
            start = first;
            data_valid = 1'b1;
            data_byte = value;
            data_last = last;
            @(negedge clk);
            start = 1'b0;
            data_valid = 1'b0;
            data_last = 1'b0;
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        resetn = 1'b1;
        send_byte("1", 1'b1, 1'b0);
        send_byte("2", 1'b0, 1'b0);
        send_byte("3", 1'b0, 1'b0);
        send_byte("4", 1'b0, 1'b0);
        send_byte("5", 1'b0, 1'b0);
        send_byte("6", 1'b0, 1'b0);
        send_byte("7", 1'b0, 1'b0);
        send_byte("8", 1'b0, 1'b0);
        send_byte("9", 1'b0, 1'b1);
        if (!crc_valid || crc !== 32'hE306_9283) $fatal(1, "CRC32C vector mismatch: %08x", crc);
        $display("PASS tb_crc32c_stream");
        $finish;
    end
endmodule

`default_nettype wire
