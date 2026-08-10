`timescale 1ns/1ps
`default_nettype none

module tb_sync_fifo;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic [7:0] input_data = '0;
    logic input_valid = 1'b0;
    logic input_ready;
    logic [7:0] output_data;
    logic output_valid;
    logic output_ready = 1'b0;
    logic [2:0] level;
    logic overflow_sticky;
    logic underflow_sticky;

    always #5 clk <= ~clk;

    sync_fifo #(
        .DATA_WIDTH(8),
        .DEPTH(4)
    ) dut (
        .clk,
        .rst_n,
        .input_data,
        .input_valid,
        .input_ready,
        .output_data,
        .output_valid,
        .output_ready,
        .level,
        .overflow_sticky,
        .underflow_sticky
    );

    task automatic push_byte(input logic [7:0] value);
        @(negedge clk);
        input_data = value;
        input_valid = 1'b1;
        #1;
        assert (input_ready) else $fatal(1, "push was not accepted");
        @(posedge clk);
        @(negedge clk);
        input_valid = 1'b0;
    endtask

    task automatic pop_byte(input logic [7:0] expected);
        @(negedge clk);
        output_ready = 1'b1;
        #1;
        assert (output_valid) else $fatal(1, "expected valid output");
        assert (output_data == expected) else $fatal(1, "FIFO ordering failed");
        @(posedge clk);
        @(negedge clk);
        output_ready = 1'b0;
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        push_byte(8'h11);
        push_byte(8'h22);
        push_byte(8'h33);
        assert (level == 3'd3) else $fatal(1, "level after pushes was incorrect");
        pop_byte(8'h11);
        pop_byte(8'h22);
        pop_byte(8'h33);
        assert (level == 3'd0) else $fatal(1, "FIFO did not empty");

        @(negedge clk);
        output_ready = 1'b1;
        @(posedge clk);
        #1;
        assert (underflow_sticky) else $fatal(1, "underflow was not reported");
        @(negedge clk);
        output_ready = 1'b0;

        push_byte(8'h01);
        push_byte(8'h02);
        push_byte(8'h03);
        push_byte(8'h04);
        @(negedge clk);
        input_data = 8'h05;
        input_valid = 1'b1;
        @(posedge clk);
        #1;
        assert (overflow_sticky) else $fatal(1, "overflow was not reported");
        @(negedge clk);
        input_valid = 1'b0;

        output_ready = 1'b1;
        input_data = 8'h05;
        input_valid = 1'b1;
        #1;
        assert (input_ready) else $fatal(1, "full simultaneous pop/push was blocked");
        @(posedge clk);
        @(negedge clk);
        output_ready = 1'b0;
        input_valid = 1'b0;

        pop_byte(8'h02);
        pop_byte(8'h03);
        pop_byte(8'h04);
        pop_byte(8'h05);

        $display("PASS tb_sync_fifo");
        $finish;
    end
endmodule

`default_nettype wire
