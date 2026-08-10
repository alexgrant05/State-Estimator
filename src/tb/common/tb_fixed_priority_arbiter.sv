`timescale 1ns/1ps
`default_nettype none

module tb_fixed_priority_arbiter;
    logic [3:0] request;
    logic [3:0] grant;
    logic grant_valid;
    logic [1:0] grant_index;

    fixed_priority_arbiter #(.REQUESTERS(4)) dut (
        .request,
        .grant,
        .grant_valid,
        .grant_index
    );

`ifdef TRACE
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_fixed_priority_arbiter);
    end
`endif

    task automatic check(
        input logic [3:0] requested,
        input logic expected_valid,
        input logic [3:0] expected_grant,
        input logic [1:0] expected_index
    );
        request = requested;
        #1;
        if (grant_valid !== expected_valid) $fatal(1, "grant_valid mismatch");
        if (grant !== expected_grant) $fatal(1, "grant mismatch: %b", grant);
        if (grant_index !== expected_index) $fatal(1, "grant_index mismatch: %0d", grant_index);
    endtask

    initial begin
        check(4'b0000, 1'b0, 4'b0000, 2'd0);
        check(4'b1000, 1'b1, 4'b1000, 2'd3);
        check(4'b1010, 1'b1, 4'b0010, 2'd1);
        check(4'b1111, 1'b1, 4'b0001, 2'd0);
        check(4'b0100, 1'b1, 4'b0100, 2'd2);
        $display("PASS tb_fixed_priority_arbiter");
        $finish;
    end
endmodule

`default_nettype wire
