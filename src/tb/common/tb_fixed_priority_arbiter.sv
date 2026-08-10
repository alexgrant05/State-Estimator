`timescale 1ns/1ps
`default_nettype none

module tb_fixed_priority_arbiter;
    logic [3:0] request;
    logic [3:0] grant;

    fixed_priority_arbiter #(
        .REQUESTERS(4)
    ) dut (
        .request,
        .grant
    );

    initial begin
        request = 4'b0000;
        #1;
        assert (grant == 4'b0000) else $fatal(1, "idle grant was not zero");

        request = 4'b1000;
        #1;
        assert (grant == 4'b1000) else $fatal(1, "single request failed");

        request = 4'b1110;
        #1;
        assert (grant == 4'b0010) else $fatal(1, "priority order was incorrect");

        request = 4'b1111;
        #1;
        assert (grant == 4'b0001) else $fatal(1, "requester zero was not highest priority");

        $display("PASS tb_fixed_priority_arbiter");
        $finish;
    end
endmodule

`default_nettype wire
