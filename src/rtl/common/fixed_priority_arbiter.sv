`timescale 1ns/1ps
`default_nettype none

module fixed_priority_arbiter #(
    parameter int unsigned REQUESTERS = 4
) (
    input  logic [REQUESTERS-1:0] request,
    output logic [REQUESTERS-1:0] grant
);

    integer index;
    logic found;

    always_comb begin
        grant = '0;
        found = 1'b0;
        for (index = 0; index < REQUESTERS; index = index + 1) begin
            if (request[index] && !found) begin
                grant[index] = 1'b1;
                found = 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
