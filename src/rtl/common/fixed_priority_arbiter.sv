`timescale 1ns/1ps
`default_nettype none

module fixed_priority_arbiter #(
    parameter int unsigned REQUESTERS = 2,
    localparam int unsigned INDEX_WIDTH = (REQUESTERS <= 2) ? 1 : $clog2(REQUESTERS)
) (
    input  logic [REQUESTERS-1:0]      request,
    output logic [REQUESTERS-1:0]      grant,
    output logic                       grant_valid,
    output logic [INDEX_WIDTH-1:0]     grant_index
);

    initial begin
        if (REQUESTERS < 1) begin
            $error("fixed_priority_arbiter requires REQUESTERS >= 1");
        end
    end

    always_comb begin
        grant       = '0;
        grant_valid = 1'b0;
        grant_index = '0;
        for (int unsigned requester = 0; requester < REQUESTERS; requester++) begin
            if (request[requester] && !grant_valid) begin
                grant[requester] = 1'b1;
                grant_valid      = 1'b1;
                grant_index      = INDEX_WIDTH'(requester);
            end
        end
    end

endmodule

`default_nettype wire
