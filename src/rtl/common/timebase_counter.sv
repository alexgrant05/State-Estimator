`timescale 1ns/1ps
`default_nettype none

module timebase_counter #(
    parameter int unsigned WIDTH = 64
) (
    input  logic                 clk,
    input  logic                 rst_n,
    output logic [WIDTH-1:0]     time_ticks
);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            time_ticks <= '0;
        end else begin
            time_ticks <= time_ticks + 1'b1;
        end
    end

endmodule

`default_nettype wire
