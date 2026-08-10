`timescale 1ns/1ps
`default_nettype none

module async_event_capture (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          async_event,
    input  logic [63:0]   time_ticks,
    output logic          event_valid,
    input  logic          event_ready,
    output logic [63:0]   event_timestamp,
    output logic          overflow_sticky,
    output logic [31:0]   overflow_count
);

    logic event_meta;
    logic event_sync;
    logic event_sync_d;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            event_meta      <= 1'b0;
            event_sync      <= 1'b0;
            event_sync_d    <= 1'b0;
            event_valid     <= 1'b0;
            event_timestamp <= '0;
            overflow_sticky <= 1'b0;
            overflow_count  <= '0;
        end else begin
            event_meta   <= async_event;
            event_sync   <= event_meta;
            event_sync_d <= event_sync;

            if (event_valid && event_ready) begin
                event_valid <= 1'b0;
            end

            if (event_sync && !event_sync_d) begin
                if (!event_valid || event_ready) begin
                    event_timestamp <= time_ticks;
                    event_valid     <= 1'b1;
                end else begin
                    overflow_sticky <= 1'b1;
                    if (overflow_count != 32'hffff_ffff) begin
                        overflow_count <= overflow_count + 1'b1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
