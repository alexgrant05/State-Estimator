`timescale 1ns/1ps
`default_nettype none

module async_event_capture #(
    parameter int unsigned TIMESTAMP_WIDTH = 64,
    parameter int unsigned SYNC_STAGES = 2,
    parameter int unsigned OVERFLOW_COUNT_WIDTH = 32
) (
    input  logic                              clk,
    input  logic                              rst_n,
    input  logic                              async_event,
    input  logic [TIMESTAMP_WIDTH-1:0]        time_ticks,
    input  logic                              event_ready,
    input  logic                              clear_status,
    output logic                              event_valid,
    output logic [TIMESTAMP_WIDTH-1:0]        event_timestamp,
    output logic                              overflow_pulse,
    output logic                              overflow_sticky,
    output logic [OVERFLOW_COUNT_WIDTH-1:0]   overflow_count
);

    (* ASYNC_REG = "TRUE" *) logic [SYNC_STAGES-1:0] synchronizer;
    logic synchronized_delayed;
    logic rising_edge;

    initial begin
        if (SYNC_STAGES < 2) begin
            $error("async_event_capture requires SYNC_STAGES >= 2");
        end
    end

    assign rising_edge = synchronizer[SYNC_STAGES-1] && !synchronized_delayed;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            synchronizer       <= '0;
            synchronized_delayed <= 1'b0;
        end else begin
            synchronizer[0] <= async_event;
            for (int unsigned stage = 1; stage < SYNC_STAGES; stage++) begin
                synchronizer[stage] <= synchronizer[stage-1];
            end
            synchronized_delayed <= synchronizer[SYNC_STAGES-1];
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            event_valid     <= 1'b0;
            event_timestamp <= '0;
            overflow_pulse  <= 1'b0;
            overflow_sticky <= 1'b0;
            overflow_count  <= '0;
        end else begin
            overflow_pulse <= 1'b0;

            if (clear_status) begin
                overflow_sticky <= 1'b0;
                overflow_count  <= '0;
            end

            if (event_valid && event_ready) begin
                event_valid <= 1'b0;
            end

            if (rising_edge) begin
                if (!event_valid || event_ready) begin
                    event_valid     <= 1'b1;
                    event_timestamp <= time_ticks;
                end else begin
                    overflow_pulse  <= 1'b1;
                    overflow_sticky <= 1'b1;
                    if (clear_status) begin
                        overflow_count <= {{(OVERFLOW_COUNT_WIDTH-1){1'b0}}, 1'b1};
                    end else if (!(&overflow_count)) begin
                        overflow_count <= overflow_count + 1'b1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
