`timescale 1ns/1ps
`default_nettype none

// nav_solution_packer
//
// Captures one decoded NAV-PVT/NAV-COV payload (already filtered by
// gnss_msg_router) into a byte-array register, and holds it valid until
// consumed downstream -- the async_event_capture idiom (held-valid +
// overflow_sticky/overflow_count) generalized from a single timestamp to
// a multi-byte payload. Frames that fail checksum are dropped silently
// here; gnss_health_monitor's checksum_fail_count is the place that
// failure becomes visible.
//
// This module captures the raw payload bytes rather than parsing
// individual NAV-PVT fields in hardware -- field-level parsing is left
// to PS software, keeping the PL side simple and protocol-version-tolerant.
//
// Inputs:  clk, rst_n, nav_byte/valid/last, nav_msg_id, nav_frame_done,
//          nav_checksum_fail, time_ticks, solution_ready
// Outputs: solution_valid, solution_data, solution_length, solution_msg_id,
//          solution_timestamp, overflow_sticky, overflow_count
module nav_solution_packer #(
    parameter int MAX_PAYLOAD_BYTES = 100  // NAV-PVT payload is 92 bytes; margin for NAV-COV etc.
) (
    input  logic                           clk,
    input  logic                           rst_n,

    input  logic [7:0]                     nav_byte,
    input  logic                           nav_byte_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic                           nav_byte_last, // kept for byte-stream interface consistency;
                                                           // frame end is tracked via nav_frame_done instead
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [7:0]                     nav_msg_id,
    input  logic                           nav_frame_done,
    input  logic                           nav_checksum_fail,

    input  logic [63:0]                    time_ticks,

    output logic                           solution_valid,
    input  logic                           solution_ready,
    output logic [8*MAX_PAYLOAD_BYTES-1:0] solution_data,
    output logic [7:0]                     solution_length,
    output logic [7:0]                     solution_msg_id,
    output logic [63:0]                    solution_timestamp,

    output logic                           overflow_sticky,
    output logic [31:0]                    overflow_count
);

    logic [7:0]                     byte_index;
    logic [8*MAX_PAYLOAD_BYTES-1:0] capture_reg;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            byte_index         <= 8'h0;
            capture_reg        <= '0;
            solution_valid     <= 1'b0;
            solution_data      <= '0;
            solution_length    <= 8'h0;
            solution_msg_id    <= 8'h0;
            solution_timestamp <= 64'h0;
            overflow_sticky    <= 1'b0;
            overflow_count     <= 32'h0;
        end else begin
            if (solution_valid && solution_ready) solution_valid <= 1'b0;

            if (nav_byte_valid) begin
                if (byte_index < 8'(MAX_PAYLOAD_BYTES)) begin // MAX_PAYLOAD_BYTES must stay <= 255 -- byte_index is 8 bits
                    capture_reg[8*byte_index +: 8] <= nav_byte;
                end
                byte_index <= byte_index + 8'h1;
            end

            if (nav_frame_done) begin
                byte_index <= 8'h0;
                if (!nav_checksum_fail) begin
                    if (!solution_valid || solution_ready) begin
                        solution_data      <= capture_reg;
                        solution_length    <= byte_index; // bytes captured this frame
                        solution_msg_id    <= nav_msg_id;
                        solution_timestamp <= time_ticks;
                        solution_valid     <= 1'b1;
                    end else begin
                        overflow_sticky <= 1'b1;
                        if (overflow_count != 32'hffff_ffff) overflow_count <= overflow_count + 1'b1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
