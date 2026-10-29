`timescale 1ns/1ps
`default_nettype none

// bulk_log_framer
//
// Frames RXM-RAWX / RXM-SFRBX payloads (already filtered by
// gnss_msg_router) for the bulk/PPK path: a small header record per
// frame (held-valid, overflow-tracked) paired 1:1 with that frame's raw
// payload bytes on an elastic FIFO (the existing sync_fifo common
// module). This is the milestone-2 "bulk/PPK path" -- a peer of the
// main top-level router, wired straight toward DDR/DMA, never through it.
//
// A header is emitted for every frame, pass or fail (header_checksum_fail
// records the outcome) -- this is deliberate: checksum status isn't known
// until frame_done, by which point all of that frame's payload bytes have
// already been pushed to the FIFO. If a failed frame silently produced no
// header, header:payload pairing would desync for every following frame.
// The post-flight PPK tool discards flagged frames using the header, not
// by re-parsing the byte stream.
//
// log_fifo_overflow_sticky surfaces sync_fifo's own overflow_sticky: if
// the downstream DMA consumer stalls long enough to fill FIFO_DEPTH
// bytes, sync_fifo silently drops further writes. When that happens
// mid-frame, header_byte_count for that frame no longer matches the
// bytes actually retrievable from log_byte -- this flag is the only
// signal that occurred. Widening FIFO_DEPTH (sized to the worst-case
// stall) or adding real backpressure-aware framing are the two ways to
// close this gap; flagging it is the stopgap for now.
//
// Inputs:  clk, rst_n, bulk_byte/valid/last, bulk_msg_id, bulk_frame_done,
//          bulk_checksum_fail, time_ticks, header_ready, log_byte_ready
// Outputs: header_valid + header_* fields, log_byte/valid,
//          log_fifo_level, log_fifo_overflow_sticky
module bulk_log_framer #(
    parameter int FIFO_DEPTH = 512
) (
    input  logic         clk,
    input  logic         rst_n,

    input  logic [7:0]   bulk_byte,
    input  logic         bulk_byte_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic         bulk_byte_last, // kept for byte-stream interface consistency;
                                         // frame end is tracked via bulk_frame_done instead
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [7:0]   bulk_msg_id,
    input  logic         bulk_frame_done,
    input  logic         bulk_checksum_fail,

    input  logic [63:0]  time_ticks,

    output logic         header_valid,
    input  logic         header_ready,
    output logic [63:0]  header_time_ticks,
    output logic [31:0]  header_seq_num,
    output logic [7:0]   header_msg_id,
    output logic [15:0]  header_byte_count,
    output logic         header_checksum_fail,
    output logic         header_overflow_sticky,
    output logic [31:0]  header_overflow_count,

    output logic [7:0]   log_byte,
    output logic         log_byte_valid,
    input  logic         log_byte_ready,
    output logic [$clog2(FIFO_DEPTH+1)-1:0] log_fifo_level,
    output logic         log_fifo_overflow_sticky
);

    localparam int FIFO_DATA_WIDTH = 8;

    /* verilator lint_off UNUSEDSIGNAL */
    logic fifo_wr_ready; // sync_fifo's own input_ready output, captured here but not yet
                         // acted on -- see the backpressure note above
    /* verilator lint_on UNUSEDSIGNAL */
    logic fifo_underflow_sticky_unused;

    sync_fifo #(
        .DATA_WIDTH (FIFO_DATA_WIDTH),
        .DEPTH      (FIFO_DEPTH)
    ) u_log_fifo (
        .clk              (clk),
        .rst_n            (rst_n),
        .input_data       (bulk_byte),
        .input_valid      (bulk_byte_valid),
        .input_ready      (fifo_wr_ready),
        .output_data      (log_byte),
        .output_valid     (log_byte_valid),
        .output_ready     (log_byte_ready),
        .level            (log_fifo_level),
        .overflow_sticky  (log_fifo_overflow_sticky),
        .underflow_sticky (fifo_underflow_sticky_unused)
    );

    logic [15:0] byte_count;
    logic [31:0] seq_num;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            byte_count             <= 16'h0;
            seq_num                <= 32'h0;
            header_valid           <= 1'b0;
            header_time_ticks      <= 64'h0;
            header_seq_num         <= 32'h0;
            header_msg_id          <= 8'h0;
            header_byte_count      <= 16'h0;
            header_checksum_fail   <= 1'b0;
            header_overflow_sticky <= 1'b0;
            header_overflow_count  <= 32'h0;
        end else begin
            if (header_valid && header_ready) header_valid <= 1'b0;

            if (bulk_byte_valid) byte_count <= byte_count + 16'h1;

            if (bulk_frame_done) begin
                byte_count <= 16'h0;
                if (!header_valid || header_ready) begin
                    header_time_ticks    <= time_ticks;
                    header_seq_num       <= seq_num;
                    header_msg_id        <= bulk_msg_id;
                    header_byte_count    <= byte_count;
                    header_checksum_fail <= bulk_checksum_fail;
                    header_valid         <= 1'b1;
                    seq_num              <= seq_num + 32'h1;
                end else begin
                    header_overflow_sticky <= 1'b1;
                    if (header_overflow_count != 32'hffff_ffff)
                        header_overflow_count <= header_overflow_count + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
