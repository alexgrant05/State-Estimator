`timescale 1ns/1ps
`default_nettype none

// gnss_msg_router
//
// Splits the decoded UBX payload stream into the real-time path
// (NAV-PVT / NAV-COV, toward the ESKF via nav_solution_packer) and the
// bulk/PPK path (RXM-RAWX / RXM-SFRBX, toward bulk_log_framer and
// eventually DDR/DMA). This is the SCC-local instance of the two-path
// split; it is a peer of the top-level architecture's main router, not
// a client of it -- the bulk path never enters the main router/FIFO.
//
// Inputs:  clk, rst_n, payload_byte/valid/last, msg_class, msg_id,
//          msg_length, frame_sof, frame_done, checksum_fail
// Outputs: nav_byte/valid/last + nav_msg_id/frame_done/checksum_fail,
//          bulk_byte/valid/last + bulk_msg_id/frame_done/checksum_fail
module gnss_msg_router (
    input  logic         clk,
    input  logic         rst_n,

    input  logic [7:0]   payload_byte,
    input  logic         payload_byte_valid,
    input  logic         payload_byte_last,
    input  logic [7:0]   msg_class,
    input  logic [7:0]   msg_id,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [15:0]  msg_length, // part of ubx_frame_sync's metadata bundle; not needed for
                                     // the class/id routing decision, so not consumed here
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic         frame_sof,
    input  logic         frame_done,
    input  logic         checksum_fail,

    output logic [7:0]   nav_byte,
    output logic         nav_byte_valid,
    output logic         nav_byte_last,
    output logic [7:0]   nav_msg_id,
    output logic         nav_frame_done,
    output logic         nav_checksum_fail,

    output logic [7:0]   bulk_byte,
    output logic         bulk_byte_valid,
    output logic         bulk_byte_last,
    output logic [7:0]   bulk_msg_id,
    output logic         bulk_frame_done,
    output logic         bulk_checksum_fail
);

    // UBX class/id values -- TODO: confirm against the u-blox Interface
    // Description before this is trusted for flight use.
    localparam logic [7:0] CLASS_NAV    = 8'h01;
    localparam logic [7:0] ID_NAV_PVT   = 8'h07;
    localparam logic [7:0] ID_NAV_COV   = 8'h36;
    localparam logic [7:0] CLASS_RXM    = 8'h02;
    localparam logic [7:0] ID_RXM_RAWX  = 8'h15;
    localparam logic [7:0] ID_RXM_SFRBX = 8'h13;

    logic route_nav, route_bulk;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            route_nav  <= 1'b0;
            route_bulk <= 1'b0;
        end else if (frame_sof) begin
            route_nav  <= (msg_class == CLASS_NAV) && ((msg_id == ID_NAV_PVT) || (msg_id == ID_NAV_COV));
            route_bulk <= (msg_class == CLASS_RXM) && ((msg_id == ID_RXM_RAWX) || (msg_id == ID_RXM_SFRBX));
        end else if (frame_done) begin
            route_nav  <= 1'b0;
            route_bulk <= 1'b0;
        end
    end

    always_comb begin
        nav_byte          = payload_byte;
        nav_byte_valid    = payload_byte_valid && route_nav;
        nav_byte_last     = payload_byte_last  && route_nav;
        nav_msg_id        = msg_id;
        nav_frame_done    = frame_done && route_nav;
        nav_checksum_fail = checksum_fail && route_nav;

        bulk_byte          = payload_byte;
        bulk_byte_valid    = payload_byte_valid && route_bulk;
        bulk_byte_last     = payload_byte_last  && route_bulk;
        bulk_msg_id        = msg_id;
        bulk_frame_done    = frame_done && route_bulk;
        bulk_checksum_fail = checksum_fail && route_bulk;
    end

endmodule

`default_nettype wire
