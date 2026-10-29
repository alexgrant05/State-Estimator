`timescale 1ns/1ps
`default_nettype none

// gnss_scc
//
// Top-level Sensor Communication Controller for the ZED-F9P GNSS
// receiver. Owns the physical pins to the module and wires together
// the SCC's sub-blocks plus two reused common modules:
//   - spi_master (via gnss_spi_engine)      -- shared 4x across sensors
//   - async_event_capture (for PPS/TIMEPULSE) -- generic edge+timestamp capture
//   - sync_fifo (inside bulk_log_framer)    -- elastic byte buffering
//
// This wrapper is also where the SPI-arbitration question lives in the
// diagram: gnss_spi_engine is the block that actually mediates the
// config sequencer's writes against steady-state polling reads on the
// one shared spi_master instance -- that arbitration was previously
// invisible "glue logic" and is now a named block in its own right.
//
// Two output paths leave this SCC, matching the top-level architecture's
// real-time/bulk split: solution_* goes to the existing router/FIFO path
// toward the ESKF; header_*/log_byte* is a peer of that router, wired
// straight toward DDR/DMA for post-flight PPK (milestone-2 transport TBD).
//
// ANT_DETECT / ANT_OFF / ANT_SHORT_N are internal to the RF front end and
// are not host-facing pins -- they are intentionally absent here.
// D_SEL is a board-level interface-select strap, not a dynamic signal,
// so it is also absent from this port list.
//
// Inputs:  clk, rst_n, time_ticks, miso, timepulse, tx_ready_n_pin,
//          rtk_stat_pin, geofence_stat_pin, start_sequence,
//          solution_ready, header_ready, log_byte_ready, pps_ready
// Outputs: sclk, mosi, cs_n, reset_n, safeboot_n, solution_* bundle,
//          nav_overflow_sticky/count, header_*/log_byte* bundle,
//          pps_* bundle, status bundle
module gnss_scc #(
    parameter int SPI_CLOCK_DIVIDE       = 100,
    parameter int MAX_NAV_PAYLOAD_BYTES  = 100
) (
    input  logic         clk,
    input  logic         rst_n,

    input  logic [63:0]  time_ticks,     // from the shared timebase_counter

    // physical pins to the ZED-F9P module
    output logic         sclk,
    output logic         mosi,
    input  logic         miso,
    output logic         cs_n,
    output logic         reset_n,          // GNSS RESET_N (active-low)
    output logic         safeboot_n,       // tie high in normal operation
    input  logic         timepulse,        // PPS-like edge, async to clk
    input  logic         tx_ready_n_pin,
    input  logic         rtk_stat_pin,
    input  logic         geofence_stat_pin,

    input  logic         start_sequence,   // PS-triggered (re)configure request

    // real-time solution path -> existing top-level router/FIFO
    output logic                              solution_valid,
    input  logic                              solution_ready,
    output logic [8*MAX_NAV_PAYLOAD_BYTES-1:0] solution_data,
    output logic [7:0]                        solution_length,
    output logic [7:0]                        solution_msg_id,
    output logic [63:0]                       solution_timestamp,
    output logic                              nav_overflow_sticky,
    output logic [31:0]                       nav_overflow_count,

    // bulk/PPK path -> DDR/DMA, bypasses the router entirely
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
    output logic [9:0]   log_fifo_level,           // sync_fifo LEVEL_WIDTH for DEPTH=512 is 10 bits
    output logic         log_fifo_overflow_sticky,

    // PPS timestamp (reuses common async_event_capture directly)
    output logic         pps_valid,
    input  logic         pps_ready,
    output logic [63:0]  pps_timestamp,
    output logic         pps_overflow_sticky,
    output logic [31:0]  pps_overflow_count,

    // aggregated status, for PS visibility
    output logic [1:0]  rtk_stat,
    output logic         geofence_stat,
    output logic         tx_ready,
    output logic [31:0] checksum_fail_count,
    output logic         link_timeout_sticky
);

    // TODO: wire reset_n to a PS-controlled power-on/hard-reset sequencer;
    // safeboot_n only needs to be pulled low for firmware recovery, which
    // is not yet designed.
    assign reset_n    = 1'b1;
    assign safeboot_n = 1'b1;

    logic [7:0] cfg_cmd_data;
    logic       cfg_cmd_valid, cfg_cmd_ready, cfg_cmd_last;
    /* verilator lint_off UNUSEDSIGNAL */
    logic       sequence_busy, sequence_done; // TODO: not yet wired into gnss_health_monitor
    /* verilator lint_on UNUSEDSIGNAL */

    gnss_config_sequencer u_config_sequencer (
        .clk            (clk),
        .rst_n          (rst_n),
        .start_sequence (start_sequence),
        .sequence_busy  (sequence_busy),
        .sequence_done  (sequence_done),
        .cfg_cmd_data   (cfg_cmd_data),
        .cfg_cmd_valid  (cfg_cmd_valid),
        .cfg_cmd_ready  (cfg_cmd_ready),
        .cfg_cmd_last   (cfg_cmd_last)
    );

    logic [7:0] rx_byte;
    logic       rx_byte_valid;

    gnss_spi_engine #(
        .CLOCK_DIVIDE (SPI_CLOCK_DIVIDE)
    ) u_spi_engine (
        .clk           (clk),
        .rst_n         (rst_n),
        .sclk          (sclk),
        .mosi          (mosi),
        .miso          (miso),
        .cs_n          (cs_n),
        .cfg_cmd_data  (cfg_cmd_data),
        .cfg_cmd_valid (cfg_cmd_valid),
        .cfg_cmd_ready (cfg_cmd_ready),
        .cfg_cmd_last  (cfg_cmd_last),
        .poll_enable   (1'b1), // TODO: gate off during config replay if the GNSS needs a quiet bus
        .rx_byte       (rx_byte),
        .rx_byte_valid (rx_byte_valid),
        /* verilator lint_off PINCONNECTEMPTY */
        .engine_busy   () // TODO: not yet wired into gnss_health_monitor
        /* verilator lint_on PINCONNECTEMPTY */
    );

    logic [7:0]  payload_byte;
    logic        payload_byte_valid, payload_byte_last;
    logic [7:0]  msg_class, msg_id;
    logic [15:0] msg_length;
    logic        frame_sof, frame_done, checksum_fail;

    ubx_frame_sync u_frame_sync (
        .clk                (clk),
        .rst_n              (rst_n),
        .byte_in            (rx_byte),
        .byte_in_valid      (rx_byte_valid),
        .payload_byte       (payload_byte),
        .payload_byte_valid (payload_byte_valid),
        .payload_byte_last  (payload_byte_last),
        .msg_class          (msg_class),
        .msg_id             (msg_id),
        .msg_length         (msg_length),
        .frame_sof          (frame_sof),
        .frame_done         (frame_done),
        .checksum_fail      (checksum_fail)
    );

    logic [7:0] nav_byte, bulk_byte;
    logic       nav_byte_valid, nav_byte_last, bulk_byte_valid, bulk_byte_last;
    logic [7:0] nav_msg_id_w, bulk_msg_id_w;
    logic       nav_frame_done, nav_checksum_fail, bulk_frame_done, bulk_checksum_fail;

    gnss_msg_router u_router (
        .clk                (clk),
        .rst_n              (rst_n),
        .payload_byte       (payload_byte),
        .payload_byte_valid (payload_byte_valid),
        .payload_byte_last  (payload_byte_last),
        .msg_class          (msg_class),
        .msg_id             (msg_id),
        .msg_length         (msg_length),
        .frame_sof          (frame_sof),
        .frame_done         (frame_done),
        .checksum_fail      (checksum_fail),
        .nav_byte           (nav_byte),
        .nav_byte_valid     (nav_byte_valid),
        .nav_byte_last      (nav_byte_last),
        .nav_msg_id         (nav_msg_id_w),
        .nav_frame_done     (nav_frame_done),
        .nav_checksum_fail  (nav_checksum_fail),
        .bulk_byte          (bulk_byte),
        .bulk_byte_valid    (bulk_byte_valid),
        .bulk_byte_last     (bulk_byte_last),
        .bulk_msg_id        (bulk_msg_id_w),
        .bulk_frame_done    (bulk_frame_done),
        .bulk_checksum_fail (bulk_checksum_fail)
    );

    nav_solution_packer #(
        .MAX_PAYLOAD_BYTES (MAX_NAV_PAYLOAD_BYTES)
    ) u_nav_packer (
        .clk                (clk),
        .rst_n              (rst_n),
        .nav_byte           (nav_byte),
        .nav_byte_valid     (nav_byte_valid),
        .nav_byte_last      (nav_byte_last),
        .nav_msg_id         (nav_msg_id_w),
        .nav_frame_done     (nav_frame_done),
        .nav_checksum_fail  (nav_checksum_fail),
        .time_ticks         (time_ticks),
        .solution_valid     (solution_valid),
        .solution_ready     (solution_ready),
        .solution_data      (solution_data),
        .solution_length    (solution_length),
        .solution_msg_id    (solution_msg_id),
        .solution_timestamp (solution_timestamp),
        .overflow_sticky    (nav_overflow_sticky),
        .overflow_count     (nav_overflow_count)
    );

    bulk_log_framer u_bulk_framer (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .bulk_byte              (bulk_byte),
        .bulk_byte_valid        (bulk_byte_valid),
        .bulk_byte_last         (bulk_byte_last),
        .bulk_msg_id            (bulk_msg_id_w),
        .bulk_frame_done        (bulk_frame_done),
        .bulk_checksum_fail     (bulk_checksum_fail),
        .time_ticks             (time_ticks),
        .header_valid           (header_valid),
        .header_ready           (header_ready),
        .header_time_ticks      (header_time_ticks),
        .header_seq_num         (header_seq_num),
        .header_msg_id          (header_msg_id),
        .header_byte_count      (header_byte_count),
        .header_checksum_fail   (header_checksum_fail),
        .header_overflow_sticky (header_overflow_sticky),
        .header_overflow_count  (header_overflow_count),
        .log_byte                 (log_byte),
        .log_byte_valid           (log_byte_valid),
        .log_byte_ready           (log_byte_ready),
        .log_fifo_level           (log_fifo_level),
        .log_fifo_overflow_sticky (log_fifo_overflow_sticky)
    );

    async_event_capture u_pps_capture (
        .clk             (clk),
        .rst_n           (rst_n),
        .async_event     (timepulse),
        .time_ticks      (time_ticks),
        .event_valid     (pps_valid),
        .event_ready     (pps_ready),
        .event_timestamp (pps_timestamp),
        .overflow_sticky (pps_overflow_sticky),
        .overflow_count  (pps_overflow_count)
    );

    gnss_health_monitor u_health_monitor (
        .clk                 (clk),
        .rst_n               (rst_n),
        .frame_done          (frame_done),
        .checksum_fail       (checksum_fail),
        .rtk_stat_raw        (rtk_stat_pin),
        .geofence_stat_raw   (geofence_stat_pin),
        .tx_ready_n_raw      (tx_ready_n_pin),
        .rtk_stat            (rtk_stat),
        .geofence_stat       (geofence_stat),
        .tx_ready            (tx_ready),
        .checksum_fail_count (checksum_fail_count),
        .link_timeout_sticky (link_timeout_sticky)
    );

endmodule

`default_nettype wire
