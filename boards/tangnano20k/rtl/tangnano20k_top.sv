`timescale 1ns/1ps
`default_nettype none

// tangnano20k_top
//
// Bring-up top level for the Tang Nano 20K: runs the real gnss_scc against a
// ZED-F9P breakout and streams the raw SPI bytes to the host over the on-board
// USB-serial bridge (raw_byte_tap removes the idle filler so it fits).
//
// This is deliberately the raw stream, not decoded NAV-PVT: it proves the SPI
// wiring and timing before anything downstream of the SPI engine is trusted.
//
// gnss_scc tie-offs for this stage:
//   - start_sequence low: the receiver keeps its factory or saved configuration
//     and the config ROM is never played (see tangnano.py for the placeholder)
//   - the status GPIOs are not wired to the breakout yet
//   - solution/header/log/pps ready high: those outputs drain unused
//
// LEDs (active-low):
//   LED0  heartbeat, toggles once per second; the design is alive
//   LED1  toggles every 64 bytes sent to the host; data is flowing
//   LED2  raw_byte_tap FIFO overflowed (sticky); the stream has lost bytes
//   LED3  link_timeout_sticky. Counts UBX frames only, so this comes on about
//         7 s after power-up while the receiver is sending just NMEA (its default)
//   LED4  toggles on each TIMEPULSE edge captured by gnss_scc
//   LED5  off
//
// Inputs:  clk_27m, gnss_miso, gnss_timepulse
// Outputs: gnss_sclk, gnss_mosi, gnss_cs_n, uart_tx, led_n[5:0]
module tangnano20k_top #(
    parameter int CLK_HZ           = 27_000_000,
    parameter int BAUD             = 115_200,
    parameter int POR_WIDTH        = 10,          // 2^10 clocks = ~38 us of reset at 27 MHz
    parameter int SPI_CLOCK_DIVIDE = 26           // 27 MHz / 26 = 1.04 MHz SCLK (receiver max: 5.5 MHz)
) (
    input  logic       clk_27m,

    output logic       gnss_sclk,
    output logic       gnss_mosi,
    input  logic       gnss_miso,
    output logic       gnss_cs_n,
    input  logic       gnss_timepulse,

    output logic       uart_tx,
    output logic [5:0] led_n
);

    // ---------------------------------------------------------------
    // Power-on reset
    //
    // por_count is the one register in the design that cannot use the
    // repo's synchronous-reset convention, because it *is* the reset
    // generator -- there is no earlier reset to clear it. It relies on the
    // FPGA loading every flip-flop with its declared initial value (0)
    // when the bitstream is configured, then counts up to all-ones and
    // holds there. That makes this declaration initializer intentional, so
    // the PROCASSINIT warning is a genuine false positive here and is
    // waived rather than "fixed".
    // ---------------------------------------------------------------
    /* verilator lint_off PROCASSINIT */
    logic [POR_WIDTH-1:0] por_count = '0;
    /* verilator lint_on PROCASSINIT */
    logic                 rst_n;

    always_ff @(posedge clk_27m) begin
        if (!(&por_count)) por_count <= por_count + 1'b1;
    end

    // Combinational is fine here: every consumer samples rst_n synchronously,
    // so any glitch while por_count's bits change settles before the next edge.
    always_comb rst_n = &por_count;

    // ---------------------------------------------------------------
    // GNSS SCC
    // ---------------------------------------------------------------
    logic [63:0] time_ticks;

    timebase_counter u_timebase (
        .clk        (clk_27m),
        .rst_n      (rst_n),
        .time_ticks (time_ticks)
    );

    logic [7:0] rx_byte;
    logic       rx_byte_valid;
    logic       pps_valid;
    logic       link_timeout_sticky;

    /* verilator lint_off PINCONNECTEMPTY */
    gnss_scc #(
        .SPI_CLOCK_DIVIDE (SPI_CLOCK_DIVIDE)
    ) u_gnss_scc (
        .clk                      (clk_27m),
        .rst_n                    (rst_n),
        .time_ticks               (time_ticks),
        .sclk                     (gnss_sclk),
        .mosi                     (gnss_mosi),
        .miso                     (gnss_miso),
        .cs_n                     (gnss_cs_n),
        .reset_n                  (),
        .safeboot_n               (),
        .timepulse                (gnss_timepulse),
        .tx_ready_n_pin           (1'b1),
        .rtk_stat_pin             (1'b0),
        .geofence_stat_pin        (1'b0),
        .start_sequence           (1'b0),
        .solution_valid           (),
        .solution_ready           (1'b1),
        .solution_data            (),
        .solution_length          (),
        .solution_msg_id          (),
        .solution_timestamp       (),
        .nav_overflow_sticky      (),
        .nav_overflow_count       (),
        .header_valid             (),
        .header_ready             (1'b1),
        .header_time_ticks        (),
        .header_seq_num           (),
        .header_msg_id            (),
        .header_byte_count        (),
        .header_checksum_fail     (),
        .header_overflow_sticky   (),
        .header_overflow_count    (),
        .log_byte                 (),
        .log_byte_valid           (),
        .log_byte_ready           (1'b1),
        .log_fifo_level           (),
        .log_fifo_overflow_sticky (),
        .pps_valid                (pps_valid),
        .pps_ready                (1'b1),
        .pps_timestamp            (),
        .pps_overflow_sticky      (),
        .pps_overflow_count       (),
        .rtk_stat                 (),
        .geofence_stat            (),
        .tx_ready                 (),
        .checksum_fail_count      (),
        .link_timeout_sticky      (link_timeout_sticky),
        .rx_byte                  (rx_byte),
        .rx_byte_valid            (rx_byte_valid)
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // ---------------------------------------------------------------
    // Raw byte stream -> host
    // ---------------------------------------------------------------
    logic [7:0] tap_data;
    logic       tap_valid, tap_ready;
    logic       tap_overflow_sticky;

    raw_byte_tap u_raw_byte_tap (
        .clk             (clk_27m),
        .rst_n           (rst_n),
        .rx_byte         (rx_byte),
        .rx_byte_valid   (rx_byte_valid),
        .out_data        (tap_data),
        .out_valid       (tap_valid),
        .out_ready       (tap_ready),
        .overflow_sticky (tap_overflow_sticky)
    );

    uart_tx #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (BAUD)
    ) u_uart_tx (
        .clk      (clk_27m),
        .rst_n    (rst_n),
        .in_data  (tap_data),
        .in_valid (tap_valid),
        .in_ready (tap_ready),
        .tx       (uart_tx)
    );

    // ---------------------------------------------------------------
    // Status LEDs
    // ---------------------------------------------------------------
    localparam int TICK_WIDTH = $clog2(CLK_HZ);
    localparam logic [TICK_WIDTH-1:0] LAST_TICK = TICK_WIDTH'(CLK_HZ - 1);

    logic [TICK_WIDTH-1:0] tick_count;
    logic                  heartbeat;
    logic [5:0]            sent_count;
    logic                  pps_toggle;

    always_ff @(posedge clk_27m) begin
        if (!rst_n) begin
            tick_count <= '0;
            heartbeat  <= 1'b0;
            sent_count <= 6'h0;
            pps_toggle <= 1'b0;
        end else begin
            if (tick_count == LAST_TICK) begin
                tick_count <= '0;
                heartbeat  <= ~heartbeat;
            end else begin
                tick_count <= tick_count + 1'b1;
            end
            if (tap_valid && tap_ready) sent_count <= sent_count + 6'h1;
            if (pps_valid)              pps_toggle <= ~pps_toggle;
        end
    end

    always_comb begin
        led_n    = 6'b111111;
        led_n[0] = ~heartbeat;
        led_n[1] = ~sent_count[5];
        led_n[2] = ~tap_overflow_sticky;
        led_n[3] = ~link_timeout_sticky;
        led_n[4] = ~pps_toggle;
    end

endmodule

`default_nettype wire
