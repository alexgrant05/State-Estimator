`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// bmp581_arty_top
//
// Bench bring-up for the BMP581 on the Digilent Arty S7-25, no Pico needed:
//
//   BMP581 <--SPI (Pmod JA)--> spimaster <--> bmp581_controller --> LD2..LD5
//      INT ------------------------------------------^
//
// Onboard LEDs:
//   LD2  led[0]  on    = sensor found, checked and configured (init_done)
//   LD3  led[1]  on    = a fault happened since power-up / last reset
//   LD4  led[2]  blink = samples arriving AND pressure plausible (80..110 kPa)
//   LD5  led[3]  blink = samples arriving, any value (tells LD4 problems apart)
// Both blinks toggle every BEAT_SAMPLES samples: ~1 Hz at 50 Hz.
//
// BTN0 = reset (active high). Clock = the board's 12 MHz oscillator; the SPI
// master divides it by 12 for a 1 MHz SCLK. A power-on reset from the
// configuration init values covers start-up without pressing anything.
// ============================================================================

module bmp581_arty_top #(
    parameter int unsigned CLK_HZ       = 12_000_000,     // Arty S7 12 MHz oscillator
    parameter int unsigned BEAT_SAMPLES = 25,             // 25 samples at 50 Hz = 0.5 s per toggle
    // plausible pressure window, raw units (Pa x 64): 80 kPa .. 110 kPa
    parameter logic [23:0] P_MIN_RAW    = 24'(80_000 * 64),
    parameter logic [23:0] P_MAX_RAW    = 24'(110_000 * 64),
    // controller timing, overridable for simulation only
    parameter int unsigned PWRUP_WAIT_CYC   = CLK_HZ / 1000 * 3,
    parameter int unsigned SOFTRST_WAIT_CYC = CLK_HZ / 1000 * 3,
    parameter int unsigned STANDBY_WAIT_CYC = CLK_HZ / 1000 * 3,
    parameter int unsigned CS_IDLE_CYC      = CLK_HZ / 1_000_000 * 2,
    parameter int unsigned DRDY_TIMEOUT_CYC = CLK_HZ / 1000 * 100,
    parameter int unsigned RETRY_WAIT_CYC   = CLK_HZ / 1000 * 100,
    parameter int unsigned STALE_CYC        = CLK_HZ / 1000 * 10
) (
    input  wire  clk,          // 12 MHz, pin F14
    input  wire  btn_rst,      // BTN0, active high

    // BMP581 pins
    output wire  bmp_cs_n,
    output wire  bmp_sclk,
    output wire  bmp_mosi,     // to sensor SDI
    input  wire  bmp_miso,     // from sensor SDO
    input  wire  bmp_int,

    // onboard LEDs LD2..LD5
    output logic [3:0] led
);

    if (CLK_HZ % 2_000_000 != 0) begin : g_clk_chk
        $error("bmp581_arty_top: CLK_HZ must be a multiple of 2 MHz for an exact 1 MHz SCLK");
    end

    // ------------------------------------------------------------------------
    // Reset: power-on reset from configuration init values, plus BTN0.
    // Everything below uses rst_i, synchronous and ACTIVE LOW (repo convention,
    // same polarity spimaster and bmp581_controller expect).
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [1:0] rst_meta = 2'b00;
    logic [15:0] por_sr = '0;
    logic        rst_i  = 1'b0;

    always_ff @(posedge clk) begin
        rst_meta <= {rst_meta[0], ~btn_rst};
        por_sr   <= {por_sr[14:0], 1'b1};
        rst_i    <= rst_meta[1] & por_sr[15];
    end

    // ------------------------------------------------------------------------
    // Fabric time base (the controller timestamps every sample with it)
    // ------------------------------------------------------------------------
    logic [63:0] timestamp_now = '0;
    always_ff @(posedge clk) begin
        if (!rst_i) timestamp_now <= '0;
        else        timestamp_now <= timestamp_now + 1'b1;
    end

    // ------------------------------------------------------------------------
    // SPI master (mode 3, 1 MHz) + BMP581 controller
    // ------------------------------------------------------------------------
    logic       spi_start, spi_hold_cs, spi_busy, spi_rx_valid;
    logic [7:0] spi_tx_data, spi_rx_data;

    spimaster #(.CLOCK_DIVIDE(CLK_HZ / 1_000_000), .CPOL(1), .CPHA(1)) u_spi (
        .sys_clk(clk), .rst(rst_i),
        .start(spi_start), .tx_data(spi_tx_data), .hold_cs(spi_hold_cs),
        .busy(spi_busy), .rx_data(spi_rx_data), .rx_valid(spi_rx_valid),
        .cs_n(bmp_cs_n), .sclk(bmp_sclk), .mosi(bmp_mosi), .miso(bmp_miso)
    );

    logic        sample_valid, init_done, fault, bus_req;
    logic [7:0]  sample_sensor_id, sample_flags, fault_count;
    logic [15:0] sample_seq, drdy_timeout_count, overrun_count;
    logic [63:0] sample_timestamp;
    logic [23:0] sample_press_raw, sample_temp_raw;
    logic [3:0]  fault_code;

    bmp581_controller #(
        .CLK_HZ(CLK_HZ),
        .PWRUP_WAIT_CYC(PWRUP_WAIT_CYC), .SOFTRST_WAIT_CYC(SOFTRST_WAIT_CYC),
        .STANDBY_WAIT_CYC(STANDBY_WAIT_CYC), .CS_IDLE_CYC(CS_IDLE_CYC),
        .DRDY_TIMEOUT_CYC(DRDY_TIMEOUT_CYC), .RETRY_WAIT_CYC(RETRY_WAIT_CYC),
        .STALE_CYC(STALE_CYC)
    ) u_bmp (
        .sys_clk(clk), .rst(rst_i),
        .timestamp_now(timestamp_now),
        .bmp_int(bmp_int),
        .bus_req(bus_req), .bus_gnt(1'b1),          // only device on this SPI master
        .spi_start(spi_start), .spi_tx_data(spi_tx_data), .spi_hold_cs(spi_hold_cs),
        .spi_busy(spi_busy), .spi_rx_data(spi_rx_data), .spi_rx_valid(spi_rx_valid),
        .sample_valid(sample_valid), .sample_sensor_id(sample_sensor_id), .sample_seq(sample_seq),
        .sample_timestamp(sample_timestamp), .sample_press_raw(sample_press_raw),
        .sample_temp_raw(sample_temp_raw), .sample_flags(sample_flags),
        .init_done(init_done), .fault(fault), .fault_code(fault_code), .fault_count(fault_count),
        .drdy_timeout_count(drdy_timeout_count), .overrun_count(overrun_count)
    );

    // ------------------------------------------------------------------------
    // LEDs
    // ------------------------------------------------------------------------
    localparam int BW = $clog2(BEAT_SAMPLES + 1);
    logic [BW-1:0] beat_cnt = '0;
    logic          beat     = 1'b0;
    logic          p_ok;

    assign p_ok = (sample_press_raw >= P_MIN_RAW) && (sample_press_raw <= P_MAX_RAW)
                  && (sample_flags[1:0] == 2'b00);   // not a skipped measurement

    always_ff @(posedge clk) begin
        if (!rst_i || !init_done) begin
            beat_cnt <= '0;
            beat     <= 1'b0;
        end else if (sample_valid && p_ok) begin
            if (beat_cnt == BW'(BEAT_SAMPLES - 1)) begin
                beat_cnt <= '0;
                beat     <= ~beat;
            end else begin
                beat_cnt <= beat_cnt + 1'b1;
            end
        end
    end

    logic [BW-1:0] raw_cnt = '0;
    logic          raw     = 1'b0;
    always_ff @(posedge clk) begin
        if (!rst_i || !init_done) begin
            raw_cnt <= '0;
            raw     <= 1'b0;
        end else if (sample_valid) begin
            if (raw_cnt == BW'(BEAT_SAMPLES - 1)) begin
                raw_cnt <= '0;
                raw     <= ~raw;
            end else begin
                raw_cnt <= raw_cnt + 1'b1;
            end
        end
    end

    // fault_seen: sticky until reset, so a brief fault that recovered on its
    // own is still visible on the bench
    logic fault_seen = 1'b0;
    always_ff @(posedge clk) begin
        if (!rst_i)          fault_seen <= 1'b0;
        else if (fault)      fault_seen <= 1'b1;
    end

    assign led = {raw, beat, fault_seen, init_done};

endmodule

`default_nettype wire
