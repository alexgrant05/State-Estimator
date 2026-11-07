`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// bmp581_tang20k_top
//
// Bench bring-up for the BMP581 on the Sipeed Tang Nano 20K
// (Gowin GW2AR-LV18QN88C8/I7), no Pico needed:
//
//   BMP581 <--SPI (header pins)--> spimaster <--> bmp581_controller --> 6 LEDs
//      INT --------------------------------------------^
//
// Onboard LEDs (active LOW on this board, LED0..LED5):
//   LED0 on    = sensor found, checked and configured (init_done)
//   LED1 on    = a fault happened since power-up / last reset
//   LED2 blink = samples arriving AND pressure plausible (80..110 kPa)
//   LED3 blink = samples arriving, any value (tells LED2 problems apart)
//   LED4 on    = SPI traffic in the last ~50 ms (steady on = reading at 50 Hz)
//   LED5 blink = FPGA is configured and running (~1 Hz), independent of the sensor
// LED2/LED3 toggle every BEAT_SAMPLES samples: ~1 Hz at 50 Hz.
//
// Clock: the board's 27 MHz crystal (pin 4). 27 MHz is not a multiple of
// 2 MHz, so SCLK = 27 MHz / 28 = 964 kHz (the closest rate not above 1 MHz).
// Reset: button S1 (pin 88, active high), plus a power-on reset.
// ============================================================================

module bmp581_tang20k_top #(
    parameter int unsigned CLK_HZ       = 27_000_000,
    parameter int unsigned BEAT_SAMPLES = 25,
    parameter logic [23:0] P_MIN_RAW    = 24'(80_000 * 64),    // Pa x 64
    parameter logic [23:0] P_MAX_RAW    = 24'(110_000 * 64),
    // controller timing, overridable for simulation only
    parameter int unsigned PWRUP_WAIT_CYC   = CLK_HZ / 1000 * 3,
    parameter int unsigned SOFTRST_WAIT_CYC = CLK_HZ / 1000 * 3,
    parameter int unsigned STANDBY_WAIT_CYC = CLK_HZ / 1000 * 3,
    parameter int unsigned CS_IDLE_CYC      = CLK_HZ / 1_000_000 * 2,
    parameter int unsigned DRDY_TIMEOUT_CYC = CLK_HZ / 1000 * 100,
    parameter int unsigned RETRY_WAIT_CYC   = CLK_HZ / 1000 * 100,
    parameter int unsigned STALE_CYC        = CLK_HZ / 1000 * 10,
    parameter int unsigned ALIVE_HALF_CYC   = CLK_HZ / 2,           // LED5: 0.5 s on, 0.5 s off
    parameter int unsigned SPI_LED_CYC      = CLK_HZ / 20           // LED4 hold time: 50 ms
) (
    input  wire        clk,        // 27 MHz, pin 4
    input  wire        btn_rst,    // S1, pin 88, active high

    output wire        bmp_cs_n,
    output wire        bmp_sclk,
    output wire        bmp_mosi,   // to sensor SDI
    input  wire        bmp_miso,   // from sensor SDO
    input  wire        bmp_int,

    output wire [5:0]  led_n       // onboard LEDs, active low
);

    // SCLK divider: smallest even divider that keeps SCLK <= 1 MHz
    localparam int unsigned SPI_DIV = ((CLK_HZ + 1_999_999) / 2_000_000) * 2;

    // ------------------------------------------------------------------------
    // Reset: S1 synchronizer + power-on reset from configuration init values.
    // rst_i is synchronous and ACTIVE LOW, as spimaster and the controller expect.
    // ------------------------------------------------------------------------
    logic [1:0]  rst_meta = 2'b00;
    logic [15:0] por_sr   = '0;
    logic        rst_i    = 1'b0;

    always_ff @(posedge clk) begin
        rst_meta <= {rst_meta[0], ~btn_rst};
        por_sr   <= {por_sr[14:0], 1'b1};
        rst_i    <= rst_meta[1] & por_sr[15];
    end

    // ------------------------------------------------------------------------
    // Fabric time base
    // ------------------------------------------------------------------------
    logic [63:0] timestamp_now = '0;
    always_ff @(posedge clk) begin
        if (!rst_i) timestamp_now <= '0;
        else        timestamp_now <= timestamp_now + 1'b1;
    end

    // ------------------------------------------------------------------------
    // SPI master (mode 3) + BMP581 controller
    // ------------------------------------------------------------------------
    logic       spi_start, spi_hold_cs, spi_busy, spi_rx_valid;
    logic [7:0] spi_tx_data, spi_rx_data;

    spimaster #(.CLOCK_DIVIDE(SPI_DIV), .CPOL(1), .CPHA(1)) u_spi (
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
        .bus_req(bus_req), .bus_gnt(1'b1),            // only device on this SPI master
        .spi_start(spi_start), .spi_tx_data(spi_tx_data), .spi_hold_cs(spi_hold_cs),
        .spi_busy(spi_busy), .spi_rx_data(spi_rx_data), .spi_rx_valid(spi_rx_valid),
        .sample_valid(sample_valid), .sample_sensor_id(sample_sensor_id), .sample_seq(sample_seq),
        .sample_timestamp(sample_timestamp), .sample_press_raw(sample_press_raw),
        .sample_temp_raw(sample_temp_raw), .sample_flags(sample_flags),
        .init_done(init_done), .fault(fault), .fault_code(fault_code), .fault_count(fault_count),
        .drdy_timeout_count(drdy_timeout_count), .overrun_count(overrun_count)
    );

    // ------------------------------------------------------------------------
    // LED logic
    // ------------------------------------------------------------------------
    localparam int BW = $clog2(BEAT_SAMPLES + 1);
    logic [BW-1:0] beat_cnt = '0, raw_cnt = '0;
    logic          beat = 1'b0, raw = 1'b0;
    logic          p_ok;

    assign p_ok = (sample_press_raw >= P_MIN_RAW) && (sample_press_raw <= P_MAX_RAW)
                  && (sample_flags[1:0] == 2'b00);

    always_ff @(posedge clk) begin
        if (!rst_i || !init_done) begin
            beat_cnt <= '0; beat <= 1'b0;
            raw_cnt  <= '0; raw  <= 1'b0;
        end else if (sample_valid) begin
            if (raw_cnt == BW'(BEAT_SAMPLES - 1)) begin raw_cnt <= '0; raw <= ~raw; end
            else raw_cnt <= raw_cnt + 1'b1;
            if (p_ok) begin
                if (beat_cnt == BW'(BEAT_SAMPLES - 1)) begin beat_cnt <= '0; beat <= ~beat; end
                else beat_cnt <= beat_cnt + 1'b1;
            end
        end
    end

    logic fault_seen = 1'b0;
    always_ff @(posedge clk) begin
        if (!rst_i)     fault_seen <= 1'b0;
        else if (fault) fault_seen <= 1'b1;
    end

    // LED4: stays on for SPI_LED_CYC after any chip-select activity
    localparam int SW = $clog2(SPI_LED_CYC + 1);
    logic [SW-1:0] spi_hold = '0;
    always_ff @(posedge clk) begin
        if (!bmp_cs_n)          spi_hold <= SW'(SPI_LED_CYC);
        else if (spi_hold != 0) spi_hold <= spi_hold - 1'b1;
    end

    // LED5: free-running blink, proves the bitstream is loaded and clocked
    localparam int AW = $clog2(ALIVE_HALF_CYC + 1);
    logic [AW-1:0] alive_cnt = '0;
    logic          alive     = 1'b0;
    always_ff @(posedge clk) begin
        if (alive_cnt == AW'(ALIVE_HALF_CYC - 1)) begin alive_cnt <= '0; alive <= ~alive; end
        else alive_cnt <= alive_cnt + 1'b1;
    end

    // active-low drive
    assign led_n = ~{alive, (spi_hold != 0), raw, beat, fault_seen, init_done};

endmodule

`default_nettype wire
