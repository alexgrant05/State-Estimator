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
// If bring-up keeps failing, LED0..LED3 instead blink the fault code in binary
// (LED0 = 1, LED1 = 2, LED2 = 4, LED3 = 8), in step with LED5:
//   1 CHIP_ID wrong or no answer   2 NVM not ready   3 no reset flag
//   4..10 config write did not stick   11 OSR/ODR combination rejected
//
// Serial report: every 0.5 s a text line goes out on uart_tx (pin 69, 115200 8N1),
// which the onboard USB chip passes to your computer as a serial port:
//   ID=50 INIT=1 CODE=0 FAULTS=00 N=0123 P=6000A3 T=190012
//   ID     last byte the sensor returned for CHIP_ID (0x50 = BMP581)
//   INIT   1 = configured      CODE / FAULTS  last fault code / number of faults
//   N      sample counter      P / T          raw pressure / temperature (hex)
//   pressure in Pa = P / 64,  temperature in degC = T / 65536
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
    parameter int unsigned SPI_LED_CYC      = CLK_HZ / 20,          // LED4 hold time: 50 ms
    parameter int unsigned BAUD             = 115_200,
    parameter int unsigned REPORT_CYC       = CLK_HZ / 2            // one serial line every 0.5 s
) (
    input  wire        clk,        // 27 MHz, pin 4
    input  wire        btn_rst,    // S1, pin 88, active high

    output wire        bmp_cs_n,
    output wire        bmp_sclk,
    output wire        bmp_mosi,   // to sensor SDI
    input  wire        bmp_miso,   // from sensor SDO
    input  wire        bmp_int,

    output wire [5:0]  led_n,      // onboard LEDs, active low
    output wire        uart_tx     // pin 69, to the onboard USB-serial chip
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

    // While bring-up is failing (fault seen, not configured), LED0..LED3 show the
    // controller's fault_code in binary (LED0 = bit 0), blinking together with LED5.
    // Otherwise the normal status view.
    logic       show_code;
    logic [3:0] code_leds;
    assign show_code = fault_seen && !init_done;
    assign code_leds = fault_code & {4{alive}};

    // active-low drive
    assign led_n = show_code ? ~{alive, (spi_hold != 0), code_leds}
                             : ~{alive, (spi_hold != 0), raw, beat, fault_seen, init_done};

    // ------------------------------------------------------------------------
    // Debug: remember the byte the sensor returned for each CHIP_ID read.
    // A CHIP_ID read is two SPI bytes: address 0x81, then a dummy 0x00 whose
    // returned byte is the ID.
    // ------------------------------------------------------------------------
    logic [7:0] cur_tx = '0, prev_tx = '0, id_seen = '0;
    always_ff @(posedge clk) begin
        if (spi_start) begin
            prev_tx <= cur_tx;
            cur_tx  <= spi_tx_data;
        end
        if (spi_rx_valid && prev_tx == 8'h81 && cur_tx == 8'h00)
            id_seen <= spi_rx_data;
    end

    bmp581_reporter #(.CLK_HZ(CLK_HZ), .BAUD(BAUD), .REPORT_CYC(REPORT_CYC)) u_rep (
        .clk(clk), .rst_n(rst_i),
        .id_seen(id_seen), .init_done(init_done), .fault_code(fault_code),
        .fault_count(fault_count), .seq(sample_seq),
        .press_raw(sample_press_raw), .temp_raw(sample_temp_raw),
        .tx(uart_tx)
    );

endmodule

// ============================================================================
// bmp581_reporter: prints one status line over UART every REPORT_CYC cycles
//   ID=## INIT=# CODE=# FAULTS=## N=#### P=###### T=######<CR><LF>
// ============================================================================
module bmp581_reporter #(
    parameter int unsigned CLK_HZ     = 27_000_000,
    parameter int unsigned BAUD       = 115_200,
    parameter int unsigned REPORT_CYC = 13_500_000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  id_seen,
    input  wire        init_done,
    input  wire [3:0]  fault_code,
    input  wire [7:0]  fault_count,
    input  wire [15:0] seq,
    input  wire [23:0] press_raw,
    input  wire [23:0] temp_raw,
    output wire        tx
);
    localparam int LEN = 54;                       // characters before CR LF
    localparam logic [8*LEN-1:0] TPL = "ID=00 INIT=0 CODE=0 FAULTS=00 N=0000 P=000000 T=000000";

    function automatic logic [7:0] hexc(input logic [3:0] n);
        hexc = (n < 4'd10) ? (8'd48 + {4'd0, n}) : (8'd55 + {4'd0, n});   // '0'..'9', 'A'..'F'
    endfunction

    // snapshot so a line never mixes two samples
    logic [7:0]  id_s = '0, fcnt_s = '0;
    logic [3:0]  init_s = '0, code_s = '0;
    logic [15:0] seq_s = '0;
    logic [23:0] p_s = '0, t_s = '0;

    localparam int RW = $clog2(REPORT_CYC + 1);
    logic [RW-1:0] tick_cnt = '0;
    logic [7:0]    idx = '0;
    logic          sending = 1'b0;
    logic          tx_valid;
    logic          tx_busy;
    logic [7:0]    c;

    always @(*) begin
        if (idx == 8'(LEN))          c = 8'h0D;      // CR
        else if (idx == 8'(LEN + 1)) c = 8'h0A;      // LF
        else                         c = TPL[8*(LEN-1-int'(idx)) +: 8];
        case (idx)
            8'd3: c = hexc(id_s[7:4]);
            8'd4: c = hexc(id_s[3:0]);
            8'd11: c = hexc(init_s[3:0]);
            8'd18: c = hexc(code_s[3:0]);
            8'd27: c = hexc(fcnt_s[7:4]);
            8'd28: c = hexc(fcnt_s[3:0]);
            8'd32: c = hexc(seq_s[15:12]);
            8'd33: c = hexc(seq_s[11:8]);
            8'd34: c = hexc(seq_s[7:4]);
            8'd35: c = hexc(seq_s[3:0]);
            8'd39: c = hexc(p_s[23:20]);
            8'd40: c = hexc(p_s[19:16]);
            8'd41: c = hexc(p_s[15:12]);
            8'd42: c = hexc(p_s[11:8]);
            8'd43: c = hexc(p_s[7:4]);
            8'd44: c = hexc(p_s[3:0]);
            8'd48: c = hexc(t_s[23:20]);
            8'd49: c = hexc(t_s[19:16]);
            8'd50: c = hexc(t_s[15:12]);
            8'd51: c = hexc(t_s[11:8]);
            8'd52: c = hexc(t_s[7:4]);
            8'd53: c = hexc(t_s[3:0]);
            default: ;
        endcase
    end

    assign tx_valid = sending && !tx_busy;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            tick_cnt <= '0;
            sending  <= 1'b0;
            idx      <= '0;
        end else if (!sending) begin
            if (tick_cnt == RW'(REPORT_CYC - 1)) begin
                tick_cnt <= '0;
                id_s   <= id_seen;
                init_s <= {3'd0, init_done};
                code_s <= fault_code;
                fcnt_s <= fault_count;
                seq_s  <= seq;
                p_s    <= press_raw;
                t_s    <= temp_raw;
                idx    <= '0;
                sending <= 1'b1;
            end else begin
                tick_cnt <= tick_cnt + 1'b1;
            end
        end else if (tx_valid) begin
            if (idx == 8'(LEN + 1)) sending <= 1'b0;
            idx <= idx + 1'b1;
        end
    end

    uart_tx_8n1 #(.DIV(CLK_HZ / BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(c), .valid(tx_valid), .busy(tx_busy), .tx(tx)
    );
endmodule

// ============================================================================
// uart_tx_8n1: 1 start bit, 8 data bits LSB first, 1 stop bit, idle high.
// Takes a byte when valid && !busy.
// ============================================================================
module uart_tx_8n1 #(
    parameter int unsigned DIV = 234                    // clock cycles per bit
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       valid,
    output wire       busy,
    output wire       tx
);
    localparam int DW = $clog2(DIV + 1);
    logic [DW-1:0] cnt    = '0;
    logic [3:0]    bitn   = '0;
    logic [9:0]    sh     = 10'h3FF;
    logic          busy_r = 1'b0;
    logic          tx_r   = 1'b1;          // idle high from configuration on
    assign busy = busy_r;
    assign tx   = tx_r;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy_r <= 1'b0;
            tx_r   <= 1'b1;
            cnt    <= '0;
            bitn   <= '0;
        end else if (!busy_r) begin
            tx_r <= 1'b1;
            if (valid) begin
                sh     <= {1'b1, data, 1'b0};         // stop, data, start (LSB out first)
                busy_r <= 1'b1;
                cnt    <= '0;
                bitn   <= '0;
            end
        end else begin
            tx_r <= sh[0];
            if (cnt == DW'(DIV - 1)) begin
                cnt <= '0;
                sh  <= {1'b1, sh[9:1]};
                if (bitn == 4'd9) busy_r <= 1'b0;
                else              bitn   <= bitn + 1'b1;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
