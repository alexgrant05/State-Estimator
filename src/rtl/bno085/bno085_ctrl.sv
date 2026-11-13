// ============================================================================
// BNO085 controller (SPI / SHTP / SH-2), raw accelerometer, gyro, magnetometer
//
//   bno085_ctrl   : sequencer. RSTN pulse -> boot -> microcode init (Set
//                   Feature per sensor) -> streaming, with automatic recovery
//                   (hub self-reset, per-sensor watchdog, boot/wake timeouts).
//   bno085_shtp   : transport. INTN / WAKE / CS handshake, SHTP header,
//                   length-driven reads, full-duplex writes. Emits a payload
//                   byte stream with sop / eop / channel / cont.
//   bno085_parser : walks the batched reports with a length table and
//                   updates the per-sensor outputs, 1-cycle *_valid each.
//   spimaster     : moves one byte at a time (mode 3).
//
// Handshake between layers: pulse start for 1 cycle, wait for a 1-cycle done.
//
// Pins (PS1 and BOOTN are tied high on the board):
//   rst_n      : driven low from FPGA configuration, so the hub stays in reset
//                until this controller releases it.
//   ps0_wake_n : PS0 at reset release (must be 1 = SPI mode, and stays 1 until
//                the hub's first INTN), WAKE afterwards (active low).
//   intn       : hub -> host, active low. Synchronized here.
//   miso       : synchronized here (2 FFs) before spimaster. That moves the
//                effective sample point 2 sys_clks earlier; checked below.
//
// Boot (CEVA spi_hal.c sh2_spi_hal_open): RSTN low >= 10 ms with CS high,
// SCLK idle high and WAKE high; release RSTN; the hub asserts INTN after
// ~94 ms and sends its advertisement (ch0), reset complete (ch1, payload
// 0x01) and an unsolicited init response (0xF1, ch2). Those are read and
// discarded. Reset complete comes from the SHTP layer, before the SH-2
// sensor hub application is ready, so the microcode waits SETTLE_CYCLES
// after it before sending Set Feature (reports still being read meanwhile).
//
// rst is active high and may be asynchronous: it is synchronized here, and a
// power-on reset is generated internally, so the design comes up correctly
// even if rst is never pulsed.
//
// All timing parameters default to real-time values derived from CLK_HZ.
// Override CLK_HZ for a different board clock; override individual cycle
// counts only for simulation.
// ============================================================================

module bno085_ctrl #(
    parameter int CLK_HZ              = 100_000_000,
    parameter int CLK_DIV             = CLK_HZ / 2_000_000 * 2, // SCLK ~1 MHz, kept even (hub max 3 MHz)

    // Report intervals requested with Set Feature, in us (0 = sensor off).
    // Datasheet max rates: accel 500 Hz, gyro 400 Hz, mag 100 Hz.
    parameter int ACCEL_INTERVAL_US   = 2_000,              // 500 Hz
    parameter int GYRO_INTERVAL_US    = 2_500,              // 400 Hz
    parameter int MAG_INTERVAL_US     = 10_000,             // 100 Hz

    parameter int RESET_HOLD_CYCLES   = CLK_HZ / 100,       // 10 ms RSTN low
    parameter int BOOT_TIMEOUT_CYCLES = CLK_HZ * 2,         // 2 s for INTN + reset complete
    parameter int CS_GAP_CYCLES       = CLK_HZ / 500_000,   // 2 us CS high between transfers
    parameter int WAKE_TIMEOUT_CYCLES = CLK_HZ / 100,       // 10 ms WAKE -> INTN (spec: 150 us max)
    parameter int INTN_STALE_CYCLES   = CLK_HZ / 100,       // 10 ms INTN stuck low -> read anyway
    parameter int FC_TIMEOUT_CYCLES   = CLK_HZ / 10,        // 100 ms, OP_EXPECT_FC only
    parameter int SETTLE_CYCLES       = CLK_HZ / 4,         // 250 ms after reset complete (OP_SETTLE)
    // Also enable the calibrated accelerometer (0x01), gyroscope (0x02) and
    // magnetic field (0x03) at the same intervals, as raw-data examples for
    // this chip do (bench: raw-only setup produced no reports). Their reports
    // are read and skipped by the parser.
    parameter bit CAL_COMPANIONS      = 1'b1,
    parameter int MAX_RX_LEN          = 32767,              // see bno085_shtp
    parameter bit WRITE_RX_DELIVER    = 1'b1,               // see bno085_shtp

    // Watchdog: a sensor with no report for WDOG_PERIODS of its interval
    // triggers an RSTN pulse and full re-init.
    parameter int WDOG_PERIODS        = 10,
    parameter int ACCEL_WDOG_CYCLES   = WDOG_PERIODS * (CLK_HZ / 1_000_000) * ACCEL_INTERVAL_US,
    parameter int GYRO_WDOG_CYCLES    = WDOG_PERIODS * (CLK_HZ / 1_000_000) * GYRO_INTERVAL_US,
    parameter int MAG_WDOG_CYCLES     = WDOG_PERIODS * (CLK_HZ / 1_000_000) * MAG_INTERVAL_US,
    // Until a sensor's first report after setup, its watchdog allows at
    // least this long: the hub has to power the sensor up after Set Feature.
    parameter int WDOG_START_CYCLES   = CLK_HZ / 2          // 500 ms
) (
    input  logic               clk,
    input  logic               rst,

    // sensor pins
    output logic               cs_n,
    output logic               sclk,
    output logic               mosi,
    input  logic               miso,
    input  logic               intn,
    output logic               ps0_wake_n,
    output logic               rst_n = 1'b0,

    // raw accelerometer (report 0x14)
    output logic               accel_valid,     // 1-cycle pulse per report
    output logic signed [15:0] accel_x, accel_y, accel_z,   // ADC counts
    output logic [31:0]        accel_ts,        // hub timestamp, us
    output logic [7:0]         accel_status,    // b1:0 accuracy, b7:2 delay[13:8]
    output logic [7:0]         accel_seq,

    // raw gyroscope (report 0x15)
    output logic               gyro_valid,
    output logic signed [15:0] gyro_x, gyro_y, gyro_z,
    output logic signed [15:0] gyro_temp,
    output logic [31:0]        gyro_ts,
    output logic [7:0]         gyro_status,
    output logic [7:0]         gyro_seq,

    // raw magnetometer (report 0x16)
    output logic               mag_valid,
    output logic signed [15:0] mag_x, mag_y, mag_z,
    output logic [31:0]        mag_ts,
    output logic [7:0]         mag_status,
    output logic [7:0]         mag_seq,

    // status / debug
    output logic               init_done,            // high while streaming
    output logic               fault,                // set on any failure; clears on the next good report
    output logic [3:0]         fault_cause,          // last fault, FC_* below
    output logic [3:0]         fault_pc,             // last fault: microcode step that was running
    output logic [7:0]         reinit_count,         // RSTN recoveries since reset (saturates)
    output logic [7:0]         reset_complete_count, // reset-complete messages seen (saturates)
    output logic [15:0]        unknown_id_count,     // packets cut short by an unknown report ID
    output logic [15:0]        cont_drop_count,      // continuation fragments dropped
    output logic [15:0]        trunc_count,          // packets truncated by a write (or MAX_RX_LEN)
    output logic [7:0]         last_chan,            // header of the last transfer
    output logic [14:0]        last_len,
    output logic [31:0]        granted_accel_us,     // from Get Feature Responses (0 = none yet)
    output logic [31:0]        granted_gyro_us,
    output logic [31:0]        granted_mag_us
);

    // fault_cause values
    localparam logic [3:0] FC_NONE          = 4'd0;
    localparam logic [3:0] FC_BOOT_NO_INTN  = 4'd1;  // INTN never asserted after RSTN release
    localparam logic [3:0] FC_BOOT_NO_RESET = 4'd2;  // INTN seen, but no reset complete
    localparam logic [3:0] FC_WAKE_TIMEOUT  = 4'd3;  // WAKE asserted, INTN never answered
    localparam logic [3:0] FC_FC_TIMEOUT    = 4'd4;  // OP_EXPECT_FC timed out (not fatal)
    localparam logic [3:0] FC_WDOG_ACCEL    = 4'd5;  // no accel report for WDOG_PERIODS
    localparam logic [3:0] FC_WDOG_GYRO     = 4'd6;
    localparam logic [3:0] FC_WDOG_MAG      = 4'd7;
    localparam logic [3:0] FC_ILLEGAL       = 4'd15; // illegal state encoding

    // ------------------------------------------------------------------------
    // Parameter checks (elaboration time)
    // ------------------------------------------------------------------------
    if (CLK_DIV < 2 || CLK_DIV % 2 != 0) begin : g_chk_div
        $error("bno085_ctrl: CLK_DIV (%0d) must be even and >= 2", CLK_DIV);
    end
    if (CLK_HZ / CLK_DIV > 3_000_000) begin : g_chk_fsclk
        $error("bno085_ctrl: SCLK %0d Hz exceeds the BNO085 3 MHz limit", CLK_HZ / CLK_DIV);
    end
    if (CLK_DIV / 2 > 65536) begin : g_chk_div_max
        $error("bno085_ctrl: CLK_DIV (%0d) too large for spimaster's 16-bit counter", CLK_DIV);
    end
    // CS setup to first SCLK edge >= 100 ns (tcssu). spimaster gives CLK_DIV/2 cycles.
    if (longint'(CLK_DIV / 2) * 1_000_000_000 < longint'(CLK_HZ) * 100) begin : g_chk_cssu
        $error("bno085_ctrl: CS setup (CLK_DIV/2 cycles) under the 100 ns tcssu");
    end
    // MISO: hub drives it <= 35 ns (tsov) after falling SCLK; we sample it about
    // CLK_DIV/2 - 3 cycles later (2-FF synchronizer + spimaster's early sample).
    if (longint'(CLK_DIV / 2 - 3) * 1_000_000_000 < longint'(CLK_HZ) * 50) begin : g_chk_miso
        $error("bno085_ctrl: CLK_DIV too small for tsov + MISO synchronizer");
    end
    if (longint'(RESET_HOLD_CYCLES) * 1_000_000 < longint'(CLK_HZ)) begin : g_chk_rsthold
        $warning("bno085_ctrl: RESET_HOLD_CYCLES under 1 us (CEVA holds 10 ms)");
    end
    if (CS_GAP_CYCLES < 4) begin : g_chk_gap
        $error("bno085_ctrl: CS_GAP_CYCLES must be >= 4 (INTN synchronizer latency)");
    end
    if (ACCEL_INTERVAL_US < 0 || GYRO_INTERVAL_US < 0 || MAG_INTERVAL_US < 0) begin : g_chk_iv
        $error("bno085_ctrl: report intervals must be >= 0");
    end
    if (ACCEL_INTERVAL_US == 0 && GYRO_INTERVAL_US == 0 && MAG_INTERVAL_US == 0) begin : g_chk_none
        $error("bno085_ctrl: all sensors disabled");
    end
    if (longint'(WDOG_PERIODS) * (CLK_HZ / 1_000_000) *
        ((ACCEL_INTERVAL_US > GYRO_INTERVAL_US) ?
            ((ACCEL_INTERVAL_US > MAG_INTERVAL_US) ? ACCEL_INTERVAL_US : MAG_INTERVAL_US) :
            ((GYRO_INTERVAL_US > MAG_INTERVAL_US) ? GYRO_INTERVAL_US : MAG_INTERVAL_US))
        > 64'h7FFF_FFFF) begin : g_chk_wdog
        $error("bno085_ctrl: watchdog cycle count overflows 32 bits; lower WDOG_PERIODS");
    end

    // ------------------------------------------------------------------------
    // Reset: 2-FF synchronizer on rst, OR'd with an 8-cycle power-on reset
    // that relies on FPGA register initial values.
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [1:0] rst_meta = 2'b11;
    logic [7:0] por_sr = 8'hFF;
    logic       rst_i  = 1'b1;

    always_ff @(posedge clk) begin
        rst_meta <= {rst_meta[0], rst};
        por_sr   <= {por_sr[6:0], 1'b0};
        rst_i    <= rst_meta[1] | por_sr[7];
    end

    // Input synchronizers. INTN powers up "high" (idle).
    (* ASYNC_REG = "TRUE" *) logic [1:0] intn_meta = 2'b11;
    (* ASYNC_REG = "TRUE" *) logic [1:0] miso_meta = 2'b00;
    always_ff @(posedge clk) begin
        intn_meta <= {intn_meta[0], intn};
        miso_meta <= {miso_meta[0], miso};
    end
    wire intn_s = intn_meta[1];
    wire miso_s = miso_meta[1];

    // ------------------------------------------------------------------------
    // Microcode. Edit this table to change the init sequence.
    // Unused slots must stay OP_END.
    // ------------------------------------------------------------------------
    typedef enum logic [2:0] {
        OP_WAIT_RESET_COMPLETE,  // wait for ch1 reset complete (BOOT_TIMEOUT_CYCLES)
        OP_SET_FEATURE,          // send Set Feature (0xFD) for id at interval_us
        OP_EXPECT_FC,            // optional: wait for Get Feature Response for id
        OP_SETTLE,               // wait SETTLE_CYCLES (hub reads continue)
        OP_PROD_ID,              // send Product ID Request (F9 00); the hub answers
                                 // 0xF8 (skipped by the parser). Kept from bring-up:
                                 // it's what the working hardware sequence sends.
        OP_END                   // init finished, start streaming
    } op_t;

    typedef struct packed {
        op_t         op;
        logic [7:0]  id;
        logic [31:0] interval_us;
    } uinstr_t;

    localparam logic [7:0] ID_RAW_ACCEL = 8'h14;
    localparam logic [7:0] ID_RAW_GYRO  = 8'h15;
    localparam logic [7:0] ID_RAW_MAG   = 8'h16;
    localparam logic [7:0] ID_CAL_ACCEL = 8'h01;
    localparam logic [7:0] ID_CAL_GYRO  = 8'h02;
    localparam logic [7:0] ID_CAL_MAG   = 8'h03;

    localparam int CAL_ACCEL_US = CAL_COMPANIONS ? ACCEL_INTERVAL_US : 0;
    localparam int CAL_GYRO_US  = CAL_COMPANIONS ? GYRO_INTERVAL_US  : 0;
    localparam int CAL_MAG_US   = CAL_COMPANIONS ? MAG_INTERVAL_US   : 0;

    localparam uinstr_t ROM [16] = '{
        '{OP_WAIT_RESET_COMPLETE, 8'h00,        32'd0},                     // 0
        '{OP_SETTLE,              8'h00,        32'd0},                     // 1
        '{OP_PROD_ID,             8'h00,        32'd0},                     // 2
        '{OP_SET_FEATURE,         ID_CAL_ACCEL, 32'(CAL_ACCEL_US)},         // 3
        '{OP_SET_FEATURE,         ID_CAL_GYRO,  32'(CAL_GYRO_US)},          // 4
        '{OP_SET_FEATURE,         ID_CAL_MAG,   32'(CAL_MAG_US)},           // 5
        '{OP_SET_FEATURE,         ID_RAW_ACCEL, 32'(ACCEL_INTERVAL_US)},    // 6
        '{OP_SET_FEATURE,         ID_RAW_GYRO,  32'(GYRO_INTERVAL_US)},     // 7
        '{OP_SET_FEATURE,         ID_RAW_MAG,   32'(MAG_INTERVAL_US)},      // 8
        '{OP_END,                 8'h00,        32'd0},                     // 9
        '{OP_END,                 8'h00,        32'd0},
        '{OP_END,                 8'h00,        32'd0},
        '{OP_END,                 8'h00,        32'd0},
        '{OP_END,                 8'h00,        32'd0},
        '{OP_END,                 8'h00,        32'd0},
        '{OP_END,                 8'h00,        32'd0}
    };

    // Set Feature packet: channel 2, 4-byte header + 17-byte payload.
    //   FD, id, flags=0, change_sens(2)=0, report_interval_us(4, LE),
    //   batch_interval_us(4)=0, sensor_specific(4)=0
    localparam logic [7:0] SET_FEATURE_LEN  = 8'd21;
    localparam logic [7:0] PROD_ID_LEN      = 8'd6;     // header + F9 00
    localparam logic [7:0] CHAN_CONTROL     = 8'd2;

    // ------------------------------------------------------------------------
    // Sequencer state
    // ------------------------------------------------------------------------
    typedef enum logic [2:0] {
        RST_HOLD,   // RSTN low, transport held in reset
        FETCH,      // look at ROM[pc] and act on it
        WAIT_WR,    // Set Feature write in progress
        WAIT_FC,    // OP_EXPECT_FC: waiting for the matching 0xFC
        STREAM      // reading reports; watchdogs running
    } state_t;

    state_t      state;
    logic [3:0]  pc;
    uinstr_t     instr;
    int unsigned delay;            // shared by the timed waits
    int unsigned wdog_a, wdog_g, wdog_m;
    logic        seen_a, seen_g, seen_m;   // first report since STREAM entry

    function automatic int unsigned wdog_limit(input logic seen, input int unsigned periodic);
        return (seen || periodic >= WDOG_START_CYCLES) ? periodic : WDOG_START_CYCLES;
    endfunction
    wire [31:0] lim_a = wdog_limit(seen_a, ACCEL_WDOG_CYCLES);
    wire [31:0] lim_g = wdog_limit(seen_g, GYRO_WDOG_CYCLES);
    wire [31:0] lim_m = wdog_limit(seen_m, MAG_WDOG_CYCLES);
    logic        rc_seen;          // reset complete seen since RSTN release
    logic        booted;           // OP_WAIT_RESET_COMPLETE has passed
    logic        restart;          // hub reset itself: rerun the microcode
    logic        saw_intn;
    logic        fc_match;         // 0xFC for instr.id seen since its Set Feature

    assign instr = ROM[pc];

    // lower-layer handshakes
    logic        wr_start, wr_done, wr_err;
    logic [7:0]  wr_idx, wr_byte;
    logic        hdr_valid, hdr_cont, hdr_in_write, rx_trunc;
    logic [14:0] hdr_len;
    logic        rx_valid, rx_sop, rx_eop, rx_cont;
    logic [7:0]  rx_data, rx_chan;
    logic        p_reset_complete, p_unknown_id, p_short_report;
    logic        fc_valid;
    logic [7:0]  fc_id;
    logic [31:0] fc_interval_us;

    // Set Feature payload byte, by index
    always_comb begin
        if (instr.op == OP_PROD_ID)
            wr_byte = (wr_idx == 8'd0) ? 8'hF9 : 8'h00;
        else case (wr_idx)
            8'd0:    wr_byte = 8'hFD;
            8'd1:    wr_byte = instr.id;
            8'd5:    wr_byte = instr.interval_us[7:0];
            8'd6:    wr_byte = instr.interval_us[15:8];
            8'd7:    wr_byte = instr.interval_us[23:16];
            8'd8:    wr_byte = instr.interval_us[31:24];
            default: wr_byte = 8'h00;
        endcase
    end

    // transport and spimaster are held in reset with the hub
    logic xrst = 1'b1;
    always_ff @(posedge clk) xrst <= rst_i || state == RST_HOLD;

    wire any_valid = accel_valid || gyro_valid || mag_valid;

    function automatic logic [15:0] sat16(input logic [15:0] v);
        return (v == 16'hFFFF) ? v : v + 1;
    endfunction
    function automatic logic [7:0] sat8(input logic [7:0] v);
        return (v == 8'hFF) ? v : v + 1;
    endfunction

    always_ff @(posedge clk) begin
        wr_start <= 1'b0;   // pulses: default low

        if (rst_i) begin
            state                <= RST_HOLD;
            pc                   <= '0;
            delay                <= 0;
            wdog_a               <= 0;
            wdog_g               <= 0;
            wdog_m               <= 0;
            seen_a               <= 1'b0;
            seen_g               <= 1'b0;
            seen_m               <= 1'b0;
            rc_seen              <= 1'b0;
            booted               <= 1'b0;
            restart              <= 1'b0;
            saw_intn             <= 1'b0;
            fc_match             <= 1'b0;
            rst_n                <= 1'b0;
            init_done            <= 1'b0;
            fault                <= 1'b0;
            fault_cause          <= FC_NONE;
            fault_pc             <= '0;
            reinit_count         <= '0;
            reset_complete_count <= '0;
            unknown_id_count     <= '0;
            cont_drop_count      <= '0;
            trunc_count          <= '0;
            last_chan            <= '0;
            last_len             <= '0;
            granted_accel_us     <= '0;
            granted_gyro_us      <= '0;
            granted_mag_us       <= '0;
        end else begin
            // ---------------- bookkeeping from the lower layers ----------------
            if (hdr_valid) begin
                last_chan <= rx_chan;
                last_len  <= hdr_len;
                if (hdr_cont && hdr_len > 4) cont_drop_count <= sat16(cont_drop_count);
            end
            if (rx_trunc)     trunc_count      <= sat16(trunc_count);
            if (p_unknown_id) unknown_id_count <= sat16(unknown_id_count);
            if (fc_valid) begin
                case (fc_id)
                    ID_RAW_ACCEL: granted_accel_us <= fc_interval_us;
                    ID_RAW_GYRO:  granted_gyro_us  <= fc_interval_us;
                    ID_RAW_MAG:   granted_mag_us   <= fc_interval_us;
                    default: ;
                endcase
                if (fc_id == instr.id) fc_match <= 1'b1;
            end
            if (!intn_s && state != RST_HOLD) saw_intn <= 1'b1;
            if (p_reset_complete) begin
                reset_complete_count <= sat8(reset_complete_count);
                rc_seen              <= 1'b1;
                if (booted) restart  <= 1'b1;    // hub reset itself: features are gone
            end
            if (any_valid) fault <= 1'b0;        // a good report means healthy

            // per-sensor watchdogs, only while streaming
            if (state == STREAM) begin
                wdog_a <= accel_valid ? 0 : wdog_a + 1;
                wdog_g <= gyro_valid  ? 0 : wdog_g + 1;
                wdog_m <= mag_valid   ? 0 : wdog_m + 1;
                if (accel_valid) seen_a <= 1'b1;
                if (gyro_valid)  seen_g <= 1'b1;
                if (mag_valid)   seen_m <= 1'b1;
            end

            // ---------------- sequencer ----------------
            case (state)

                RST_HOLD: begin
                    rst_n <= 1'b0;
                    if (delay >= RESET_HOLD_CYCLES) begin
                        rst_n    <= 1'b1;        // PS0/WAKE is high (transport idle)
                        pc       <= '0;
                        delay    <= 0;
                        rc_seen  <= 1'b0;
                        booted   <= 1'b0;
                        restart  <= 1'b0;
                        saw_intn <= 1'b0;
                        state    <= FETCH;
                    end else
                        delay <= delay + 1;
                end

                FETCH:
                    if (restart) begin
                        // hub self-reset: rerun the microcode (rc_seen is already set)
                        restart   <= 1'b0;
                        booted    <= 1'b0;
                        init_done <= 1'b0;
                        pc        <= '0;
                        delay     <= 0;
                    end else begin
                        case (instr.op)
                            OP_WAIT_RESET_COMPLETE:
                                if (rc_seen) begin
                                    booted <= 1'b1;
                                    delay  <= 0;
                                    pc     <= pc + 1;
                                end else if (delay >= BOOT_TIMEOUT_CYCLES) begin
                                    fault       <= 1'b1;
                                    fault_cause <= saw_intn ? FC_BOOT_NO_RESET : FC_BOOT_NO_INTN;
                                    fault_pc    <= pc;
                                    if (reinit_count != 8'hFF) reinit_count <= reinit_count + 1;
                                    delay       <= 0;
                                    state       <= RST_HOLD;
                                end else
                                    delay <= delay + 1;

                            OP_SET_FEATURE, OP_PROD_ID: begin
                                fc_match <= 1'b0;
                                wr_start <= 1'b1;
                                state    <= WAIT_WR;
                            end

                            OP_EXPECT_FC: begin
                                delay <= 0;
                                state <= WAIT_FC;
                            end

                            OP_SETTLE:
                                if (delay >= SETTLE_CYCLES) begin
                                    delay <= 0;
                                    pc    <= pc + 1;
                                end else
                                    delay <= delay + 1;

                            default: begin               // OP_END
                                init_done <= 1'b1;
                                wdog_a    <= 0;
                                wdog_g    <= 0;
                                wdog_m    <= 0;
                                seen_a    <= 1'b0;
                                seen_g    <= 1'b0;
                                seen_m    <= 1'b0;
                                state     <= STREAM;
                            end
                        endcase
                    end

                WAIT_WR:
                    if (wr_done) begin
                        if (wr_err) begin
                            fault       <= 1'b1;
                            fault_cause <= FC_WAKE_TIMEOUT;
                            fault_pc    <= pc;
                            if (reinit_count != 8'hFF) reinit_count <= reinit_count + 1;
                            init_done   <= 1'b0;
                            delay       <= 0;
                            state       <= RST_HOLD;
                        end else begin
                            pc    <= pc + 1;
                            state <= FETCH;
                        end
                    end

                WAIT_FC:
                    if (fc_match || restart) begin
                        pc    <= pc + 1;
                        state <= FETCH;
                    end else if (delay >= FC_TIMEOUT_CYCLES) begin
                        fault       <= 1'b1;             // not fatal: carry on
                        fault_cause <= FC_FC_TIMEOUT;
                        fault_pc    <= pc;
                        pc          <= pc + 1;
                        state       <= FETCH;
                    end else
                        delay <= delay + 1;

                // Reports are read by the transport whenever INTN asks.
                STREAM:
                    if (restart) begin
                        init_done <= 1'b0;
                        state     <= FETCH;              // FETCH restarts at pc 0
                    end else if ((ACCEL_INTERVAL_US != 0 && wdog_a >= lim_a) ||
                                 (GYRO_INTERVAL_US  != 0 && wdog_g >= lim_g) ||
                                 (MAG_INTERVAL_US   != 0 && wdog_m >= lim_m)) begin
                        fault       <= 1'b1;
                        fault_cause <= (ACCEL_INTERVAL_US != 0 && wdog_a >= lim_a) ? FC_WDOG_ACCEL :
                                       (GYRO_INTERVAL_US  != 0 && wdog_g >= lim_g) ? FC_WDOG_GYRO  :
                                                                                                 FC_WDOG_MAG;
                        fault_pc    <= pc;
                        if (reinit_count != 8'hFF) reinit_count <= reinit_count + 1;
                        init_done   <= 1'b0;
                        delay       <= 0;
                        state       <= RST_HOLD;
                    end

                default: begin                           // illegal encoding: recover
                    fault       <= 1'b1;
                    fault_cause <= FC_ILLEGAL;
                    fault_pc    <= pc;
                    init_done   <= 1'b0;
                    delay       <= 0;
                    state       <= RST_HOLD;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Lower layers
    // ------------------------------------------------------------------------
    logic       spi_start, spi_hold_cs, spi_rx_valid;
    logic [7:0] spi_tx, spi_rx;
    /* verilator lint_off UNUSEDSIGNAL */
    logic       spi_busy;          // not needed: the transport waits on rx_valid
    logic       hdr_in_write_unused, short_unused;
    /* verilator lint_on UNUSEDSIGNAL */
    assign hdr_in_write_unused = hdr_in_write;
    assign short_unused        = p_short_report;

    bno085_shtp #(
        .CS_GAP_CYCLES(CS_GAP_CYCLES),
        .INTN_STALE_CYCLES(INTN_STALE_CYCLES),
        .WAKE_TIMEOUT_CYCLES(WAKE_TIMEOUT_CYCLES),
        .MAX_RX_LEN(MAX_RX_LEN),
        .WRITE_RX_DELIVER(WRITE_RX_DELIVER)
    ) u_shtp (
        .clk, .rst(xrst),
        .intn_s, .wake_n(ps0_wake_n),
        .wr_start, .wr_chan(CHAN_CONTROL), .wr_len(instr.op == OP_PROD_ID ? PROD_ID_LEN : SET_FEATURE_LEN), .wr_idx, .wr_byte,
        .wr_done, .wr_err,
        .rx_valid, .rx_data, .rx_sop, .rx_eop, .rx_chan, .rx_cont,
        .hdr_valid, .hdr_len, .hdr_cont, .hdr_in_write, .rx_trunc,
        .spi_start, .spi_tx, .spi_hold_cs, .spi_rx, .spi_rx_valid, .spi_cs_n(cs_n)
    );

    bno085_parser u_parser (
        .clk, .rst(xrst),
        .rx_valid, .rx_data, .rx_sop, .rx_eop, .rx_chan, .rx_cont,
        .reset_complete(p_reset_complete), .unknown_id(p_unknown_id), .short_report(p_short_report),
        .accel_valid, .accel_x, .accel_y, .accel_z, .accel_ts, .accel_status, .accel_seq,
        .gyro_valid, .gyro_x, .gyro_y, .gyro_z, .gyro_temp, .gyro_ts, .gyro_status, .gyro_seq,
        .mag_valid, .mag_x, .mag_y, .mag_z, .mag_ts, .mag_status, .mag_seq,
        .fc_valid, .fc_id, .fc_interval_us
    );

    spimaster #(.CLOCK_DIVIDE(CLK_DIV), .CPOL(1), .CPHA(1)) u_spi (
        .sys_clk(clk), .rst(xrst),
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold_cs),
        .busy(spi_busy), .rx_data(spi_rx), .rx_valid(spi_rx_valid),
        .cs_n, .sclk, .mosi, .miso(miso_s)
    );

endmodule
