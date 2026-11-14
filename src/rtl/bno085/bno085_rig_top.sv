// ============================================================================
// BNO085 test rig - FPGA top level
//
//   BNO085 <--SPI--> [bno085_ctrl (spimaster #1)]                FPGA = SPI master
//                              |
//                     [bno085_rig_source] --> [rig_link_tx (spimaster #2)] --SPI--> Pico 2 W
//                                                                               --USB--> PC
//
// The FPGA pushes one v2 frame to the Pico (docs/rig_link_protocol.md):
//   * for every raw accel / gyro / mag report (types 0x14 / 0x15 / 0x16)
//   * a STATUS frame immediately when the controller's status changes
//   * a STATUS frame at least every HEARTBEAT_CYCLES, so the PC sees the
//     state during boot, re-init and faults
// This rig is source slot SRC (default 1; slot 0 is left for the ADXL375 in
// the combined rig). Frames carry the slot, so several drivers can share one
// Pico link later through an arbiter.
//
// Link: mode 3 (CPOL=1, CPHA=1), MSB first, CS low for the whole frame (the
// Pico's PL022 slave needs CPHA=1 for back-to-back bytes). One-way.
//
// Default timing assumes CLK_HZ; override the BNO_* and link cycle counts
// only for simulation.
// ============================================================================

module bno085_rig_top #(
    parameter int         CLK_HZ           = 100_000_000,
    parameter bit         RST_ACTIVE_LOW   = 1'b0,        // set to 1 if the board's reset button is active low
    parameter logic [3:0] SRC              = 4'd1,        // link source slot

    // Pico link
    parameter int LINK_CLK_DIV     = CLK_HZ / 4_000_000 * 2, // ~2 MHz SCLK, kept even
    parameter int LINK_GAP_CYCLES  = CLK_HZ / 500_000,       // 2 us CS high between frames
    parameter int HEARTBEAT_CYCLES = CLK_HZ / 10,            // STATUS at least every 100 ms

    // passed straight to bno085_ctrl (defaults = real-time values)
    parameter int BNO_CLK_DIV             = CLK_HZ / 2_000_000 * 2,   // ~1 MHz
    parameter int BNO_ACCEL_INTERVAL_US   = 2_000,
    parameter int BNO_GYRO_INTERVAL_US    = 2_500,
    parameter int BNO_MAG_INTERVAL_US     = 10_000,
    parameter int BNO_RESET_HOLD_CYCLES   = CLK_HZ / 100,
    parameter int BNO_BOOT_TIMEOUT_CYCLES = CLK_HZ * 2,
    parameter int BNO_CS_GAP_CYCLES       = CLK_HZ / 500_000,
    parameter int BNO_WAKE_TIMEOUT_CYCLES = CLK_HZ / 100,
    parameter int BNO_INTN_STALE_CYCLES   = CLK_HZ / 100,
    parameter int BNO_SETTLE_CYCLES       = CLK_HZ / 4,
    parameter int BNO_WDOG_PERIODS        = 10,
    parameter int BNO_ACCEL_WDOG_CYCLES   = BNO_WDOG_PERIODS * (CLK_HZ / 1_000_000) * BNO_ACCEL_INTERVAL_US,
    parameter int BNO_GYRO_WDOG_CYCLES    = BNO_WDOG_PERIODS * (CLK_HZ / 1_000_000) * BNO_GYRO_INTERVAL_US,
    parameter int BNO_MAG_WDOG_CYCLES     = BNO_WDOG_PERIODS * (CLK_HZ / 1_000_000) * BNO_MAG_INTERVAL_US
) (
    input  logic       clk,
    input  logic       rst_btn,

    // BNO085 (PS1 and BOOTN strapped high on the board)
    output logic       bno_cs_n,
    output logic       bno_sclk,
    output logic       bno_mosi,
    input  logic       bno_miso,
    input  logic       bno_intn,
    output logic       bno_wake_n,      // PS0/WAKE
    output logic       bno_rst_n,

    // Pico 2 W (SPI0 slave: GP17 CSn, GP18 SCK, GP16 RX)
    output logic       pico_cs_n,
    output logic       pico_sclk,
    output logic       pico_mosi,

    // board LEDs: [0] streaming  [1] fault  [2] ~1 Hz blink while reports flow
    //             [3] the sensor has needed a recovery (RSTN re-init or hub self-reset)
    output logic [3:0] led
);

    // ------------------------------------------------------------------------
    // Parameter checks
    // ------------------------------------------------------------------------
    // The RP2350's PL022 needs clk_peri >= 12 x SCLK in slave mode
    // (150 MHz / 12 = 12.5 MHz). Stay well under it for jumper wires.
    if (LINK_CLK_DIV < 2 || LINK_CLK_DIV % 2 != 0) begin : g_chk_link_div
        $error("bno085_rig_top: LINK_CLK_DIV (%0d) must be even and >= 2", LINK_CLK_DIV);
    end
    if (CLK_HZ / LINK_CLK_DIV > 10_000_000) begin : g_chk_link_fsclk
        $error("bno085_rig_top: link SCLK %0d Hz is above 10 MHz", CLK_HZ / LINK_CLK_DIV);
    end

    // ------------------------------------------------------------------------
    // Reset: synchronize the button, plus a power-on reset from FPGA init values
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [1:0] rst_meta = 2'b11;
    logic [7:0] por_sr = 8'hFF;
    logic       rst_i  = 1'b1;

    always_ff @(posedge clk) begin
        rst_meta <= {rst_meta[0], rst_btn ^ RST_ACTIVE_LOW};
        por_sr   <= {por_sr[6:0], 1'b0};
        rst_i    <= rst_meta[1] | por_sr[7];
    end

    // ------------------------------------------------------------------------
    // Sensor controller (contains spimaster #1)
    // ------------------------------------------------------------------------
    logic               accel_valid, gyro_valid, mag_valid;
    logic signed [15:0] accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, gyro_temp, mag_x, mag_y, mag_z;
    logic [31:0]        accel_ts, gyro_ts, mag_ts;
    logic [7:0]         accel_status, gyro_status, mag_status, accel_seq, gyro_seq, mag_seq;
    logic               init_done, fault;
    logic [3:0]         fault_cause, fault_pc;
    logic [7:0]         reinit_count, reset_complete_count, last_chan;
    logic [15:0]        unknown_id_count, cont_drop_count, trunc_count;
    logic [14:0]        last_len;
    logic [31:0]        granted_accel_us, granted_gyro_us, granted_mag_us;

    bno085_ctrl #(
        .CLK_HZ(CLK_HZ),
        .CLK_DIV(BNO_CLK_DIV),
        .ACCEL_INTERVAL_US(BNO_ACCEL_INTERVAL_US),
        .GYRO_INTERVAL_US(BNO_GYRO_INTERVAL_US),
        .MAG_INTERVAL_US(BNO_MAG_INTERVAL_US),
        .RESET_HOLD_CYCLES(BNO_RESET_HOLD_CYCLES),
        .BOOT_TIMEOUT_CYCLES(BNO_BOOT_TIMEOUT_CYCLES),
        .CS_GAP_CYCLES(BNO_CS_GAP_CYCLES),
        .WAKE_TIMEOUT_CYCLES(BNO_WAKE_TIMEOUT_CYCLES),
        .INTN_STALE_CYCLES(BNO_INTN_STALE_CYCLES),
        .SETTLE_CYCLES(BNO_SETTLE_CYCLES),
        .WDOG_PERIODS(BNO_WDOG_PERIODS),
        .ACCEL_WDOG_CYCLES(BNO_ACCEL_WDOG_CYCLES),
        .GYRO_WDOG_CYCLES(BNO_GYRO_WDOG_CYCLES),
        .MAG_WDOG_CYCLES(BNO_MAG_WDOG_CYCLES)
    ) u_bno (
        .clk, .rst(rst_i),
        .cs_n(bno_cs_n), .sclk(bno_sclk), .mosi(bno_mosi), .miso(bno_miso),
        .intn(bno_intn), .ps0_wake_n(bno_wake_n), .rst_n(bno_rst_n),
        .accel_valid, .accel_x, .accel_y, .accel_z, .accel_ts, .accel_status, .accel_seq,
        .gyro_valid, .gyro_x, .gyro_y, .gyro_z, .gyro_temp, .gyro_ts, .gyro_status, .gyro_seq,
        .mag_valid, .mag_x, .mag_y, .mag_z, .mag_ts, .mag_status, .mag_seq,
        .init_done, .fault, .fault_cause, .fault_pc, .reinit_count, .reset_complete_count,
        .unknown_id_count, .cont_drop_count, .trunc_count, .last_chan, .last_len,
        .granted_accel_us, .granted_gyro_us, .granted_mag_us
    );

    // INTN level for reporting only (the controller has its own synchronizer)
    (* ASYNC_REG = "TRUE" *) logic intn_s1 = 1'b1, intn_s2 = 1'b1;
    always_ff @(posedge clk) begin
        intn_s1 <= bno_intn;
        intn_s2 <= intn_s1;
    end

    // ------------------------------------------------------------------------
    // Frames to the Pico
    // ------------------------------------------------------------------------
    logic       f_start, f_done, f_busy;
    logic [3:0] f_src;
    logic [7:0] f_type, f_byte;
    logic [5:0] f_len, f_idx;
    logic [7:0] link_drops;

    bno085_rig_source #(.SRC(SRC), .HEARTBEAT_CYCLES(HEARTBEAT_CYCLES)) u_src (
        .clk, .rst(rst_i),
        .accel_valid, .accel_x, .accel_y, .accel_z, .accel_ts, .accel_status, .accel_seq,
        .gyro_valid, .gyro_x, .gyro_y, .gyro_z, .gyro_temp, .gyro_ts, .gyro_status, .gyro_seq,
        .mag_valid, .mag_x, .mag_y, .mag_z, .mag_ts, .mag_status, .mag_seq,
        .init_done, .fault, .fault_cause, .fault_pc, .reinit_count, .reset_complete_count,
        .unknown_id_count, .cont_drop_count, .trunc_count, .last_chan, .last_len,
        .granted_accel_us, .granted_gyro_us, .granted_mag_us,
        .intn_level(intn_s2), .in_reset(!bno_rst_n),
        .frame_start(f_start), .frame_src(f_src), .frame_type(f_type), .frame_len(f_len),
        .frame_byte_idx(f_idx), .frame_byte(f_byte), .frame_done(f_done),
        .link_drops
    );

    rig_link_tx #(.CLK_DIV(LINK_CLK_DIV), .GAP_CYCLES(LINK_GAP_CYCLES)) u_link (
        .clk, .rst(rst_i),
        .start(f_start), .src(f_src), .type_id(f_type), .len(f_len),
        .byte_idx(f_idx), .byte_in(f_byte), .busy(f_busy), .done(f_done),
        .cs_n(pico_cs_n), .sclk(pico_sclk), .mosi(pico_mosi)
    );

    // ------------------------------------------------------------------------
    // LEDs: useful even with the Pico unplugged
    // ------------------------------------------------------------------------
    logic [9:0] blink_cnt;
    logic       blink;
    /* verilator lint_off UNUSEDSIGNAL */
    logic       f_busy_unused;
    /* verilator lint_on UNUSEDSIGNAL */
    assign f_busy_unused = f_busy;

    always_ff @(posedge clk) begin
        if (rst_i) begin
            blink_cnt <= '0;
            blink     <= 1'b0;
        end else if (accel_valid || gyro_valid || mag_valid) begin
            // ~1000 reports/s at the default rates -> toggles about twice a second
            if (blink_cnt == 10'd499) begin
                blink_cnt <= '0;
                blink     <= ~blink;
            end else
                blink_cnt <= blink_cnt + 1;
        end
    end

    assign led = {reinit_count != 8'd0 || reset_complete_count > 8'd1,
                  blink & init_done, fault, init_done};

endmodule
