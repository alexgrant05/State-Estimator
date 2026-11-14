// ============================================================================
// Sensor test bench - FPGA top level (Arty S7-25)
//
//   ADXL375 (JC) <--SPI--> [adxl375_ctrl     ] -> [adxl375_rig_source slot 0] --+
//   BNO085  (JB) <--SPI--> [bno085_ctrl      ] -> [bno085_rig_source  slot 1] --+--> [rig_link_arb] -> [rig_link_tx] --SPI--> Pico 2 W (JD) --USB--> PC
//   BMP581  (JA) <--SPI--> [bmp581_controller] -> [bmp581_rig_source  slot 2] --+
//
// Every driver runs on its own SPI bus, independently. Each one has a
// source that turns its samples and status into v2 frames for its slot; the
// arbiter takes frames from the sources in turn and rig_link_tx sends them
// to the Pico (docs/rig_link_protocol.md). The Pico firmware (rig_monitor)
// shows every slot.
//
// Adding a sensor (up to 4 for the bench, 16 for the protocol):
//   1. instantiate its controller and a *_rig_source with the next SRC slot,
//   2. connect the source to the next arbiter port and raise N_SRC,
//   3. add its pins here and in constraints/sensor_bench.xdc (JD3, JD7-10
//      are the only free Pmod pins left).
//
// LEDs (LD2..LD5 = led[0..3]):
//   [0] ADXL375 streaming   [1] BNO085 streaming   [2] BMP581 streaming
//   [3] a sensor has a fault now, or has needed a recovery since reset
// ============================================================================

module sensor_bench_top #(
    parameter int         CLK_HZ           = 100_000_000,
    parameter bit         RST_ACTIVE_LOW   = 1'b0,
    parameter logic [3:0] ADXL_SRC         = 4'd0,     // link slots shown by the Pico
    parameter logic [3:0] BNO_SRC          = 4'd1,
    parameter logic [3:0] BMP_SRC          = 4'd2,

    // Pico link (shared by all sensors)
    parameter int LINK_CLK_DIV     = CLK_HZ / 4_000_000 * 2, // ~2 MHz SCLK, kept even
    parameter int LINK_GAP_CYCLES  = CLK_HZ / 500_000,       // 2 us CS high between frames
    parameter int HEARTBEAT_CYCLES = CLK_HZ / 10,            // STATUS per sensor at least every 100 ms

    // ADXL375: passed straight to adxl375_ctrl (defaults = real-time values).
    // Full rate: 3200 Hz (BW_RATE 0x0F), one sample every 312.5 us. That needs
    // SCLK >= 2 MHz (2.5 MHz here; a 6-byte read takes ~25 us), the INT1
    // re-arm wait well under a sample period, and a tighter INT1 watchdog.
    parameter logic [7:0] ACC_BW_RATE    = 8'h0F,                     // 3200 Hz
    parameter int ACC_CLK_DIV            = CLK_HZ / 5_000_000 * 2,    // 2.5 MHz
    parameter int ACC_PWR_UP_CYCLES      = CLK_HZ / 50,
    parameter int ACC_RETRY_CYCLES       = CLK_HZ / 100,
    parameter int ACC_MAX_ATTEMPTS       = 10,
    parameter int ACC_REINIT_CYCLES      = CLK_HZ / 2,
    parameter int ACC_CS_GAP_CYCLES      = CLK_HZ / 2_000_000,
    parameter int ACC_HOLDOFF_CYCLES     = CLK_HZ / 200_000,          // 5 us
    parameter int ACC_REARM_CYCLES       = CLK_HZ / 10_000,           // 100 us (period 312.5 us)
    parameter int ACC_INT_TIMEOUT_CYCLES = CLK_HZ / 100,              // 10 ms = 32 samples

    // BNO085: passed straight to bno085_ctrl (defaults = real-time values)
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
    parameter int BNO_MAG_WDOG_CYCLES     = BNO_WDOG_PERIODS * (CLK_HZ / 1_000_000) * BNO_MAG_INTERVAL_US,

    // BMP581: passed straight to bmp581_controller (defaults = real-time values)
    parameter int BMP_CLK_DIV              = CLK_HZ / 2_000_000 * 2,  // ~1 MHz
    parameter int unsigned BMP_PWRUP_WAIT_CYC   = CLK_HZ / 1000 * 3,
    parameter int unsigned BMP_SOFTRST_WAIT_CYC = CLK_HZ / 1000 * 3,
    parameter int unsigned BMP_STANDBY_WAIT_CYC = CLK_HZ / 1000 * 3,
    parameter int unsigned BMP_CS_IDLE_CYC      = CLK_HZ / 1_000_000 * 2,
    parameter int unsigned BMP_DRDY_TIMEOUT_CYC = CLK_HZ / 1000 * 100,
    parameter int unsigned BMP_RETRY_WAIT_CYC   = CLK_HZ / 1000 * 100,
    parameter int unsigned BMP_STALE_CYC        = CLK_HZ / 1000 * 10
) (
    input  logic       clk,
    input  logic       rst_btn,

    // ADXL375 on JC
    output logic       acc_cs_n,
    output logic       acc_sclk,
    output logic       acc_mosi,
    input  logic       acc_miso,
    input  logic       acc_int1,

    // BNO085 on JB (PS1 and BOOTN high on the board)
    output logic       bno_cs_n,
    output logic       bno_sclk,
    output logic       bno_mosi,
    input  logic       bno_miso,
    input  logic       bno_intn,
    output logic       bno_wake_n,      // PS0/WAKE
    output logic       bno_rst_n,

    // BMP581 on JA
    output logic       bmp_cs_n,
    output logic       bmp_sclk,
    output logic       bmp_mosi,        // to sensor SDI
    input  logic       bmp_miso,        // from sensor SDO
    input  logic       bmp_int,

    // Pico 2 W on JD (SPI0 slave: GP17 CSn, GP18 SCK, GP16 RX)
    output logic       pico_cs_n,
    output logic       pico_sclk,
    output logic       pico_mosi,

    output logic [3:0] led
);

    localparam int N_SRC = 3;           // arbiter ports in use

    // ------------------------------------------------------------------------
    // Parameter checks
    // ------------------------------------------------------------------------
    // The RP2350's PL022 needs clk_peri >= 12 x SCLK in slave mode
    // (150 MHz / 12 = 12.5 MHz). Stay well under it for jumper wires.
    if (LINK_CLK_DIV < 2 || LINK_CLK_DIV % 2 != 0) begin : g_chk_link_div
        $error("sensor_bench_top: LINK_CLK_DIV (%0d) must be even and >= 2", LINK_CLK_DIV);
    end
    if (CLK_HZ / LINK_CLK_DIV > 10_000_000) begin : g_chk_link_fsclk
        $error("sensor_bench_top: link SCLK %0d Hz is above 10 MHz", CLK_HZ / LINK_CLK_DIV);
    end
    if (ADXL_SRC == BNO_SRC || ADXL_SRC == BMP_SRC || BNO_SRC == BMP_SRC) begin : g_chk_src
        $error("sensor_bench_top: ADXL_SRC, BNO_SRC and BMP_SRC must all differ");
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

    // Arbiter ports: index 0 = ADXL375, 1 = BNO085, 2 = BMP581
    logic [N_SRC-1:0]       a_start, a_done;
    logic [N_SRC-1:0][3:0]  a_src;
    logic [N_SRC-1:0][7:0]  a_type, a_byte;
    logic [N_SRC-1:0][5:0]  a_len;
    logic [5:0]             f_idx;

    // ------------------------------------------------------------------------
    // ADXL375 (slot ADXL_SRC)
    // ------------------------------------------------------------------------
    logic               acc_sample_valid, acc_init_done, acc_fault, acc_fault_cause;
    logic signed [15:0] acc_x, acc_y, acc_z;
    logic [3:0]         acc_fault_pc;
    logic [7:0]         acc_fault_rdata, acc_reinit_count, acc_link_drops;

    adxl375_ctrl #(
        .CLK_HZ(CLK_HZ),
        .CLK_DIV(ACC_CLK_DIV),
        .PWR_UP_CYCLES(ACC_PWR_UP_CYCLES),
        .RETRY_CYCLES(ACC_RETRY_CYCLES),
        .MAX_ATTEMPTS(ACC_MAX_ATTEMPTS),
        .REINIT_CYCLES(ACC_REINIT_CYCLES),
        .CS_GAP_CYCLES(ACC_CS_GAP_CYCLES),
        .HOLDOFF_CYCLES(ACC_HOLDOFF_CYCLES),
        .REARM_CYCLES(ACC_REARM_CYCLES),
        .INT_TIMEOUT_CYCLES(ACC_INT_TIMEOUT_CYCLES),
        .BW_RATE(ACC_BW_RATE)
    ) u_acc (
        .clk, .rst(rst_i),
        .cs_n(acc_cs_n), .sclk(acc_sclk), .mosi(acc_mosi), .miso(acc_miso), .int1(acc_int1),
        .sample_valid(acc_sample_valid), .accel_x(acc_x), .accel_y(acc_y), .accel_z(acc_z),
        .init_done(acc_init_done), .fault(acc_fault), .fault_cause(acc_fault_cause),
        .fault_pc(acc_fault_pc), .fault_rdata(acc_fault_rdata), .reinit_count(acc_reinit_count)
    );

    // INT1 level for reporting only (the controller has its own synchronizer)
    (* ASYNC_REG = "TRUE" *) logic int1_s1, int1_s2;
    always_ff @(posedge clk) begin
        int1_s1 <= acc_int1;
        int1_s2 <= int1_s1;
    end

    adxl375_rig_source #(.SRC(ADXL_SRC), .HEARTBEAT_CYCLES(HEARTBEAT_CYCLES)) u_acc_src (
        .clk, .rst(rst_i),
        .sample_valid(acc_sample_valid), .accel_x(acc_x), .accel_y(acc_y), .accel_z(acc_z),
        .init_done(acc_init_done), .fault(acc_fault), .fault_cause(acc_fault_cause),
        .fault_pc(acc_fault_pc), .fault_rdata(acc_fault_rdata), .reinit_count(acc_reinit_count),
        .int1_level(int1_s2),
        .frame_start(a_start[0]), .frame_src(a_src[0]), .frame_type(a_type[0]), .frame_len(a_len[0]),
        .frame_byte_idx(f_idx), .frame_byte(a_byte[0]), .frame_done(a_done[0]),
        .link_drops(acc_link_drops)
    );

    // ------------------------------------------------------------------------
    // BNO085 (slot BNO_SRC)
    // ------------------------------------------------------------------------
    logic               bno_accel_valid, bno_gyro_valid, bno_mag_valid;
    logic signed [15:0] bno_accel_x, bno_accel_y, bno_accel_z;
    logic signed [15:0] bno_gyro_x, bno_gyro_y, bno_gyro_z, bno_gyro_temp;
    logic signed [15:0] bno_mag_x, bno_mag_y, bno_mag_z;
    logic [31:0]        bno_accel_ts, bno_gyro_ts, bno_mag_ts;
    logic [7:0]         bno_accel_status, bno_gyro_status, bno_mag_status;
    logic [7:0]         bno_accel_seq, bno_gyro_seq, bno_mag_seq;
    logic               bno_init_done, bno_fault;
    logic [3:0]         bno_fault_cause, bno_fault_pc;
    logic [7:0]         bno_reinit_count, bno_rc_count, bno_last_chan, bno_link_drops;
    logic [15:0]        bno_unknown_count, bno_cont_count, bno_trunc_count;
    logic [14:0]        bno_last_len;
    logic [31:0]        bno_granted_accel, bno_granted_gyro, bno_granted_mag;

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
        .accel_valid(bno_accel_valid), .accel_x(bno_accel_x), .accel_y(bno_accel_y), .accel_z(bno_accel_z),
        .accel_ts(bno_accel_ts), .accel_status(bno_accel_status), .accel_seq(bno_accel_seq),
        .gyro_valid(bno_gyro_valid), .gyro_x(bno_gyro_x), .gyro_y(bno_gyro_y), .gyro_z(bno_gyro_z),
        .gyro_temp(bno_gyro_temp), .gyro_ts(bno_gyro_ts), .gyro_status(bno_gyro_status), .gyro_seq(bno_gyro_seq),
        .mag_valid(bno_mag_valid), .mag_x(bno_mag_x), .mag_y(bno_mag_y), .mag_z(bno_mag_z),
        .mag_ts(bno_mag_ts), .mag_status(bno_mag_status), .mag_seq(bno_mag_seq),
        .init_done(bno_init_done), .fault(bno_fault), .fault_cause(bno_fault_cause), .fault_pc(bno_fault_pc),
        .reinit_count(bno_reinit_count), .reset_complete_count(bno_rc_count),
        .unknown_id_count(bno_unknown_count), .cont_drop_count(bno_cont_count), .trunc_count(bno_trunc_count),
        .last_chan(bno_last_chan), .last_len(bno_last_len),
        .granted_accel_us(bno_granted_accel), .granted_gyro_us(bno_granted_gyro), .granted_mag_us(bno_granted_mag)
    );

    // INTN level for reporting only (the controller has its own synchronizer)
    (* ASYNC_REG = "TRUE" *) logic intn_s1 = 1'b1, intn_s2 = 1'b1;
    always_ff @(posedge clk) begin
        intn_s1 <= bno_intn;
        intn_s2 <= intn_s1;
    end

    bno085_rig_source #(.SRC(BNO_SRC), .HEARTBEAT_CYCLES(HEARTBEAT_CYCLES)) u_bno_src (
        .clk, .rst(rst_i),
        .accel_valid(bno_accel_valid), .accel_x(bno_accel_x), .accel_y(bno_accel_y), .accel_z(bno_accel_z),
        .accel_ts(bno_accel_ts), .accel_status(bno_accel_status), .accel_seq(bno_accel_seq),
        .gyro_valid(bno_gyro_valid), .gyro_x(bno_gyro_x), .gyro_y(bno_gyro_y), .gyro_z(bno_gyro_z),
        .gyro_temp(bno_gyro_temp), .gyro_ts(bno_gyro_ts), .gyro_status(bno_gyro_status), .gyro_seq(bno_gyro_seq),
        .mag_valid(bno_mag_valid), .mag_x(bno_mag_x), .mag_y(bno_mag_y), .mag_z(bno_mag_z),
        .mag_ts(bno_mag_ts), .mag_status(bno_mag_status), .mag_seq(bno_mag_seq),
        .init_done(bno_init_done), .fault(bno_fault), .fault_cause(bno_fault_cause), .fault_pc(bno_fault_pc),
        .reinit_count(bno_reinit_count), .reset_complete_count(bno_rc_count),
        .unknown_id_count(bno_unknown_count), .cont_drop_count(bno_cont_count), .trunc_count(bno_trunc_count),
        .last_chan(bno_last_chan), .last_len(bno_last_len),
        .granted_accel_us(bno_granted_accel), .granted_gyro_us(bno_granted_gyro), .granted_mag_us(bno_granted_mag),
        .intn_level(intn_s2), .in_reset(!bno_rst_n),
        .frame_start(a_start[1]), .frame_src(a_src[1]), .frame_type(a_type[1]), .frame_len(a_len[1]),
        .frame_byte_idx(f_idx), .frame_byte(a_byte[1]), .frame_done(a_done[1]),
        .link_drops(bno_link_drops)
    );

    // ------------------------------------------------------------------------
    // BMP581 (slot BMP_SRC)
    // bmp581_controller takes an ACTIVE-LOW reset (its own convention); this
    // repo's spimaster takes an active-high one, like everything else here.
    // ------------------------------------------------------------------------
    logic [63:0] ticks;                 // fabric time base, timestamps BMP581 samples
    always_ff @(posedge clk) begin
        if (rst_i) ticks <= '0;
        else       ticks <= ticks + 1'b1;
    end

    logic       bmp_spi_start, bmp_spi_hold_cs, bmp_spi_busy, bmp_spi_rx_valid;
    logic [7:0] bmp_spi_tx, bmp_spi_rx;

    spimaster #(.CLOCK_DIVIDE(BMP_CLK_DIV), .CPOL(1), .CPHA(1)) u_bmp_spi (
        .sys_clk(clk), .rst(rst_i),
        .start(bmp_spi_start), .tx_data(bmp_spi_tx), .hold_cs(bmp_spi_hold_cs),
        .busy(bmp_spi_busy), .rx_data(bmp_spi_rx), .rx_valid(bmp_spi_rx_valid),
        .cs_n(bmp_cs_n), .sclk(bmp_sclk), .mosi(bmp_mosi), .miso(bmp_miso)
    );

    logic        bmp_sample_valid, bmp_init_done, bmp_fault;
    logic [7:0]  bmp_sensor_id, bmp_flags, bmp_fault_count, bmp_link_drops;
    logic [15:0] bmp_seq, bmp_drdy_to_count, bmp_overrun_count;
    logic [63:0] bmp_ts;
    logic [23:0] bmp_press_raw, bmp_temp_raw;
    logic [3:0]  bmp_fault_code;
    /* verilator lint_off UNUSEDSIGNAL */
    logic        bmp_bus_req;           // only device on its spimaster
    /* verilator lint_on UNUSEDSIGNAL */

    bmp581_controller #(
        .CLK_HZ(CLK_HZ),
        .SENSOR_ID(8'(BMP_SRC)),
        .PWRUP_WAIT_CYC(BMP_PWRUP_WAIT_CYC), .SOFTRST_WAIT_CYC(BMP_SOFTRST_WAIT_CYC),
        .STANDBY_WAIT_CYC(BMP_STANDBY_WAIT_CYC), .CS_IDLE_CYC(BMP_CS_IDLE_CYC),
        .DRDY_TIMEOUT_CYC(BMP_DRDY_TIMEOUT_CYC), .RETRY_WAIT_CYC(BMP_RETRY_WAIT_CYC),
        .STALE_CYC(BMP_STALE_CYC)
    ) u_bmp (
        .sys_clk(clk), .rst(!rst_i),
        .timestamp_now(ticks),
        .bmp_int(bmp_int),
        .bus_req(bmp_bus_req), .bus_gnt(1'b1),
        .spi_start(bmp_spi_start), .spi_tx_data(bmp_spi_tx), .spi_hold_cs(bmp_spi_hold_cs),
        .spi_busy(bmp_spi_busy), .spi_rx_data(bmp_spi_rx), .spi_rx_valid(bmp_spi_rx_valid),
        .sample_valid(bmp_sample_valid), .sample_sensor_id(bmp_sensor_id), .sample_seq(bmp_seq),
        .sample_timestamp(bmp_ts), .sample_press_raw(bmp_press_raw),
        .sample_temp_raw(bmp_temp_raw), .sample_flags(bmp_flags),
        .init_done(bmp_init_done), .fault(bmp_fault), .fault_code(bmp_fault_code),
        .fault_count(bmp_fault_count),
        .drdy_timeout_count(bmp_drdy_to_count), .overrun_count(bmp_overrun_count)
    );

    // INT level for reporting only (the controller has its own synchronizer)
    (* ASYNC_REG = "TRUE" *) logic bint_s1, bint_s2;
    always_ff @(posedge clk) begin
        bint_s1 <= bmp_int;
        bint_s2 <= bint_s1;
    end

    bmp581_rig_source #(.SRC(BMP_SRC), .HEARTBEAT_CYCLES(HEARTBEAT_CYCLES)) u_bmp_src (
        .clk, .rst(rst_i),
        .sample_valid(bmp_sample_valid), .sample_seq(bmp_seq), .sample_ts(bmp_ts[31:0]),
        .sample_press_raw(bmp_press_raw), .sample_temp_raw(bmp_temp_raw), .sample_flags(bmp_flags),
        .init_done(bmp_init_done), .fault(bmp_fault), .fault_code(bmp_fault_code),
        .fault_count(bmp_fault_count), .drdy_timeout_count(bmp_drdy_to_count),
        .overrun_count(bmp_overrun_count), .int_level(bint_s2),
        .frame_start(a_start[2]), .frame_src(a_src[2]), .frame_type(a_type[2]), .frame_len(a_len[2]),
        .frame_byte_idx(f_idx), .frame_byte(a_byte[2]), .frame_done(a_done[2]),
        .link_drops(bmp_link_drops)
    );

    // ------------------------------------------------------------------------
    // Shared link to the Pico
    // ------------------------------------------------------------------------
    logic       f_start, f_done;
    logic [3:0] f_src;
    logic [7:0] f_type, f_byte;
    logic [5:0] f_len;
    /* verilator lint_off UNUSEDSIGNAL */
    logic       f_busy;
    /* verilator lint_on UNUSEDSIGNAL */

    rig_link_arb #(.N(N_SRC)) u_arb (
        .clk, .rst(rst_i),
        .req_start(a_start), .req_src(a_src), .req_type(a_type), .req_len(a_len),
        .req_byte(a_byte), .req_done(a_done),
        .tx_start(f_start), .tx_src(f_src), .tx_type(f_type), .tx_len(f_len),
        .tx_byte(f_byte), .tx_done(f_done)
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
    assign led = {acc_fault || bno_fault || bmp_fault ||
                  acc_reinit_count != 8'd0 || bno_reinit_count != 8'd0 || bno_rc_count > 8'd1 ||
                  bmp_fault_count != 8'd0,
                  bmp_init_done,
                  bno_init_done,
                  acc_init_done};

endmodule
