`default_nettype none

// ============================================================================
// bmp581_controller
//
// BMP581 barometer controller for the CRT FPGA state estimation PL.
//
//   BMP581 pins <-> spimaster <-> bmp581_controller -> sample stream (FIFO)
//         INT pin ------------------------^  (DR line, NOT through the SPI master)
//
// Two cooperating FSMs:
//   1. Transaction engine: turns "read N bytes from addr" / "write 1 byte" into
//      a sequence of spimaster byte starts, holding CS across the burst.
//   2. Sequencer: bring-up checks, register configuration (read-modify-write
//      with readback verify), then the run loop (DRDY edge -> burst read ->
//      INT_STATUS read -> emit sample).
//
// Spimaster must be instantiated with CPOL=1, CPHA=1 (mode 3). The BMP581
// accepts mode 0 or mode 3 and picks by the SCK level when CSB falls; mode 3
// matches the rest of the bus.
//
// Register map, bit fields and timing: Bosch BMP581 datasheet (BST-BMP581-DS004).
// ============================================================================

module bmp581_controller #(
    parameter int          CLK_HZ           = 100_000_000,
    parameter logic [7:0]  SENSOR_ID        = 8'd2,

    // ---- measurement configuration (datasheet register codes) ----
    parameter logic [2:0]  OSR_P            = 3'b011,  // pressure x8   (0.30 Pa RMS)
    parameter logic [2:0]  OSR_T            = 3'b000,  // temperature x1
    parameter logic [4:0]  ODR_CODE         = 5'h0F,   // 50.056 Hz
    parameter logic [2:0]  IIR_P            = 3'b000,  // bypass (ESKF does the filtering)
    parameter logic [2:0]  IIR_T            = 3'b000,  // bypass

    // ---- timing, in sys_clk cycles ----
    parameter int unsigned PWRUP_WAIT_CYC   = CLK_HZ / 1000 * 3,       // t_powup   2 ms  -> 3 ms
    parameter int unsigned SOFTRST_WAIT_CYC = CLK_HZ / 1000 * 3,       // t_soft_res 2 ms -> 3 ms
    parameter int unsigned STANDBY_WAIT_CYC = CLK_HZ / 1000 * 3,       // mode change 2.5 ms max -> 3 ms
    parameter int unsigned CS_IDLE_CYC      = CLK_HZ / 1_000_000 * 2,  // CSB high between txns, 2 us (>= 1 us rule)
    parameter int unsigned DRDY_TIMEOUT_CYC = CLK_HZ / 1000 * 100,     // 100 ms, about 5 missed samples at 50 Hz
    parameter int unsigned RETRY_WAIT_CYC   = CLK_HZ / 1000 * 100,     // back-off before retrying a failed bring-up
    parameter int unsigned STALE_CYC        = CLK_HZ / 1000 * 10,      // ~half the ODR period: edge-to-read latency
                                                                       // above this means the data may be a newer sample

    parameter int          TS_W             = 64
)(
    input  wire              sys_clk,
    input  wire              rst,              // synchronous, active low (repo convention)

    // free-running fabric time base (authoritative 64-bit counter)
    input  wire [TS_W-1:0]   timestamp_now,

    // BMP581 INT pin, asynchronous to sys_clk
    input  wire              bmp_int,

    // shared-bus hook: tie bus_gnt = 1'b1 if this controller owns its spimaster.
    // An arbiter must not drop bus_gnt while bus_req is high.
    output logic             bus_req,
    input  wire              bus_gnt,

    // spimaster fabric side
    output logic             spi_start,
    output logic [7:0]       spi_tx_data,
    output logic             spi_hold_cs,
    input  wire              spi_busy,
    input  wire  [7:0]       spi_rx_data,
    input  wire              spi_rx_valid,

    // sample stream out: raw payload + timestamp + sensor ID + sequence
    output logic             sample_valid,     // 1-cycle pulse, fields stable until next pulse
    output logic [7:0]       sample_sensor_id,
    output logic [15:0]      sample_seq,
    output logic [TS_W-1:0]  sample_timestamp, // fabric time at DRDY rising edge (+2 cycle sync delay)
    output logic [23:0]      sample_press_raw, // Pa = raw / 64
    output logic [23:0]      sample_temp_raw,  // degC = signed(raw) / 65536
    output logic [7:0]       sample_flags,     // [0] press skipped  [1] temp skipped  [2] overrun
                                               // [3] drdy status bit not set  [4] stale  [7:5] reserved (0)

    // status
    output logic             init_done,
    output logic             fault,            // sticky until the next successful bring-up
    output logic [3:0]       fault_code,       // last fault, see FLT_* below
    output logic [7:0]       fault_count,
    output logic [15:0]      drdy_timeout_count,
    output logic [15:0]      overrun_count
);

    // ------------------------------------------------------------------------
    // BMP581 constants
    // ------------------------------------------------------------------------
    localparam logic [7:0] SPI_RD          = 8'h80;   // address bit 7 = 1 -> read
    localparam logic [7:0] REG_CHIP_ID     = 8'h01;
    localparam logic [7:0] REG_INT_CONFIG  = 8'h14;
    localparam logic [7:0] REG_INT_SOURCE  = 8'h15;
    localparam logic [7:0] REG_TEMP_XLSB   = 8'h1D;   // 0x1D..0x1F temp, 0x20..0x22 press
    localparam logic [7:0] REG_INT_STATUS  = 8'h27;   // clear on read
    localparam logic [7:0] REG_STATUS      = 8'h28;
    localparam logic [7:0] REG_DSP_CONFIG  = 8'h30;
    localparam logic [7:0] REG_DSP_IIR     = 8'h31;
    localparam logic [7:0] REG_OSR_CONFIG  = 8'h36;
    localparam logic [7:0] REG_ODR_CONFIG  = 8'h37;
    localparam logic [7:0] REG_OSR_EFF     = 8'h38;
    localparam logic [7:0] REG_CMD         = 8'h7E;

    localparam logic [7:0] CMD_SOFT_RESET  = 8'hB6;
    localparam logic [7:0] CHIP_ID_BMP581  = 8'h50;
    localparam logic [23:0] SKIPPED        = 24'h7F7F7F;

    // fault codes
    localparam logic [3:0] FLT_NONE        = 4'd0;
    localparam logic [3:0] FLT_CHIP_ID     = 4'd1;   // CHIP_ID != 0x50
    localparam logic [3:0] FLT_NVM         = 4'd2;   // STATUS.nvm_rdy = 0 or nvm_err = 1
    localparam logic [3:0] FLT_NO_POR      = 4'd3;   // INT_STATUS.por not set after soft reset
    localparam logic [3:0] FLT_CFG_BASE    = 4'd4;   // 4..10 = config step 0..6 readback mismatch
    localparam logic [3:0] FLT_OSR_ODR     = 4'd11;  // OSR_EFF.odr_is_valid = 0
    localparam logic [3:0] FLT_SENSOR_RST  = 4'd12;  // INT_STATUS.por seen during run
    localparam logic [3:0] FLT_DRDY_TO     = 4'd13;  // no DRDY edge within DRDY_TIMEOUT_CYC

    // ------------------------------------------------------------------------
    // Configuration ROM: one read-modify-write-verify per entry.
    // Only bits in cfg_mask are changed; reserved / untouched bits are preserved.
    // Order matters: standby first (IIR/DSP only writable in standby),
    // interrupts before measurement starts, normal mode last.
    // ------------------------------------------------------------------------
    localparam int         CFG_N    = 7;
    localparam logic [2:0] CFG_LAST = 3'(CFG_N - 1);

    logic [2:0]  cfg_idx;
    logic [7:0]  cfg_addr, cfg_mask, cfg_val;
    logic [31:0] cfg_wait;

    always_comb begin
        cfg_addr = 8'h00;
        cfg_mask = 8'h00;
        cfg_val  = 8'h00;
        cfg_wait = 32'd0;
        case (cfg_idx)
            // ODR_CONFIG: deep_dis=1, pwr_mode=standby, keep odr
            3'd0: begin cfg_addr = REG_ODR_CONFIG; cfg_mask = 8'b1000_0011;
                        cfg_val  = 8'b1000_0000;   cfg_wait = STANDBY_WAIT_CYC; end
            // OSR_CONFIG: press_en=1, osr_p, osr_t (bit 7 reserved)
            3'd1: begin cfg_addr = REG_OSR_CONFIG; cfg_mask = 8'b0111_1111;
                        cfg_val  = {1'b0, 1'b1, OSR_P, OSR_T}; end
            // DSP_CONFIG: comp_pt_en=11 (P and T compensated),
            //             shdw_sel_iir_p/t = 1 only if that IIR is enabled
            3'd2: begin cfg_addr = REG_DSP_CONFIG; cfg_mask = 8'b0010_1011;
                        cfg_val  = {2'b00, (IIR_P != 3'd0), 1'b0, (IIR_T != 3'd0), 1'b0, 2'b11}; end
            // DSP_IIR: set_iir_p, set_iir_t
            3'd3: begin cfg_addr = REG_DSP_IIR;    cfg_mask = 8'b0011_1111;
                        cfg_val  = {2'b00, IIR_P, IIR_T}; end
            // INT_SOURCE: drdy_data_reg_en only
            3'd4: begin cfg_addr = REG_INT_SOURCE; cfg_mask = 8'b0000_1111;
                        cfg_val  = 8'b0000_0001; end
            // INT_CONFIG: int_en=1, int_od=0 (push-pull), int_pol=1 (active high),
            //             int_mode=1 (latched). pad_int_drv [7:4] untouched.
            3'd5: begin cfg_addr = REG_INT_CONFIG; cfg_mask = 8'b0000_1111;
                        cfg_val  = 8'b0000_1011; end
            // ODR_CONFIG: deep_dis=1, odr, pwr_mode=normal -> measurements start
            3'd6: begin cfg_addr = REG_ODR_CONFIG; cfg_mask = 8'b1111_1111;
                        cfg_val  = {1'b1, ODR_CODE, 2'b01}; end
            default: ;
        endcase
    end

    // ------------------------------------------------------------------------
    // Sequencer state
    // ------------------------------------------------------------------------
    typedef enum logic [4:0] {
        S_PWRUP_WAIT,    // wait t_powup
        S_DUMMY1,        // dummy CHIP_ID read: switches the interface I2C -> SPI
        S_SOFTRST,       // CMD <= 0xB6
        S_SOFTRST_WAIT,  // wait t_soft_res
        S_DUMMY2,        // dummy read again (soft reset returns interface to I2C)
        S_CHK_CHIPID,    // CHIP_ID == 0x50
        S_CHK_STATUS,    // STATUS.nvm_rdy == 1, nvm_err == 0
        S_CHK_POR,       // INT_STATUS.por == 1 (also clears it)
        S_CFG_RD,        // config ROM: read current value
        S_CFG_WR,        //             write merged value
        S_CFG_VFY,       //             read back and compare masked bits
        S_CFG_WAIT,      //             optional settle time
        S_CHK_OSREFF,    // OSR_EFF.odr_is_valid == 1
        S_RUN_IDLE,      // wait for DRDY edge (or timeout)
        S_RUN_DATA,      // burst read 0x1D..0x22 (6 bytes, shadowed)
        S_RUN_INTST,     // read INT_STATUS: clears latched INT, health check
        S_FAULT          // back-off, then re-run bring-up
    } sstate_t;

    sstate_t     sstate;
    logic [31:0] wait_cnt;
    logic        txn_pending;      // a transaction for the current state is in flight
    logic [7:0]  rmw_byte;         // value read in S_CFG_RD

    // ------------------------------------------------------------------------
    // Transaction request decode (combinational, from sequencer state)
    // ------------------------------------------------------------------------
    logic        is_txn_state;
    logic [7:0]  req_b0;           // first byte: address with R/W bit
    logic [7:0]  req_wd;           // write data (0x00 when reading)
    logic [2:0]  req_nbytes;       // total bytes including address byte
    logic        txn_go;
    logic        txn_done;

    always_comb begin
        is_txn_state = 1'b1;
        req_b0       = 8'h00;
        req_wd       = 8'h00;
        req_nbytes   = 3'd2;
        case (sstate)
            S_DUMMY1, S_DUMMY2,
            S_CHK_CHIPID:  req_b0 = SPI_RD | REG_CHIP_ID;
            S_SOFTRST:     begin req_b0 = REG_CMD; req_wd = CMD_SOFT_RESET; end
            S_CHK_STATUS:  req_b0 = SPI_RD | REG_STATUS;
            S_CHK_POR:     req_b0 = SPI_RD | REG_INT_STATUS;
            S_CFG_RD,
            S_CFG_VFY:     req_b0 = SPI_RD | cfg_addr;
            S_CFG_WR:      begin req_b0 = cfg_addr & 8'h7F;
                                 req_wd = (rmw_byte & ~cfg_mask) | (cfg_val & cfg_mask); end
            S_CHK_OSREFF:  req_b0 = SPI_RD | REG_OSR_EFF;
            S_RUN_DATA:    begin req_b0 = SPI_RD | REG_TEMP_XLSB; req_nbytes = 3'd7; end
            S_RUN_INTST:   req_b0 = SPI_RD | REG_INT_STATUS;
            default:       is_txn_state = 1'b0;
        endcase
        txn_go = is_txn_state && !txn_pending;
    end

    // ------------------------------------------------------------------------
    // Transaction engine
    // One transaction = CSB low, address byte, N-1 data bytes, CSB high.
    // hold_cs = 1 on every byte except the last, so CSB stays low for bursts.
    // ------------------------------------------------------------------------
    typedef enum logic [1:0] { T_IDLE, T_REQ, T_SEND, T_WAIT } tstate_t;

    tstate_t     tstate;
    logic [2:0]  byte_idx;
    logic [2:0]  last_idx;
    logic [7:0]  txn_b0, txn_wd;
    logic [7:0]  rx_buf [0:5];
    logic [15:0] cs_idle_cnt;      // cycles since our CSB last went high (saturating)

    always_comb begin
        spi_tx_data = (byte_idx == 3'd0) ? txn_b0 : txn_wd;
        spi_hold_cs = (byte_idx != last_idx);
        spi_start   = (tstate == T_SEND) && !spi_busy;
        bus_req     = (tstate != T_IDLE);
    end

    always_ff @(posedge sys_clk) begin
        if (!rst) begin
            tstate      <= T_IDLE;
            byte_idx    <= '0;
            last_idx    <= '0;
            txn_b0      <= '0;
            txn_wd      <= '0;
            txn_done    <= 1'b0;
            cs_idle_cnt <= '1;
        end else begin
            txn_done <= 1'b0;
            if (cs_idle_cnt != '1) cs_idle_cnt <= cs_idle_cnt + 1'b1;

            case (tstate)
                T_IDLE: if (txn_go) begin
                    txn_b0   <= req_b0;
                    txn_wd   <= req_wd;
                    last_idx <= req_nbytes - 3'd1;
                    byte_idx <= '0;
                    tstate   <= T_REQ;
                end

                // wait for the bus and for the CSB idle gap since our last txn
                T_REQ: if (bus_gnt && (cs_idle_cnt >= 16'(CS_IDLE_CYC))) tstate <= T_SEND;

                // spi_start is asserted combinationally this cycle when !spi_busy
                T_SEND: if (!spi_busy) tstate <= T_WAIT;

                T_WAIT: if (spi_rx_valid) begin
                    if (byte_idx != 3'd0) rx_buf[byte_idx - 3'd1] <= spi_rx_data;
                    if (byte_idx == last_idx) begin
                        tstate      <= T_IDLE;
                        txn_done    <= 1'b1;
                        cs_idle_cnt <= '0;           // CSB just went high
                    end else begin
                        byte_idx <= byte_idx + 3'd1;
                        tstate   <= T_SEND;
                    end
                end

                default: tstate <= T_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // DRDY: 2-flop synchronizer + rising-edge detect.
    // Timestamp is captured at the edge, never at read completion.
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [1:0] int_sync;
    logic int_q;
    logic drdy_rise;

    always_ff @(posedge sys_clk) begin
        if (!rst) begin
            int_sync <= '0;
            int_q    <= 1'b0;
        end else begin
            int_sync <= {int_sync[0], bmp_int};
            int_q    <= int_sync[1];
        end
    end
    assign drdy_rise = int_sync[1] & ~int_q;

    // ------------------------------------------------------------------------
    // Sequencer
    // ------------------------------------------------------------------------
    logic            run_armed;
    logic            drdy_pending;
    logic [TS_W-1:0] ts_capture;
    logic            consume;

    logic [TS_W-1:0] cur_ts;
    logic            cur_overrun;
    logic [23:0]     cur_press, cur_temp;
    logic            cur_stale;

    assign consume = (sstate == S_RUN_IDLE) && drdy_pending;
    assign sample_sensor_id = SENSOR_ID;

    always_ff @(posedge sys_clk) begin
        if (!rst) begin
            sstate             <= S_PWRUP_WAIT;
            wait_cnt           <= '0;
            txn_pending        <= 1'b0;
            rmw_byte           <= '0;
            cfg_idx            <= '0;
            run_armed          <= 1'b0;
            drdy_pending       <= 1'b0;
            ts_capture         <= '0;
            cur_ts             <= '0;
            cur_overrun        <= 1'b0;
            cur_press          <= '0;
            cur_temp           <= '0;
            cur_stale          <= 1'b0;
            init_done          <= 1'b0;
            fault              <= 1'b0;
            fault_code         <= FLT_NONE;
            fault_count        <= '0;
            drdy_timeout_count <= '0;
            overrun_count      <= '0;
            sample_valid       <= 1'b0;
            sample_seq         <= '0;
            sample_timestamp   <= '0;
            sample_press_raw   <= '0;
            sample_temp_raw    <= '0;
            sample_flags       <= '0;
        end else begin
            sample_valid <= 1'b0;

            // transaction handshake bookkeeping
            if (txn_go)        txn_pending <= 1'b1;
            else if (txn_done) txn_pending <= 1'b0;

            // ---- DRDY capture (runs in parallel with the FSM) ----
            if (consume) drdy_pending <= 1'b0;
            if (drdy_rise && run_armed) begin
                ts_capture   <= timestamp_now;
                drdy_pending <= 1'b1;
                if (drdy_pending && !consume) begin
                    // previous sample never got read: it is lost
                    cur_overrun   <= 1'b1;
                    overrun_count <= overrun_count + 1'b1;
                end
            end

            case (sstate)
                // ---------------- bring-up ----------------
                S_PWRUP_WAIT: begin
                    init_done <= 1'b0;
                    run_armed <= 1'b0;
                    drdy_pending <= 1'b0;
                    cur_overrun <= 1'b0;
                    if (wait_cnt >= PWRUP_WAIT_CYC) begin wait_cnt <= '0; sstate <= S_DUMMY1; end
                    else wait_cnt <= wait_cnt + 1'b1;
                end

                S_DUMMY1:  if (txn_done) sstate <= S_SOFTRST;     // data is invalid by design

                S_SOFTRST: if (txn_done) begin wait_cnt <= '0; sstate <= S_SOFTRST_WAIT; end

                S_SOFTRST_WAIT: begin
                    if (wait_cnt >= SOFTRST_WAIT_CYC) begin wait_cnt <= '0; sstate <= S_DUMMY2; end
                    else wait_cnt <= wait_cnt + 1'b1;
                end

                S_DUMMY2:  if (txn_done) sstate <= S_CHK_CHIPID;

                S_CHK_CHIPID: if (txn_done) begin
                    if (rx_buf[0] == CHIP_ID_BMP581) sstate <= S_CHK_STATUS;
                    else begin fault_code <= FLT_CHIP_ID; fault <= 1'b1;
                               fault_count <= fault_count + 1'b1; wait_cnt <= '0; sstate <= S_FAULT; end
                end

                S_CHK_STATUS: if (txn_done) begin
                    if (rx_buf[0][1] && !rx_buf[0][2]) sstate <= S_CHK_POR;
                    else begin fault_code <= FLT_NVM; fault <= 1'b1;
                               fault_count <= fault_count + 1'b1; wait_cnt <= '0; sstate <= S_FAULT; end
                end

                S_CHK_POR: if (txn_done) begin
                    if (rx_buf[0][4]) begin cfg_idx <= '0; sstate <= S_CFG_RD; end
                    else begin fault_code <= FLT_NO_POR; fault <= 1'b1;
                               fault_count <= fault_count + 1'b1; wait_cnt <= '0; sstate <= S_FAULT; end
                end

                // ---------------- configuration ROM walk ----------------
                S_CFG_RD: begin
                    // Arm DRDY capture before the step that enters normal mode, so the
                    // first sample's edge is caught even if it lands before S_RUN_IDLE.
                    if (cfg_idx == CFG_LAST) run_armed <= 1'b1;
                    if (txn_done) begin rmw_byte <= rx_buf[0]; sstate <= S_CFG_WR; end
                end

                S_CFG_WR:  if (txn_done) sstate <= S_CFG_VFY;

                S_CFG_VFY: if (txn_done) begin
                    if ((rx_buf[0] & cfg_mask) == (cfg_val & cfg_mask)) begin
                        wait_cnt <= '0;
                        sstate   <= S_CFG_WAIT;
                    end else begin
                        fault_code <= FLT_CFG_BASE + {1'b0, cfg_idx}; fault <= 1'b1;
                        fault_count <= fault_count + 1'b1; wait_cnt <= '0; sstate <= S_FAULT;
                    end
                end

                S_CFG_WAIT: begin
                    if (wait_cnt >= cfg_wait) begin
                        wait_cnt <= '0;
                        if (cfg_idx == CFG_LAST) sstate <= S_CHK_OSREFF;
                        else begin cfg_idx <= cfg_idx + 3'd1; sstate <= S_CFG_RD; end
                    end else wait_cnt <= wait_cnt + 1'b1;
                end

                S_CHK_OSREFF: if (txn_done) begin
                    if (rx_buf[0][7]) begin
                        init_done   <= 1'b1;
                        fault       <= 1'b0;
                        wait_cnt    <= '0;
                        sstate      <= S_RUN_IDLE;
                    end else begin
                        fault_code <= FLT_OSR_ODR; fault <= 1'b1;
                        fault_count <= fault_count + 1'b1; wait_cnt <= '0; sstate <= S_FAULT;
                    end
                end

                // ---------------- run loop ----------------
                S_RUN_IDLE: begin
                    if (drdy_pending) begin
                        cur_ts       <= ts_capture;
                        wait_cnt     <= '0;
                        sstate       <= S_RUN_DATA;
                    end else if (wait_cnt >= DRDY_TIMEOUT_CYC) begin
                        drdy_timeout_count <= drdy_timeout_count + 1'b1;
                        fault_code  <= FLT_DRDY_TO; fault <= 1'b1;
                        fault_count <= fault_count + 1'b1;
                        wait_cnt    <= '0;
                        sstate      <= S_PWRUP_WAIT;        // sensor is suspect: full re-init
                    end else wait_cnt <= wait_cnt + 1'b1;
                end

                S_RUN_DATA: if (txn_done) begin
                    // little endian: XLSB, LSB, MSB
                    cur_temp  <= {rx_buf[2], rx_buf[1], rx_buf[0]};
                    cur_press <= {rx_buf[5], rx_buf[4], rx_buf[3]};
                    // Latched INT gives no new edge for a sample that lands before we read,
                    // so a long edge-to-read latency is the only sign the data moved on.
                    cur_stale <= ((timestamp_now - cur_ts) > TS_W'(STALE_CYC));
                    sstate    <= S_RUN_INTST;
                end

                S_RUN_INTST: if (txn_done) begin
                    if (rx_buf[0][4]) begin
                        // POR bit: sensor reset under us, config is gone. Drop sample.
                        fault_code  <= FLT_SENSOR_RST; fault <= 1'b1;
                        fault_count <= fault_count + 1'b1;
                        wait_cnt    <= '0;
                        sstate      <= S_PWRUP_WAIT;
                    end else begin
                        sample_valid     <= 1'b1;
                        sample_seq       <= sample_seq + 1'b1;
                        sample_timestamp <= cur_ts;
                        sample_press_raw <= cur_press;
                        sample_temp_raw  <= cur_temp;
                        sample_flags     <= { 3'b000,
                                              cur_stale,               // [4] data may be newer than timestamp
                                              ~rx_buf[0][0],           // [3] drdy status not set
                                              cur_overrun,             // [2] a sample was lost before this one
                                              (cur_temp  == SKIPPED),  // [1]
                                              (cur_press == SKIPPED) };// [0]
                        cur_overrun      <= 1'b0;
                        wait_cnt         <= '0;
                        sstate           <= S_RUN_IDLE;
                    end
                end

                S_FAULT: begin
                    init_done <= 1'b0;
                    run_armed <= 1'b0;
                    if (wait_cnt >= RETRY_WAIT_CYC) begin wait_cnt <= '0; sstate <= S_PWRUP_WAIT; end
                    else wait_cnt <= wait_cnt + 1'b1;
                end

                default: sstate <= S_PWRUP_WAIT;
            endcase
        end
    end

endmodule

`default_nettype wire
