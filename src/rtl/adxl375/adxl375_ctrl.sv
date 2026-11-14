// ============================================================================
// ADXL375 controller
//
//   adxl375_ctrl : sequencer. Power-up wait -> microcode init -> streaming,
//                  with automatic recovery from init failures and lost INT1.
//   adxl375_txn  : frame engine. One ADXL375 frame = command byte + N bytes.
//   spimaster    : moves one byte at a time (mode 3).
//
// Handshake between layers: pulse start for 1 cycle, wait for a 1-cycle done.
//
// rst is active high and may be asynchronous (e.g. a push button or an MMCM's
// ~locked): it is synchronized here, and a power-on reset is generated
// internally, so the design comes up correctly even if rst is never pulsed.
//
// All timing parameters default to real-time values derived from CLK_HZ.
// Override CLK_HZ for a different board clock; override individual cycle
// counts only for simulation.
// ============================================================================

module adxl375_ctrl #(
    parameter int CLK_HZ             = 100_000_000,
    parameter int CLK_DIV            = CLK_HZ / 2_000_000 * 2, // SCLK ~1 MHz, kept even (sensor max 5 MHz)
    parameter int PWR_UP_CYCLES      = CLK_HZ / 50,         // 20 ms before first access
    parameter int RETRY_CYCLES       = CLK_HZ / 100,        // 10 ms between attempts of one step
    parameter int MAX_ATTEMPTS       = 10,                  // attempts per step before a fault
    parameter int REINIT_CYCLES      = CLK_HZ / 2,          // 500 ms after a fault, then re-init
    parameter int CS_GAP_CYCLES      = CLK_HZ / 2_000_000,  // 500 ns CS high between frames
    parameter int HOLDOFF_CYCLES     = CLK_HZ / 200_000,    // 5 us after each sample read
    parameter int REARM_CYCLES       = CLK_HZ / 200,        // 5 ms: max wait for INT1 to drop
    parameter int INT_TIMEOUT_CYCLES = CLK_HZ / 100 * 3,    // 30 ms (3 samples) with no INT1 -> re-init
    // BW_RATE register value: output data rate = 3200 Hz / 2^(15 - code).
    //   0x0A = 100 Hz (default)   0x0D = 800 Hz   0x0E = 1600 Hz   0x0F = 3200 Hz
    // 1600 and 3200 Hz need SCLK >= 2 MHz (datasheet), REARM_CYCLES under one
    // sample period and INT_TIMEOUT_CYCLES over one (checked below).
    parameter logic [7:0] BW_RATE    = 8'h0A
) (
    input  logic clk,
    input  logic rst,

    // sensor pins
    output logic cs_n,
    output logic sclk,
    output logic mosi,
    input  logic miso,
    input  logic int1,

    // to the rest of the fabric
    output logic               sample_valid,  // 1-cycle pulse per new sample
    output logic signed [15:0] accel_x,       // raw counts, ~49 mg/LSB
    output logic signed [15:0] accel_y,
    output logic signed [15:0] accel_z,
    output logic               init_done,     // high while streaming
    output logic               fault,         // set on any failure; clears on the next good sample
    output logic               fault_cause,   // last fault: 0 = init step failed, 1 = INT1 watchdog
    output logic [3:0]         fault_pc,      // last fault: init step that was running
    output logic [7:0]         fault_rdata,   // last fault: last byte read back
    output logic [7:0]         reinit_count   // recoveries since reset (saturates at 255)
);

    // ------------------------------------------------------------------------
    // Parameter checks (elaboration time)
    // ------------------------------------------------------------------------
    if (CLK_DIV < 2 || CLK_DIV % 2 != 0) begin : g_chk_div
        $error("adxl375_ctrl: CLK_DIV (%0d) must be even and >= 2", CLK_DIV);
    end
    if (CLK_HZ / CLK_DIV > 5_000_000) begin : g_chk_fsclk
        $error("adxl375_ctrl: SCLK %0d Hz exceeds the ADXL375 5 MHz limit", CLK_HZ / CLK_DIV);
    end
    if (CLK_DIV / 2 > 65535) begin : g_chk_div_max
        $error("adxl375_ctrl: CLK_DIV (%0d) too large for spimaster's 16-bit counter", CLK_DIV);
    end
    if (CS_GAP_CYCLES < (CLK_HZ / 1_000_000) * 150 / 1000) begin : g_chk_cs_gap
        $error("adxl375_ctrl: CS_GAP_CYCLES (%0d) is under the 150 ns tCS,DIS", CS_GAP_CYCLES);
    end
    if (MAX_ATTEMPTS < 1) begin : g_chk_attempts
        $error("adxl375_ctrl: MAX_ATTEMPTS must be >= 1");
    end
    if (HOLDOFF_CYCLES + REARM_CYCLES >= INT_TIMEOUT_CYCLES) begin : g_chk_wdog
        $error("adxl375_ctrl: INT_TIMEOUT_CYCLES must exceed HOLDOFF_CYCLES + REARM_CYCLES");
    end

    // one sample period at BW_RATE, in clk cycles
    localparam longint ODR_PERIOD_CYCLES = (longint'(CLK_HZ) / 3200) << (15 - BW_RATE[3:0]);
    if (BW_RATE[7:4] != 4'h0) begin : g_chk_bw
        $error("adxl375_ctrl: BW_RATE 0x%0h: only the rate code (bits 3:0) may be set", BW_RATE);
    end
    if (BW_RATE[3:0] >= 4'hE && CLK_HZ / CLK_DIV < 2_000_000) begin : g_chk_fast_sclk
        $error("adxl375_ctrl: 1600/3200 Hz output needs SCLK >= 2 MHz (have %0d Hz)", CLK_HZ / CLK_DIV);
    end
    // Warnings, not errors: testbenches run the sensor model far faster than
    // any real rate and scale these down to match the model instead.
    if (longint'(HOLDOFF_CYCLES + REARM_CYCLES) >= ODR_PERIOD_CYCLES) begin : g_chk_rearm
        $warning("adxl375_ctrl: HOLDOFF_CYCLES + REARM_CYCLES should be under one sample period (%0d cycles)",
                 ODR_PERIOD_CYCLES);
    end
    if (longint'(INT_TIMEOUT_CYCLES) <= ODR_PERIOD_CYCLES) begin : g_chk_timeout
        $warning("adxl375_ctrl: INT_TIMEOUT_CYCLES should exceed one sample period (%0d cycles)", ODR_PERIOD_CYCLES);
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

    // ------------------------------------------------------------------------
    // Microcode. Edit this table to change the init sequence.
    // Unused slots must stay OP_END.
    // ------------------------------------------------------------------------
    typedef enum logic [1:0] {
        OP_READ_EXPECT,   // read addr until it equals data
        OP_WRITE_VERIFY,  // write data to addr, read back, retry on mismatch
        OP_END            // init finished, start streaming
    } op_t;

    typedef struct packed {
        op_t        op;
        logic [5:0] addr;
        logic [7:0] data;
    } uinstr_t;

    localparam uinstr_t ROM [16] = '{
        '{OP_READ_EXPECT,  6'h00, 8'hE5},  // 0 DEVID (retries double as power-up poll)
        '{OP_WRITE_VERIFY, 6'h2D, 8'h00},  // 1 POWER_CTL   standby while configuring
        '{OP_WRITE_VERIFY, 6'h2C, BW_RATE}, // 2 BW_RATE    output data rate (parameter)
        '{OP_WRITE_VERIFY, 6'h31, 8'h0B},  // 3 DATA_FORMAT 4-wire SPI, INT active high
        '{OP_WRITE_VERIFY, 6'h38, 8'h00},  // 4 FIFO_CTL    bypass
        '{OP_WRITE_VERIFY, 6'h2F, 8'h00},  // 5 INT_MAP     DATA_READY -> INT1
        '{OP_WRITE_VERIFY, 6'h2E, 8'h80},  // 6 INT_ENABLE  DATA_READY
        '{OP_WRITE_VERIFY, 6'h2D, 8'h08},  // 7 POWER_CTL   measure (must be last)
        '{OP_END,          6'h00, 8'h00},  // 8
        '{OP_END,          6'h00, 8'h00},
        '{OP_END,          6'h00, 8'h00},
        '{OP_END,          6'h00, 8'h00},
        '{OP_END,          6'h00, 8'h00},
        '{OP_END,          6'h00, 8'h00},
        '{OP_END,          6'h00, 8'h00},
        '{OP_END,          6'h00, 8'h00}
    };

    // ------------------------------------------------------------------------
    // Sequencer state
    // ------------------------------------------------------------------------
    typedef enum logic [3:0] {
        PWR_WAIT,      // count power-up delay
        FETCH,         // look at ROM[pc], launch its first transaction
        WAIT_WR,       // write in progress -> then launch the readback
        WAIT_RD,       // read in progress  -> then compare
        RETRY_WAIT,    // mismatch: wait, then FETCH the same pc again
        REINIT_WAIT,   // step ran out of attempts: wait, then restart at pc 0
        STREAM_READY,  // wait for INT1 high (watchdog running)
        STREAM_WAIT,   // 6-byte burst in progress
        STREAM_HOLD,   // fixed holdoff after a burst
        STREAM_ARM     // wait for INT1 to drop before reading again
    } state_t;

    state_t      state;
    logic [3:0]  pc;
    uinstr_t     instr;
    int unsigned delay;            // shared by all the timed waits
    int unsigned wdog;             // cycles since the last sample was delivered
    int unsigned fails;            // failed attempts at the current step
    logic        streaming;
    logic [47:0] raw;              // burst bytes, shifted in from the top
    (* ASYNC_REG = "TRUE" *) logic int1_s1, int1_s2;  // INT1 synchronizer

    assign instr = ROM[pc];

    // Requests to the frame engine. Address/length/data come straight from
    // the current instruction (or the fixed streaming read), so the FSM only
    // has to choose read vs write and pulse start.
    logic       txn_start, txn_rw, txn_done, rd_valid;
    logic [7:0] rd_data;
    wire  [5:0] txn_addr   = streaming ? 6'h32 : instr.addr;
    wire  [2:0] txn_nbytes = streaming ? 3'd6  : 3'd1;

    always_ff @(posedge clk) begin
        txn_start    <= 1'b0;   // pulses: default low
        sample_valid <= 1'b0;
        int1_s1      <= int1;
        int1_s2      <= int1_s1;

        if (rst_i) begin
            state        <= PWR_WAIT;
            pc           <= '0;
            delay        <= 0;
            wdog         <= 0;
            fails        <= 0;
            streaming    <= 1'b0;
            txn_rw       <= 1'b0;
            raw          <= '0;
            accel_x      <= '0;
            accel_y      <= '0;
            accel_z      <= '0;
            init_done    <= 1'b0;
            fault        <= 1'b0;
            fault_cause  <= 1'b0;
            fault_pc     <= '0;
            fault_rdata  <= '0;
            reinit_count <= '0;
        end else begin
            // Watchdog runs whenever streaming is waiting on the sensor.
            // Assignments further down (to 0) take precedence.
            if (state == STREAM_HOLD || state == STREAM_ARM || state == STREAM_READY)
                wdog <= wdog + 1;

            case (state)

                PWR_WAIT:
                    if (delay >= PWR_UP_CYCLES) state <= FETCH;
                    else                        delay <= delay + 1;

                FETCH:
                    case (instr.op)
                        OP_WRITE_VERIFY: begin
                            txn_rw    <= 1'b0;               // write first
                            txn_start <= 1'b1;
                            state     <= WAIT_WR;
                        end
                        OP_READ_EXPECT: begin
                            txn_rw    <= 1'b1;               // read only
                            txn_start <= 1'b1;
                            state     <= WAIT_RD;
                        end
                        default: begin                       // OP_END
                            txn_rw    <= 1'b1;               // streaming is read-only
                            streaming <= 1'b1;
                            init_done <= 1'b1;
                            wdog      <= 0;
                            state     <= STREAM_READY;       // INT1 may already be high
                        end
                    endcase

                WAIT_WR:
                    if (txn_done) begin
                        txn_rw    <= 1'b1;                   // now read it back
                        txn_start <= 1'b1;
                        state     <= WAIT_RD;
                    end

                WAIT_RD:
                    if (txn_done) begin
                        if (rd_data == instr.data) begin     // match: next instruction
                            pc    <= pc + 1;
                            fails <= 0;
                            state <= FETCH;
                        end else if (fails + 1 >= MAX_ATTEMPTS) begin
                            // Out of attempts: report, back off, start over.
                            fault       <= 1'b1;
                            fault_cause <= 1'b0;
                            fault_pc    <= pc;
                            fault_rdata <= rd_data;
                            delay       <= 0;
                            state       <= REINIT_WAIT;
                        end else begin
                            fails <= fails + 1;
                            delay <= 0;
                            state <= RETRY_WAIT;
                        end
                    end

                RETRY_WAIT:
                    if (delay >= RETRY_CYCLES) state <= FETCH;
                    else                       delay <= delay + 1;

                REINIT_WAIT:
                    if (delay >= REINIT_CYCLES) begin
                        pc    <= '0;
                        fails <= 0;
                        if (reinit_count != 8'hFF) reinit_count <= reinit_count + 1;
                        state <= FETCH;
                    end else
                        delay <= delay + 1;

                // Read whenever INT1 is high. If it stays low too long the
                // sensor has lost its configuration (brown-out) or INT1 is
                // broken: re-run init from the top.
                STREAM_READY:
                    if (int1_s2) begin
                        txn_start <= 1'b1;
                        state     <= STREAM_WAIT;
                    end else if (wdog >= INT_TIMEOUT_CYCLES) begin
                        fault       <= 1'b1;
                        fault_cause <= 1'b1;
                        fault_pc    <= pc;
                        fault_rdata <= rd_data;
                        streaming   <= 1'b0;
                        init_done   <= 1'b0;
                        pc          <= '0;
                        fails       <= 0;
                        if (reinit_count != 8'hFF) reinit_count <= reinit_count + 1;
                        state       <= FETCH;
                    end

                STREAM_WAIT: begin
                    if (rd_valid) raw <= {rd_data, raw[47:8]};
                    if (txn_done) begin
                        // bytes arrive X0 X1 Y0 Y1 Z0 Z1 (little endian)
                        {accel_z, accel_y, accel_x} <= raw;
                        sample_valid <= 1'b1;
                        fault        <= 1'b0;                // a good sample means healthy
                        wdog         <= 0;
                        delay        <= 0;
                        state        <= STREAM_HOLD;
                    end
                end

                STREAM_HOLD:
                    if (delay >= HOLDOFF_CYCLES) begin
                        delay <= 0;
                        state <= STREAM_ARM;
                    end else
                        delay <= delay + 1;

                // Don't read again until INT1 has dropped, so a slow-clearing
                // DATA_READY can't produce a duplicate sample. If it never
                // drops (a new sample landed during the read), give up waiting
                // after REARM_CYCLES and read it.
                STREAM_ARM:
                    if (!int1_s2 || delay >= REARM_CYCLES) state <= STREAM_READY;
                    else                                   delay <= delay + 1;

                default: begin                               // illegal encoding: recover
                    fault       <= 1'b1;
                    fault_cause <= 1'b0;
                    fault_pc    <= pc;
                    fault_rdata <= rd_data;
                    streaming   <= 1'b0;
                    init_done   <= 1'b0;
                    delay       <= 0;
                    state       <= REINIT_WAIT;
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
    logic       spi_busy;          // not needed: the frame engine waits on rx_valid
    /* verilator lint_on UNUSEDSIGNAL */

    adxl375_txn #(.CS_GAP_CYCLES(CS_GAP_CYCLES)) u_txn (
        .clk, .rst(rst_i),
        .start(txn_start), .rw(txn_rw), .addr(txn_addr), .nbytes(txn_nbytes), .wdata(instr.data),
        .done(txn_done), .rd_valid, .rd_data,
        .spi_start, .spi_tx, .spi_hold_cs, .spi_rx, .spi_rx_valid
    );

    spimaster #(.CLOCK_DIVIDE(CLK_DIV), .CPOL(1), .CPHA(1)) u_spi (
        .sys_clk(clk), .rst(rst_i),
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold_cs),
        .busy(spi_busy), .rx_data(spi_rx), .rx_valid(spi_rx_valid),
        .cs_n, .sclk, .mosi, .miso
    );

endmodule


// ============================================================================
// Frame engine
//
// Sends [rw][mb][addr] then nbytes payload bytes with CS held low, then keeps
// CS high for CS_GAP_CYCLES before pulsing done. Writes use nbytes = 1.
// Byte counter: 0 = command byte, 1..nbytes = payload.
// ============================================================================

module adxl375_txn #(
    parameter int CS_GAP_CYCLES = 50
) (
    input  logic       clk,
    input  logic       rst,

    input  logic       start,      // latches rw/addr/nbytes/wdata
    input  logic       rw,         // 1 = read
    input  logic [5:0] addr,
    input  logic [2:0] nbytes,     // payload bytes, 1..6
    input  logic [7:0] wdata,

    output logic       done,       // 1-cycle pulse
    output logic       rd_valid,   // 1-cycle pulse per byte read
    output logic [7:0] rd_data,    // holds the last byte read

    output logic       spi_start,
    output logic [7:0] spi_tx,
    output logic       spi_hold_cs,
    input  logic [7:0] spi_rx,
    input  logic       spi_rx_valid
);

    typedef enum logic [1:0] {IDLE, SEND, BYTE_WAIT, CS_GAP} state_t;

    state_t      state;
    logic        rw_r;
    logic [5:0]  addr_r;
    logic [2:0]  n_r, cnt;
    logic [7:0]  wdata_r;
    int unsigned gap;

    always_ff @(posedge clk) begin
        spi_start <= 1'b0;
        done      <= 1'b0;
        rd_valid  <= 1'b0;

        if (rst) begin
            state       <= IDLE;
            spi_tx      <= '0;
            spi_hold_cs <= 1'b0;
            rd_data     <= '0;
        end else begin
            case (state)

                IDLE:
                    if (start) begin
                        rw_r    <= rw;
                        addr_r  <= addr;
                        n_r     <= nbytes;
                        wdata_r <= wdata;
                        cnt     <= '0;
                        state   <= SEND;
                    end

                SEND: begin
                    if (cnt == 0) spi_tx <= {rw_r, n_r > 1, addr_r};  // command byte
                    else          spi_tx <= rw_r ? 8'h00 : wdata_r;   // dummy or data
                    spi_hold_cs <= (cnt != n_r);                      // release on last byte
                    spi_start   <= 1'b1;
                    state       <= BYTE_WAIT;
                end

                // wait on rx_valid, not busy (busy rises a cycle late)
                BYTE_WAIT:
                    if (spi_rx_valid) begin
                        if (rw_r && cnt != 0) begin
                            rd_valid <= 1'b1;
                            rd_data  <= spi_rx;
                        end
                        if (cnt == n_r) begin
                            gap   <= 0;
                            state <= CS_GAP;
                        end else begin
                            cnt   <= cnt + 1;
                            state <= SEND;
                        end
                    end

                CS_GAP:
                    if (gap >= CS_GAP_CYCLES) begin
                        done  <= 1'b1;
                        state <= IDLE;
                    end else
                        gap <= gap + 1;

            endcase
        end
    end

endmodule
