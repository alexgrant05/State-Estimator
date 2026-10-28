// ============================================================================
// ADXL375 test rig - FPGA top level
//
//   ADXL375 <--SPI--> [adxl375_ctrl (spimaster #1)]           FPGA = SPI master
//                              |
//                       [packet builder] --> [spimaster #2] --SPI--> Pico 2 W (SPI slave)
//                                                                  --USB--> PC
//
// The FPGA pushes one 20-byte packet to the Pico:
//   * for every new sample (SAMPLE packet, ~100 Hz once streaming)
//   * immediately when the controller's status changes (STATUS packet)
//   * every HEARTBEAT_CYCLES if nothing else was sent (STATUS packet), so the
//     PC sees the state during power-up, init retries and faults.
// Every packet carries the full status, so any single packet is enough to know
// the sensor's state.
//
// Link: mode 3 (CPOL=1, CPHA=1), MSB first, CS held low for the whole packet.
// Mode 3 matters: the Pico's PL022 in slave mode with CPHA=0 needs CS to go
// high between every byte; with CPHA=1 it accepts back-to-back bytes.
// The link is one-way. The Pico's TX (MISO) pin is not used.
//
// Packet (little endian, like the ADXL375 data registers):
//   [0]  0xA5            sync
//   [1]  0x5A            sync
//   [2]  0x1T            version 1 in the high nibble, type T: 1 = SAMPLE, 2 = STATUS
//   [3]  seq             packet counter, wraps at 256
//   [4]  flags           b0 init_done, b1 fault, b2 fault_cause (1 = INT1 watchdog),
//                        b3 INT1 pin level, b7..b4 = 0
//   [5]  fault_pc        last fault: init step (0..15)
//   [6]  fault_rdata     last fault: last byte read back
//   [7]  reinit_count    recoveries since reset (saturates at 255)
//   [8..11]  sample_count  samples since reset (this packet's sample if SAMPLE)
//   [12..13] X  [14..15] Y  [16..17] Z   signed raw counts, latest sample
//   [18] link_drops      samples overwritten before they could be sent (saturates)
//   [19] crc             CRC-8, poly 0x07, init 0x00, over bytes [0..18]
//
// Default timing assumes CLK_HZ; override the ACC_* and link cycle counts only
// for simulation.
// ============================================================================

module adxl375_rig_top #(
    parameter int CLK_HZ           = 100_000_000,
    parameter bit RST_ACTIVE_LOW   = 1'b0,        // set to 1 if the board's reset button is active low

    // Pico link
    parameter int LINK_CLK_DIV     = CLK_HZ / 2_000_000 * 2,  // ~1 MHz SCLK, kept even
    parameter int LINK_GAP_CYCLES  = CLK_HZ / 500_000,        // 2 us CS high between packets
    parameter int HEARTBEAT_CYCLES = CLK_HZ / 10,             // status packet at least every 100 ms

    // passed straight to adxl375_ctrl (defaults = real-time values)
    parameter int ACC_CLK_DIV            = CLK_HZ / 2_000_000 * 2,
    parameter int ACC_PWR_UP_CYCLES      = CLK_HZ / 50,
    parameter int ACC_RETRY_CYCLES       = CLK_HZ / 100,
    parameter int ACC_MAX_ATTEMPTS       = 10,
    parameter int ACC_REINIT_CYCLES      = CLK_HZ / 2,
    parameter int ACC_CS_GAP_CYCLES      = CLK_HZ / 2_000_000,
    parameter int ACC_HOLDOFF_CYCLES     = CLK_HZ / 200_000,
    parameter int ACC_REARM_CYCLES       = CLK_HZ / 200,
    parameter int ACC_INT_TIMEOUT_CYCLES = CLK_HZ / 100 * 3
) (
    input  logic       clk,
    input  logic       rst_btn,

    // ADXL375
    output logic       acc_cs_n,
    output logic       acc_sclk,
    output logic       acc_mosi,
    input  logic       acc_miso,
    input  logic       acc_int1,

    // Pico 2 W (SPI0 slave: GP17 CSn, GP18 SCK, GP16 RX)
    output logic       pico_cs_n,
    output logic       pico_sclk,
    output logic       pico_mosi,

    // board LEDs: [0] streaming  [1] fault  [2] 1 Hz blink while streaming
    //             [3] sensor has needed a recovery since reset
    output logic [3:0] led
);

    // ------------------------------------------------------------------------
    // Parameter checks
    // ------------------------------------------------------------------------
    // The RP2350's PL022 needs clk_peri >= 12 x SCLK in slave mode
    // (150 MHz / 12 = 12.5 MHz). Stay well under it for jumper wires.
    if (LINK_CLK_DIV < 2 || LINK_CLK_DIV % 2 != 0) begin : g_chk_link_div
        $error("adxl375_rig_top: LINK_CLK_DIV (%0d) must be even and >= 2", LINK_CLK_DIV);
    end
    if (CLK_HZ / LINK_CLK_DIV > 10_000_000) begin : g_chk_link_fsclk
        $error("adxl375_rig_top: link SCLK %0d Hz is above 10 MHz", CLK_HZ / LINK_CLK_DIV);
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
    logic               sample_valid, init_done, fault, fault_cause;
    logic signed [15:0] ax, ay, az;
    logic [3:0]         fault_pc;
    logic [7:0]         fault_rdata, reinit_count;

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
        .INT_TIMEOUT_CYCLES(ACC_INT_TIMEOUT_CYCLES)
    ) u_acc (
        .clk, .rst(rst_i),
        .cs_n(acc_cs_n), .sclk(acc_sclk), .mosi(acc_mosi), .miso(acc_miso), .int1(acc_int1),
        .sample_valid, .accel_x(ax), .accel_y(ay), .accel_z(az),
        .init_done, .fault, .fault_cause, .fault_pc, .fault_rdata, .reinit_count
    );

    // INT1 level for reporting only (the controller has its own synchronizer)
    (* ASYNC_REG = "TRUE" *) logic int1_s1, int1_s2;
    always_ff @(posedge clk) begin
        int1_s1 <= acc_int1;
        int1_s2 <= int1_s1;
    end

    // ------------------------------------------------------------------------
    // Packet builder / sender
    // ------------------------------------------------------------------------
    localparam int         PKT_LEN     = 20;
    localparam logic [3:0] T_SAMPLE    = 4'h1;
    localparam logic [3:0] T_STATUS    = 4'h2;

    function automatic logic [7:0] crc8_step(input logic [7:0] crc, input logic [7:0] d);
        logic [7:0] c;
        c = crc ^ d;
        for (int i = 0; i < 8; i++)
            c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
        return c;
    endfunction

    typedef enum logic [1:0] {L_IDLE, L_SEND, L_WAIT, L_GAP} lstate_t;

    lstate_t                  lstate;
    logic [8*(PKT_LEN-1)-1:0] snap;          // bytes 0..18; byte i at snap[8*i +: 8]
    logic [4:0]               idx;
    logic [7:0]               crc, seq, link_drops;
    logic [31:0]              sample_count;
    logic                     sample_pending;
    int unsigned              hb_cnt, gap;
    logic [6:0]               blink_cnt;
    logic                     blink;

    // status as last sent; a difference triggers an immediate packet
    logic [22:0] status_now, status_sent;
    assign status_now = {init_done, fault, fault_cause, fault_pc, fault_rdata, reinit_count};

    wire [7:0] flags    = {4'b0, int1_s2, fault_cause, fault, init_done};
    wire [7:0] cur_byte = (idx == 5'(PKT_LEN - 1)) ? crc : snap[8*idx +: 8];

    logic       spi_start, spi_hold_cs, spi_rx_valid;
    logic [7:0] spi_tx;
    /* verilator lint_off UNUSEDSIGNAL */
    logic       spi_busy;
    logic [7:0] spi_rx;                      // link is one-way: nothing comes back
    /* verilator lint_on UNUSEDSIGNAL */

    always_ff @(posedge clk) begin
        spi_start <= 1'b0;

        if (rst_i) begin
            lstate         <= L_IDLE;
            snap           <= '0;
            idx            <= '0;
            crc            <= '0;
            seq            <= '0;
            link_drops     <= '0;
            sample_count   <= '0;
            sample_pending <= 1'b0;
            hb_cnt         <= 0;
            gap            <= 0;
            spi_tx         <= '0;
            spi_hold_cs    <= 1'b0;
            status_sent    <= '1;          // differs from any real status: report at boot
            blink_cnt      <= '0;
            blink          <= 1'b0;
        end else begin
            if (hb_cnt < HEARTBEAT_CYCLES) hb_cnt <= hb_cnt + 1;

            // New sample: count it and mark it for sending. If the previous
            // one is still waiting, it is lost (only if the link is too slow).
            if (sample_valid) begin
                sample_count   <= sample_count + 1;
                sample_pending <= 1'b1;
                if (sample_pending && link_drops != 8'hFF) link_drops <= link_drops + 1;
                if (blink_cnt == 7'd49) begin
                    blink_cnt <= '0;
                    blink     <= ~blink;
                end else
                    blink_cnt <= blink_cnt + 1;
            end

            case (lstate)
                // Take a snapshot. Never on a sample_valid cycle, so the
                // sample data and sample_count are always consistent.
                L_IDLE:
                    if (!sample_valid &&
                        (sample_pending || status_now != status_sent || hb_cnt >= HEARTBEAT_CYCLES)) begin
                        snap <= {link_drops,
                                 az, ay, ax,
                                 sample_count,
                                 reinit_count,
                                 fault_rdata,
                                 {4'h0, fault_pc},
                                 flags,
                                 seq,
                                 {4'h1, sample_pending ? T_SAMPLE : T_STATUS},
                                 8'h5A, 8'hA5};
                        status_sent    <= status_now;
                        sample_pending <= 1'b0;
                        hb_cnt         <= 0;
                        seq            <= seq + 1;
                        idx            <= '0;
                        crc            <= '0;
                        lstate         <= L_SEND;
                    end

                L_SEND: begin
                    spi_tx      <= cur_byte;
                    spi_hold_cs <= (idx != 5'(PKT_LEN - 1));    // release CS after the CRC
                    spi_start   <= 1'b1;
                    crc         <= crc8_step(crc, cur_byte);
                    lstate      <= L_WAIT;
                end

                // wait on rx_valid, not busy (busy rises a cycle late)
                L_WAIT:
                    if (spi_rx_valid) begin
                        if (idx == 5'(PKT_LEN - 1)) begin
                            gap    <= 0;
                            lstate <= L_GAP;
                        end else begin
                            idx    <= idx + 1;
                            lstate <= L_SEND;
                        end
                    end

                L_GAP:
                    if (gap >= LINK_GAP_CYCLES) lstate <= L_IDLE;
                    else                        gap    <= gap + 1;
            endcase
        end
    end

    spimaster #(.CLOCK_DIVIDE(LINK_CLK_DIV), .CPOL(1), .CPHA(1)) u_link (
        .sys_clk(clk), .rst(rst_i),
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold_cs),
        .busy(spi_busy), .rx_data(spi_rx), .rx_valid(spi_rx_valid),
        .cs_n(pico_cs_n), .sclk(pico_sclk), .mosi(pico_mosi), .miso(1'b0)
    );

    // ------------------------------------------------------------------------
    // LEDs: useful even with the Pico unplugged
    // ------------------------------------------------------------------------
    assign led = {reinit_count != 8'd0, blink & init_done, fault, init_done};

endmodule
