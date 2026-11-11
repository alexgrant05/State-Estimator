// ============================================================================
// BNO085 SHTP transport (SPI)
//
// Sits between bno085_ctrl and spimaster. Owns the INTN / WAKE / CS
// handshake and the SHTP 4-byte header, following CEVA's SPI HAL
// (sh2-demo-nucleo app/spi_hal.c: spiActivate / spiCompleted / hal_write):
//
//   Read  (INTN low, nothing to write):
//     CS low, clock 4 header bytes sending 0x00, len = {b1,b0} & 0x7FFF.
//     len > 4: keep CS low and clock len-4 more bytes. Otherwise CS high.
//   Write (wr_start):
//     WAKE low, wait for INTN low, CS low, WAKE high once CS is low,
//     clock exactly wr_len bytes ([len][len>>8][chan][seq] + payload), CS high.
//     The transfer is full duplex: the hub's header arrives in bytes 0..3.
//     If its packet fits inside the write (4 < len <= wr_len) it is delivered
//     like a read (WRITE_RX_DELIVER = 1). If it is longer it was truncated: the
//     hub resends the rest later as a continuation fragment, which the parser
//     drops; rx_trunc pulses so the controller can count it.
//
// After every transfer CS stays high for CS_GAP_CYCLES before INTN is looked
// at again. INTN is edge-triggered in SHTP (one transfer per assertion) and
// the hub may release it late, so a new transfer needs INTN to have been seen
// high since the last one started ("armed"), or INTN stuck low for
// INTN_STALE_CYCLES (backstop; the hub itself re-pulses INTN after ~10 ms).
//
// Output byte stream: payload bytes only (header stripped), with sop on the
// first payload byte, eop on the last, and chan/cont from the header held
// for the whole packet. Packets with len <= 4 (null headers) produce nothing.
// ============================================================================

module bno085_shtp #(
    parameter int CS_GAP_CYCLES       = 200,        // CS high after each transfer
    parameter int INTN_STALE_CYCLES   = 1_000_000,  // INTN low this long counts as a new request
    parameter int WAKE_TIMEOUT_CYCLES = 1_000_000,  // WAKE low with no INTN -> wr_err
    parameter int MAX_RX_LEN          = 32767,      // longer packets are cut (rest arrives as continuation)
    parameter bit WRITE_RX_DELIVER    = 1'b1        // deliver complete packets received during a write
) (
    input  logic        clk,
    input  logic        rst,

    input  logic        intn_s,       // synchronized INTN, active low
    output logic        wake_n = 1'b1,

    // write request (1 packet). Payload bytes are fetched by index.
    input  logic        wr_start,     // 1-cycle pulse; may arrive mid-read (queued until idle)
    input  logic [7:0]  wr_chan,
    input  logic [7:0]  wr_len,       // whole packet incl. header, >= 5
    output logic [7:0]  wr_idx,       // payload byte wanted (0 = first byte after header)
    input  logic [7:0]  wr_byte,      // combinational from wr_idx
    output logic        wr_done,      // 1-cycle pulse, after the CS gap
    output logic        wr_err,       // valid with wr_done: WAKE timed out, nothing sent

    // received payload stream
    output logic        rx_valid,
    output logic [7:0]  rx_data,
    output logic        rx_sop,
    output logic        rx_eop,
    output logic [7:0]  rx_chan,
    output logic        rx_cont,

    // per-transfer header info (1-cycle pulse after header byte 3)
    output logic        hdr_valid,
    output logic [14:0] hdr_len,
    output logic        hdr_cont,
    output logic        hdr_in_write,
    output logic        rx_trunc,     // packet longer than the transfer (write, or > MAX_RX_LEN)

    // spimaster
    output logic        spi_start,
    output logic [7:0]  spi_tx,
    output logic        spi_hold_cs,
    input  logic [7:0]  spi_rx,
    input  logic        spi_rx_valid,
    input  logic        spi_cs_n      // CS pin as driven by spimaster
);

    if (MAX_RX_LEN < 5 || MAX_RX_LEN > 32767) begin : g_chk_maxlen
        $error("bno085_shtp: MAX_RX_LEN (%0d) must be 5..32767", MAX_RX_LEN);
    end

    typedef enum logic [2:0] {IDLE, WAKE_WAIT, SEND, BYTE_WAIT, GAP} state_t;

    state_t      state;
    logic        is_wr;
    logic        wr_pend;      // write requested, not started yet
    logic [15:0] idx;          // byte of the current transfer, 0..n-1
    logic [15:0] n;            // bytes to clock in this transfer
    logic [15:0] d_end;        // deliver payload bytes 4 .. d_end-1
    logic        deliver;
    logic [7:0]  b0;           // rx header byte 0 (len LSB)
    logic [7:0]  w_chan, w_len;
    logic [7:0]  seq [8];      // tx sequence number per channel
    int unsigned timer;

    // ---- INTN request detection ----
    logic        armed;
    int unsigned low_cnt;
    logic        xfer_begin;
    wire         intn_ready = !intn_s && (armed || low_cnt >= INTN_STALE_CYCLES);

    always_ff @(posedge clk) begin
        if (rst) begin
            armed   <= 1'b1;   // INTN already low at release: read once (null header is harmless)
            low_cnt <= 0;
        end else begin
            if (intn_s) begin
                armed   <= 1'b1;
                low_cnt <= 0;
            end else if (low_cnt < INTN_STALE_CYCLES) begin
                low_cnt <= low_cnt + 1;
            end
            if (xfer_begin) begin
                armed   <= 1'b0;
                low_cnt <= 0;
            end
        end
    end

    // ---- header decode of the byte just received as byte 1 ----
    wire [15:0] raw_len  = {spi_rx, b0};
    wire        len_bad  = (raw_len == 16'hFFFF);          // SHTP: 65535 = error
    wire [14:0] len15    = len_bad ? 15'd0 : raw_len[14:0];
    wire        cont_bit = raw_len[15] && !len_bad;

    assign wr_idx = 8'(idx - 16'd4);

    // byte to send and CS hold for byte idx
    logic [7:0] tx_byte;
    logic       tx_hold;
    always_comb begin
        if (is_wr) begin
            case (idx)
                16'd0:   tx_byte = w_len;
                16'd1:   tx_byte = 8'h00;              // len MSB = 0, no continuation
                16'd2:   tx_byte = w_chan;
                16'd3:   tx_byte = seq[w_chan[2:0]];
                default: tx_byte = wr_byte;
            endcase
            tx_hold = (idx != n - 1);
        end else begin
            tx_byte = 8'h00;
            tx_hold = (idx < 3) ? 1'b1 : (idx != n - 1);  // n is known by byte 2
        end
    end

    always_ff @(posedge clk) begin
        spi_start  <= 1'b0;
        wr_done    <= 1'b0;
        rx_valid   <= 1'b0;
        rx_sop     <= 1'b0;
        rx_eop     <= 1'b0;
        hdr_valid  <= 1'b0;
        rx_trunc   <= 1'b0;
        xfer_begin <= 1'b0;

        if (rst) begin
            state        <= IDLE;
            wake_n       <= 1'b1;
            is_wr        <= 1'b0;
            idx          <= '0;
            n            <= 16'd4;
            d_end        <= '0;
            deliver      <= 1'b0;
            b0           <= '0;
            w_chan       <= '0;
            w_len        <= '0;
            wr_pend      <= 1'b0;
            timer        <= 0;
            wr_err       <= 1'b0;
            spi_tx       <= '0;
            spi_hold_cs  <= 1'b0;
            rx_data      <= '0;
            rx_chan      <= '0;
            rx_cont      <= 1'b0;
            hdr_len      <= '0;
            hdr_cont     <= 1'b0;
            hdr_in_write <= 1'b0;
            for (int c = 0; c < 8; c++) seq[c] <= '0;
        end else begin
            // WAKE goes back high as soon as CS is low (HAL order)
            if (!wake_n && !spi_cs_n) wake_n <= 1'b1;

            // queue a write request (the controller may ask while a read runs)
            if (wr_start) begin
                wr_pend <= 1'b1;
                w_chan  <= wr_chan;
                w_len   <= wr_len;
            end

            case (state)

                IDLE:
                    if (wr_pend) begin
                        wr_pend <= 1'b0;
                        wake_n  <= 1'b0;
                        timer  <= 0;
                        state  <= WAKE_WAIT;
                    end else if (intn_ready) begin
                        is_wr      <= 1'b0;
                        n          <= 16'd4;
                        idx        <= '0;
                        xfer_begin <= 1'b1;
                        state      <= SEND;
                    end

                WAKE_WAIT:
                    if (intn_ready) begin
                        is_wr      <= 1'b1;
                        n          <= 16'(w_len);
                        idx        <= '0;
                        xfer_begin <= 1'b1;
                        state      <= SEND;
                    end else if (timer >= WAKE_TIMEOUT_CYCLES) begin
                        wake_n  <= 1'b1;
                        wr_err  <= 1'b1;
                        wr_done <= 1'b1;
                        state   <= IDLE;
                    end else begin
                        timer <= timer + 1;
                    end

                SEND: begin
                    spi_tx      <= tx_byte;
                    spi_hold_cs <= tx_hold;
                    spi_start   <= 1'b1;
                    state       <= BYTE_WAIT;
                end

                // wait on rx_valid, not busy (busy rises a cycle late)
                BYTE_WAIT:
                    if (spi_rx_valid) begin
                        case (idx)
                            16'd0: b0 <= spi_rx;
                            16'd1: begin
                                hdr_len  <= len15;
                                hdr_cont <= cont_bit;
                                rx_cont  <= cont_bit;
                                if (is_wr) begin
                                    deliver  <= WRITE_RX_DELIVER && len15 > 4 && 16'(len15) <= n;
                                    d_end    <= 16'(len15);
                                    rx_trunc <= len15 > 4 && 16'(len15) > n;
                                end else begin
                                    n        <= (len15 <= 4) ? 16'd4 :
                                                (len15 > MAX_RX_LEN) ? 16'(MAX_RX_LEN) : 16'(len15);
                                    deliver  <= len15 > 4;
                                    d_end    <= (len15 > MAX_RX_LEN) ? 16'(MAX_RX_LEN) : 16'(len15);
                                    rx_trunc <= len15 > MAX_RX_LEN;
                                end
                            end
                            16'd2: rx_chan <= spi_rx;
                            16'd3: begin
                                hdr_valid    <= 1'b1;
                                hdr_in_write <= is_wr;
                            end
                            default:
                                if (deliver && idx < d_end) begin
                                    rx_valid <= 1'b1;
                                    rx_data  <= spi_rx;
                                    rx_sop   <= (idx == 16'd4);
                                    rx_eop   <= (idx == d_end - 1);
                                end
                        endcase

                        if (idx == n - 1) begin
                            if (is_wr) seq[w_chan[2:0]] <= seq[w_chan[2:0]] + 1;
                            timer <= 0;
                            state <= GAP;              // CS is high from this cycle
                        end else begin
                            idx   <= idx + 1;
                            state <= SEND;
                        end
                    end

                GAP:
                    if (timer >= CS_GAP_CYCLES) begin
                        if (is_wr) begin
                            wr_err  <= 1'b0;
                            wr_done <= 1'b1;
                        end
                        is_wr <= 1'b0;
                        state <= IDLE;
                    end else begin
                        timer <= timer + 1;
                    end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
