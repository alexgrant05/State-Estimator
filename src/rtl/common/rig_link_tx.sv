// ============================================================================
// rig_link_tx: v2 frame sender for the FPGA -> Pico link
//
//   [source / arbiter] --start, src, type, len--> [rig_link_tx] --> [spimaster] --SPI--> Pico
//                      <--byte_idx / byte_in----
//                      <--done-------------------
//
// Frame (docs/rig_link_protocol.md):
//   A5 5A (0x20|src) type seq len payload[len] crc8
// seq is a link-wide counter kept here; crc8 (poly 0x07, init 0) covers
// everything before it. Payload bytes are fetched one at a time: byte_idx
// selects payload byte 0..len-1 and the requester answers combinationally
// on byte_in. It must keep the payload stable until done.
//
// Handshake: pulse start for 1 cycle while idle (busy low). done pulses for
// 1 cycle after the frame is out and CS has been high for GAP_CYCLES.
// Link: mode 3, CS low for the whole frame (needed by the Pico's PL022).
// ============================================================================

module rig_link_tx #(
    parameter int CLK_DIV    = 50,     // SCLK = clk / CLK_DIV (100 MHz / 50 = 2 MHz)
    parameter int GAP_CYCLES = 200,    // CS high between frames
    parameter int MAX_LEN    = 48      // payload bytes, at most
) (
    input  logic       clk,
    input  logic       rst,

    input  logic       start,          // 1-cycle pulse while !busy
    input  logic [3:0] src,
    input  logic [7:0] type_id,
    input  logic [5:0] len,
    output logic [5:0] byte_idx,       // payload byte wanted
    input  logic [7:0] byte_in,
    output logic       busy,
    output logic       done,           // 1-cycle pulse

    output logic       cs_n,
    output logic       sclk,
    output logic       mosi
);

    if (CLK_DIV < 2 || CLK_DIV % 2 != 0) begin : g_chk_div
        $error("rig_link_tx: CLK_DIV (%0d) must be even and >= 2", CLK_DIV);
    end
    if (MAX_LEN < 1 || MAX_LEN > 63) begin : g_chk_len
        $error("rig_link_tx: MAX_LEN (%0d) must be 1..63", MAX_LEN);
    end

    function automatic logic [7:0] crc8_step(input logic [7:0] crc, input logic [7:0] d);
        logic [7:0] c;
        c = crc ^ d;
        for (int i = 0; i < 8; i++)
            c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
        return c;
    endfunction

    typedef enum logic [1:0] {IDLE, SEND, WAIT, GAP} state_t;

    state_t      state;
    logic [3:0]  src_r;
    logic [7:0]  type_r, seq, crc;
    logic [5:0]  len_r;
    logic [6:0]  idx;              // frame byte, 0 .. len+6
    int unsigned gap;

    wire [6:0] last_idx = 7'(len_r) + 7'd6;   // CRC byte
    assign byte_idx = 6'(idx - 7'd6);
    assign busy     = (state != IDLE);

    logic [7:0] cur_byte;
    always_comb begin
        case (idx)
            7'd0:    cur_byte = 8'hA5;
            7'd1:    cur_byte = 8'h5A;
            7'd2:    cur_byte = {4'h2, src_r};
            7'd3:    cur_byte = type_r;
            7'd4:    cur_byte = seq;
            7'd5:    cur_byte = {2'b00, len_r};
            default: cur_byte = (idx == last_idx) ? crc : byte_in;
        endcase
    end

    logic       spi_start, spi_hold_cs, spi_rx_valid;
    logic [7:0] spi_tx;
    /* verilator lint_off UNUSEDSIGNAL */
    logic       spi_busy;
    logic [7:0] spi_rx;            // one-way link
    /* verilator lint_on UNUSEDSIGNAL */

    always_ff @(posedge clk) begin
        spi_start <= 1'b0;
        done      <= 1'b0;

        if (rst) begin
            state       <= IDLE;
            src_r       <= '0;
            type_r      <= '0;
            len_r       <= '0;
            seq         <= '0;
            crc         <= '0;
            idx         <= '0;
            gap         <= 0;
            spi_tx      <= '0;
            spi_hold_cs <= 1'b0;
        end else begin
            case (state)
                IDLE:
                    if (start) begin
                        src_r  <= src;
                        type_r <= type_id;
                        len_r  <= (len > 6'(MAX_LEN)) ? 6'(MAX_LEN) : len;
                        idx    <= '0;
                        crc    <= '0;
                        state  <= SEND;
                    end

                SEND: begin
                    spi_tx      <= cur_byte;
                    spi_hold_cs <= (idx != last_idx);     // release CS after the CRC
                    spi_start   <= 1'b1;
                    crc         <= crc8_step(crc, cur_byte);
                    state       <= WAIT;
                end

                // wait on rx_valid, not busy (busy rises a cycle late)
                WAIT:
                    if (spi_rx_valid) begin
                        if (idx == last_idx) begin
                            seq   <= seq + 1;
                            gap   <= 0;
                            state <= GAP;
                        end else begin
                            idx   <= idx + 1;
                            state <= SEND;
                        end
                    end

                GAP:
                    if (gap >= GAP_CYCLES) begin
                        done  <= 1'b1;
                        state <= IDLE;
                    end else
                        gap <= gap + 1;
            endcase
        end
    end

    spimaster #(.CLOCK_DIVIDE(CLK_DIV), .CPOL(1), .CPHA(1)) u_spi (
        .sys_clk(clk), .rst,
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold_cs),
        .busy(spi_busy), .rx_data(spi_rx), .rx_valid(spi_rx_valid),
        .cs_n, .sclk, .mosi, .miso(1'b0)
    );

endmodule
