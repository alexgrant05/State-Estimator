// ============================================================================
// spimaster: single-byte SPI master, MSB first, any of the 4 SPI modes.
//
// Handshake (unchanged since the original version):
//   * Pulse start for 1 cycle while idle. tx_data and hold_cs are latched then.
//   * CS falls on the cycle after start. rx_valid pulses for 1 cycle when the
//     byte is done, with rx_data holding the received byte.
//   * hold_cs is per byte: 1 = leave CS low after this byte (more bytes follow
//     in the same frame), 0 = raise CS at the end of this byte.
//   * Wait for rx_valid before the next start. busy is high from the cycle
//     after start until rx_valid; it rises a cycle late, so don't use it to
//     detect that a start was accepted.
//
// Timing, in sys_clk cycles, with H = HALF_CLOCK_PERIOD = CLOCK_DIVIDE / 2:
//   start seen ............ CS falls (next cycle)
//   CS falls -> 1st edge .. H          (same for every byte; see FIX below)
//   edge -> edge .......... H          (16 edges per byte)
//   last edge -> rx_valid . H          (CS rises here too, unless hold_cs)
//
// Waveform per mode (edges counted 0..15 from the first edge after CS falls):
//   CPHA = 0 (modes 0, 2): MOSI bit 7 is driven when CS falls. Even edges
//     (leading) sample MISO; odd edges (trailing) shift out the next MOSI bit.
//   CPHA = 1 (modes 1, 3): even edges (leading) drive the next MOSI bit; odd
//     edges (trailing) sample MISO. In mode 3 (CPOL = 1) SCLK idles high,
//     leading = falling, trailing = rising: the slave drives MISO on the
//     falling edge and both sides sample on the rising edge.
//   SCLK ends each byte back at CPOL.
//
// MISO is sampled on the same sys_clk edge that toggles the SCLK register, so
// the actual sample point is 1 sys_clk before the SCLK edge appears on the pin.
// That is still ~H cycles after the slave changed MISO. miso is not
// synchronized here; a design that wants a synchronizer adds it outside (it
// moves the effective sample point a few cycles earlier, still well inside H).
//
// FIX (this revision): the half-period counter was not cleared when a byte
// finished. It was left at H-1, so every byte after the first one following
// reset had its first SCLK edge 1 sys_clk after CS fell instead of H. It is
// now cleared when start is accepted, so every byte has identical timing.
// cs_n also powers up high (FPGA init value), before rst is ever applied.
// ============================================================================

module spimaster #(
    parameter int CLOCK_DIVIDE = 100,  // SCLK = sys_clk / CLOCK_DIVIDE (100 MHz / 100 = 1 MHz)
    parameter int CPOL         = 0,    // SCLK idle level
    parameter int CPHA         = 0     // 0: sample on leading edge, 1: on trailing edge
) (
    input  logic       sys_clk,
    input  logic       rst,            // synchronous, active high

    // from fabric
    input  logic       start,          // 1-cycle pulse while idle
    input  logic [7:0] tx_data,
    input  logic       hold_cs,        // keep CS low after this byte

    // to fabric
    output logic       busy,
    output logic [7:0] rx_data,
    output logic       rx_valid,       // 1-cycle pulse per byte

    // pins
    output logic       cs_n = 1'b1,    // deselected from configuration onward
    output logic       sclk,
    output logic       mosi,
    input  logic       miso
);

    localparam int HALF_CLOCK_PERIOD = CLOCK_DIVIDE / 2;

    // ------------------------------------------------------------------------
    // Parameter checks (elaboration time)
    // ------------------------------------------------------------------------
    if (CLOCK_DIVIDE < 2 || CLOCK_DIVIDE % 2 != 0) begin : g_chk_div
        $error("spimaster: CLOCK_DIVIDE (%0d) must be even and >= 2", CLOCK_DIVIDE);
    end
    if (HALF_CLOCK_PERIOD > 65536) begin : g_chk_div_max
        $error("spimaster: CLOCK_DIVIDE (%0d) too large for the 16-bit half-period counter", CLOCK_DIVIDE);
    end
    if (CPOL != 0 && CPOL != 1) begin : g_chk_cpol
        $error("spimaster: CPOL (%0d) must be 0 or 1", CPOL);
    end
    if (CPHA != 0 && CPHA != 1) begin : g_chk_cpha
        $error("spimaster: CPHA (%0d) must be 0 or 1", CPHA);
    end

    typedef enum logic [1:0] {
        IDLE = 2'b00,
        TXN  = 2'b01
    } state_t;

    state_t      current_state = IDLE;
    logic [7:0]  tx_data_reg;
    logic [7:0]  rx_data_reg;
    logic        hold_cs_reg;
    logic        sclk_internal = 1'(CPOL);
    logic [15:0] clks_since_last_sclk_edge = '0;   // 0 .. H-1
    logic [5:0]  sclk_edge_count = '0;             // edges done in this byte, 0 .. 16

    assign busy = (current_state == TXN);
    assign sclk = sclk_internal;

    always_ff @(posedge sys_clk) begin
        if (rst) begin
            current_state             <= IDLE;
            sclk_internal             <= 1'(CPOL);
            tx_data_reg               <= '0;
            rx_data_reg               <= '0;
            clks_since_last_sclk_edge <= '0;
            sclk_edge_count           <= '0;
            hold_cs_reg               <= 1'b0;
            rx_data                   <= '0;
            rx_valid                  <= 1'b0;
            cs_n                      <= 1'b1;
            mosi                      <= 1'b0;
        end

        else if (current_state == IDLE) begin
            rx_valid <= 1'b0;
            if (start) begin
                tx_data_reg               <= tx_data;
                hold_cs_reg               <= hold_cs;
                cs_n                      <= 1'b0;
                mosi                      <= tx_data[7];   // CPHA = 0 needs bit 7 before the 1st edge
                clks_since_last_sclk_edge <= '0;           // FIX: full half period to the 1st edge
                current_state             <= TXN;
            end
        end

        else begin  // TXN
            if (clks_since_last_sclk_edge != 16'(HALF_CLOCK_PERIOD - 1)) begin
                // between edges
                clks_since_last_sclk_edge <= clks_since_last_sclk_edge + 1;
                rx_valid                  <= 1'b0;
            end
            else if (sclk_edge_count == 16) begin
                // half a period after the last edge: byte done, SCLK is at CPOL
                sclk_edge_count <= '0;
                cs_n            <= ~hold_cs_reg;
                current_state   <= IDLE;
                rx_valid        <= 1'b1;
                rx_data         <= rx_data_reg;
            end
            else begin
                // SCLK edge number sclk_edge_count (0..15)
                rx_valid                  <= 1'b0;
                sclk_internal             <= ~sclk_internal;
                clks_since_last_sclk_edge <= '0;
                sclk_edge_count           <= sclk_edge_count + 1;

                if (CPHA == 0) begin
                    // even edge: sample. odd edge 2k+1: drive bit 6-k (k = 0..6);
                    // edge 15 drives nothing (bit 0 is already out).
                    if (sclk_edge_count[0] == 1'b0)     rx_data_reg <= {rx_data_reg[6:0], miso};
                    else if (sclk_edge_count[5:1] < 7)  mosi <= tx_data_reg[3'(6 - sclk_edge_count[5:1])];
                end else begin
                    // even edge 2k: drive bit 7-k. odd edge: sample.
                    if (sclk_edge_count[0] == 1'b0)     mosi <= tx_data_reg[3'(7 - sclk_edge_count[5:1])];
                    else                                rx_data_reg <= {rx_data_reg[6:0], miso};
                end
            end
        end
    end

endmodule
