`timescale 1ns/1ps
`default_nettype none

// gnss_spi_engine
//
// Owns the single spi_master instance shared by the ZED-F9P SCC and
// arbitrates it between two callers: gnss_config_sequencer's byte-wide
// write commands (rare, at wake/reconfigure) and this module's own
// autonomous continuous read-poll (steady state). Config writes always
// pre-empt polling at the next byte boundary.
//
// The ZED-F9P tolerates being polled continuously with cs_n held low,
// returning 0xFF idle filler when no message byte is pending -- this
// engine relies on that (see u-blox Interface Description [2], SPI
// section) rather than gating each byte on TX_READY. ubx_frame_sync
// discards filler naturally since it never matches the 0xB5 0x62 sync
// sequence.
//
// Inputs:  clk, rst_n, miso, cfg_cmd_data/valid/last, poll_enable
// Outputs: sclk, mosi, cs_n, cfg_cmd_ready, rx_byte/rx_byte_valid, engine_busy
module gnss_spi_engine #(
    parameter int CLOCK_DIVIDE = 100
) (
    input  logic       clk,
    input  logic       rst_n,

    // physical SPI pins (ZED-F9P is SPI peripheral; FPGA is SPI master)
    output logic       sclk,
    output logic       mosi,
    input  logic       miso,
    output logic       cs_n,

    // config-write command stream, one byte at a time (from gnss_config_sequencer)
    input  logic [7:0] cfg_cmd_data,
    input  logic       cfg_cmd_valid,
    output logic       cfg_cmd_ready,
    input  logic       cfg_cmd_last,     // marks final byte of a config message

    // continuous read-poll enable (driven by gnss_scc; typically tied high)
    input  logic       poll_enable,

    // received byte stream (to ubx_frame_sync; includes 0xFF idle filler,
    // filtered downstream by the sync-byte matcher)
    output logic [7:0] rx_byte,
    output logic       rx_byte_valid,

    output logic       engine_busy
);

    typedef enum logic [1:0] {
        S_IDLE     = 2'b00,
        S_CFG_XFER = 2'b01,
        S_POLL     = 2'b10
    } state_t;

    state_t state;

    logic       spi_start;
    logic [7:0] spi_tx_data;
    logic       spi_hold_cs;
    logic       spi_busy;
    logic [7:0] spi_rx_data;
    logic       spi_rx_valid;

    spi_master #(
        .CLOCK_DIVIDE (CLOCK_DIVIDE),
        .CPOL         (0),
        .CPHA         (0)
    ) u_spi_master (
        .sys_clk  (clk),
        .rst      (rst_n),
        .start    (spi_start),
        .tx_data  (spi_tx_data),
        .hold_cs  (spi_hold_cs),
        .busy     (spi_busy),
        .rx_data  (spi_rx_data),
        .rx_valid (spi_rx_valid),
        .cs_n     (cs_n),
        .sclk     (sclk),
        .mosi     (mosi),
        .miso     (miso)
    );

    always_comb engine_busy = (state != S_IDLE) || spi_busy;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            spi_start     <= 1'b0;
            spi_tx_data   <= 8'h00;
            spi_hold_cs   <= 1'b0;
            cfg_cmd_ready <= 1'b0;
            rx_byte       <= 8'h00;
            rx_byte_valid <= 1'b0;
        end else begin
            spi_start     <= 1'b0;
            cfg_cmd_ready <= 1'b0;
            rx_byte_valid <= 1'b0;

            if (spi_rx_valid) begin
                rx_byte       <= spi_rx_data;
                rx_byte_valid <= (state == S_POLL);
            end

            case (state)
                S_IDLE: begin
                    spi_hold_cs <= 1'b0;
                    if (cfg_cmd_valid) begin
                        spi_tx_data   <= cfg_cmd_data;
                        spi_hold_cs   <= !cfg_cmd_last;
                        spi_start     <= 1'b1;
                        cfg_cmd_ready <= 1'b1;
                        state         <= S_CFG_XFER;
                    end else if (poll_enable) begin
                        spi_tx_data <= 8'hFF;
                        spi_hold_cs <= 1'b1;
                        spi_start   <= 1'b1;
                        state       <= S_POLL;
                    end
                end

                S_CFG_XFER: begin
                    if (!spi_busy && !spi_start) begin
                        if (cfg_cmd_valid) begin
                            spi_tx_data   <= cfg_cmd_data;
                            spi_hold_cs   <= !cfg_cmd_last;
                            spi_start     <= 1'b1;
                            cfg_cmd_ready <= 1'b1;
                        end else begin
                            state <= S_IDLE; // cs_n release already handled by spi_master
                        end
                    end
                end

                S_POLL: begin
                    if (!spi_busy && !spi_start) begin
                        if (cfg_cmd_valid) begin
                            // config write pre-empts polling at the next byte boundary
                            spi_hold_cs <= 1'b0;
                            state       <= S_IDLE;
                        end else if (poll_enable) begin
                            spi_tx_data <= 8'hFF;
                            spi_hold_cs <= 1'b1;
                            spi_start   <= 1'b1;
                        end else begin
                            spi_hold_cs <= 1'b0;
                            state       <= S_IDLE;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
