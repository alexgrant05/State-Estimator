`timescale 1ns/1ps
`default_nettype none

// Runs all four SPI modes side by side from one shared command stream, each
// against its own peripheral model. The peripheral is written from the SPI
// definition (leading/trailing edges), not from spi_master's own code, so it
// checks the DUT against the protocol rather than against itself.
module tb_spi_master;
    localparam int CLOCK_DIVIDE = 8;
    localparam int HALF_PERIOD_NS = 10 * CLOCK_DIVIDE / 2; // 10 ns clock
    localparam int NUM_BYTES = 4;
    localparam int NUM_BITS = 8 * NUM_BYTES;
    localparam logic [NUM_BITS-1:0] MASTER_TX = 32'hA500_FF3C;
    localparam logic [NUM_BITS-1:0] PERIPHERAL_TX = 32'h5AFF_00C3;

    logic clk;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic [7:0] tx_data = 8'h00;
    logic hold_cs = 1'b0;
    logic restart = 1'b1; // peripheral models rewind their streams on the next CS fall

    logic [3:0] busy, rx_valid, cs_n, sclk;
    logic [7:0] rx_data [4];
    logic [NUM_BITS-1:0] sampled [4]; // MOSI as seen by each peripheral
    int sclk_edges [4];

    // clk is owned entirely by this one process (no declaration initializer), which
    // keeps Verilator PROCASSINIT quiet without a waiver on any Verilator version
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    for (genvar m = 0; m < 4; m++) begin : g_mode
        localparam int CPOL = (m >> 1) & 1;
        localparam int CPHA = m & 1;

        logic mosi;
        logic miso = 1'b1;

        spi_master #(
            .CLOCK_DIVIDE (CLOCK_DIVIDE),
            .CPOL         (CPOL),
            .CPHA         (CPHA)
        ) dut (
            .sys_clk  (clk),
            .rst      (rst_n),
            .start,
            .tx_data,
            .hold_cs,
            .busy     (busy[m]),
            .rx_data  (rx_data[m]),
            .rx_valid (rx_valid[m]),
            .cs_n     (cs_n[m]),
            .sclk     (sclk[m]),
            .mosi,
            .miso
        );

        // Peripheral model. drive_index/sample_index count bits across the whole
        // stream, so a held CS and a re-asserted CS use the same logic.
        function automatic logic peripheral_bit(int n);
            return (n < NUM_BITS) ? PERIPHERAL_TX[NUM_BITS - 1 - n] : 1'b1;
        endfunction

        initial begin
            automatic int drive_index = 0;
            automatic int sample_index = 0;
            automatic logic previous_cs = 1'b1;
            automatic logic previous_sclk = 1'(CPOL);
            automatic int edges_this_run = 0;
            automatic time last_edge = 0;
            automatic bit leading;

            sclk_edges[m] = 0;
            sampled[m] = '0;
            forever begin
                @(cs_n[m] or sclk[m]);
                if (cs_n[m] != previous_cs) begin
                    previous_cs = cs_n[m];
                    edges_this_run = 0;
                    if (!cs_n[m]) begin
                        if (restart) begin
                            sample_index = 0;
                            sampled[m] = '0;
                            sclk_edges[m] = 0;
                        end
                        // CPHA 0: first bit must be valid before the first clock edge
                        drive_index = sample_index;
                        if (CPHA == 0) begin
                            miso = peripheral_bit(drive_index);
                            drive_index++;
                        end
                    end
                end
                if (sclk[m] != previous_sclk) begin
                    previous_sclk = sclk[m];
                    assert (!cs_n[m]) else $fatal(1, "mode %0d: SCLK moved with CS high", m);
                    sclk_edges[m]++;
                    if ((edges_this_run % 16) != 0)
                        assert ($time - last_edge == 64'(HALF_PERIOD_NS))
                            else $fatal(1, "mode %0d: half period %0t", m, $time - last_edge);
                    edges_this_run++;
                    last_edge = $time;

                    leading = (sclk[m] != 1'(CPOL));
                    if (leading == (CPHA == 0)) begin
                        // sample edge: CPHA 0 leading, CPHA 1 trailing
                        if (sample_index < NUM_BITS) sampled[m][NUM_BITS - 1 - sample_index] = mosi;
                        sample_index++;
                    end else begin
                        // drive edge: CPHA 0 trailing, CPHA 1 leading
                        miso = peripheral_bit(drive_index);
                        drive_index++;
                    end
                end
            end
        end
    end

    // One byte: start pulse, wait for completion, check the received byte in every mode
    task automatic transfer(int index, bit hold);
        logic [7:0] expected;
        expected = PERIPHERAL_TX[NUM_BITS - 1 - 8 * index -: 8];

        @(negedge clk);
        tx_data = MASTER_TX[NUM_BITS - 1 - 8 * index -: 8];
        hold_cs = hold;
        start   = 1'b1;
        @(negedge clk);
        start   = 1'b0;
        tx_data = 8'h00; // the DUT must have latched its inputs at the start pulse
        hold_cs = !hold;
        assert (busy == 4'b1111) else $fatal(1, "busy did not rise, busy=%b", busy);
        assert (cs_n == 4'b0000) else $fatal(1, "cs_n did not fall, cs_n=%b", cs_n);

        while (busy != 4'b0000) begin
            assert (rx_valid == 4'b0000) else $fatal(1, "rx_valid pulsed during a transfer");
            @(negedge clk);
        end

        assert (rx_valid == 4'b1111) else $fatal(1, "rx_valid missing, rx_valid=%b", rx_valid);
        for (int m = 0; m < 4; m++)
            assert (rx_data[m] == expected)
                else $fatal(1, "mode %0d byte %0d: received %02x, expected %02x", m, index, rx_data[m], expected);
        assert (cs_n == (hold ? 4'b0000 : 4'b1111))
            else $fatal(1, "cs_n wrong after byte, hold=%0d cs_n=%b", hold, cs_n);
        assert (sclk == 4'b1100) else $fatal(1, "SCLK not at its idle level, sclk=%b", sclk);

        @(negedge clk);
        assert (rx_valid == 4'b0000) else $fatal(1, "rx_valid was not a one-cycle pulse");
    endtask

    task automatic check_stream(string label);
        // let the peripheral processes observe the final CS edge
        #1;
        for (int m = 0; m < 4; m++) begin
            assert (sampled[m] == MASTER_TX)
                else $fatal(1, "%s mode %0d: peripheral got %08x, expected %08x", label, m, sampled[m], MASTER_TX);
            assert (sclk_edges[m] == 16 * NUM_BYTES)
                else $fatal(1, "%s mode %0d: %0d SCLK edges", label, m, sclk_edges[m]);
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk);
        assert (cs_n == 4'b1111) else $fatal(1, "cs_n not high in reset");
        rst_n = 1'b1;
        repeat (3) @(negedge clk);
        assert (cs_n == 4'b1111 && busy == 4'b0000) else $fatal(1, "not idle after reset");
        assert (sclk == 4'b1100) else $fatal(1, "SCLK idle level does not match CPOL");

        // Scenario 1: CS held low across all bytes, released on the last (burst transfer)
        restart = 1'b1;
        for (int i = 0; i < NUM_BYTES; i++) begin
            transfer(i, i != NUM_BYTES - 1);
            restart = 1'b0;
        end
        check_stream("held CS");

        // Scenario 2: CS released after every byte
        restart = 1'b1;
        for (int i = 0; i < NUM_BYTES; i++) begin
            transfer(i, 1'b0);
            restart = 1'b0;
        end
        check_stream("per-byte CS");

        $display("PASS tb_spi_master");
        $finish;
    end
endmodule

`default_nettype wire
