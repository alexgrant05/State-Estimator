`timescale 1ns/1ps
`default_nettype none

// Uses the real board parameters (27 MHz, 115200 baud -> 234 clocks/bit) so
// the exact divisor that runs on hardware is the one being checked.
module tb_uart_tx;
    localparam int CLK_HZ       = 27_000_000;
    localparam int BAUD         = 115_200;
    localparam int CLKS_PER_BIT = CLK_HZ / BAUD;

    logic clk;
    logic rst_n = 1'b0;
    logic [7:0] in_data = 8'h00;
    logic in_valid = 1'b0;
    logic in_ready;
    logic tx;

    // clk is owned entirely by this one process (no declaration initializer), which
    // keeps Verilator PROCASSINIT quiet without a waiver on any Verilator version
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    uart_tx #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (BAUD)
    ) dut (
        .clk, .rst_n, .in_data, .in_valid, .in_ready, .tx
    );

    byte sent[$];
    byte received[$];

    // Receiver model: on a start-bit falling edge, step to the middle of each bit
    // and sample. Sampling mid-bit means tx is never changing on the sampled edge.
    initial begin
        byte b;
        int  i;
        forever begin
            @(negedge tx);
            repeat (CLKS_PER_BIT / 2) @(posedge clk);
            assert (tx == 1'b0) else $fatal(1, "start bit not low at mid-bit");
            for (i = 0; i < 8; i++) begin
                repeat (CLKS_PER_BIT) @(posedge clk);
                b[i] = tx;
            end
            repeat (CLKS_PER_BIT) @(posedge clk);
            assert (tx == 1'b1) else $fatal(1, "stop bit not high at mid-bit");
            received.push_back(b);
        end
    end

    // Offer a byte at a negedge, wait for in_ready, and return just after the
    // handshake edge. Returns the number of clocks until in_ready came back,
    // which must be exactly one 10-bit frame.
    task automatic send_byte(byte b, output int frame_clocks);
        @(negedge clk);
        in_data  = b;
        in_valid = 1'b1;
        while (!in_ready) @(negedge clk);
        @(posedge clk); #1;
        in_valid = 1'b0;
        sent.push_back(b);
        frame_clocks = 0;
        assert (!in_ready) else $fatal(1, "in_ready stayed high after a handshake");
        while (!in_ready) begin
            @(posedge clk); #1;
            frame_clocks++;
        end
    endtask

    initial begin
        static byte pattern[] = '{8'h55, 8'h00, 8'hFF, 8'hA5, 8'h0D, 8'h0A};
        int frame_clocks;
        int i;

        repeat (3) @(posedge clk);
        assert (tx == 1'b1) else $fatal(1, "tx not idle-high during reset");
        @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        #1;
        assert (tx == 1'b1 && in_ready) else $fatal(1, "not idle and ready after reset");

        // back-to-back bytes, including all-0 / all-1 / alternating patterns
        for (i = 0; i < pattern.size(); i++) begin
            send_byte(pattern[i], frame_clocks);
            assert (frame_clocks == 10 * CLKS_PER_BIT)
                else $fatal(1, "frame took %0d clocks, expected %0d", frame_clocks, 10 * CLKS_PER_BIT);
        end

        // an idle gap, then one more byte: the line must sit high in between
        repeat (3 * CLKS_PER_BIT) begin
            @(posedge clk); #1;
            assert (tx == 1'b1) else $fatal(1, "tx not idle-high between frames");
        end
        send_byte(8'h3C, frame_clocks);

        // let the receiver model finish sampling the final stop bit
        repeat (CLKS_PER_BIT) @(posedge clk);

        assert (received.size() == sent.size())
            else $fatal(1, "received %0d bytes, sent %0d", received.size(), sent.size());
        for (i = 0; i < sent.size(); i++)
            assert (received[i] == sent[i])
                else $fatal(1, "byte %0d: received %02x, sent %02x", i, received[i], sent[i]);

        $display("PASS tb_uart_tx");
        $finish;
    end
endmodule

`default_nettype wire
