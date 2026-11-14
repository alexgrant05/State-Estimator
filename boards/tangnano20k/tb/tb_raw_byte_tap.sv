`timescale 1ns/1ps
`default_nettype none

module tb_raw_byte_tap;
    localparam int FIFO_DEPTH = 8;
    localparam int FF_KEEP = 4;

    logic clk;
    logic rst_n = 1'b0;
    logic [7:0] rx_byte = 8'h00;
    logic rx_byte_valid = 1'b0;
    logic [7:0] out_data;
    logic out_valid;
    logic out_ready = 1'b1;
    logic overflow_sticky;

    // clk is owned entirely by this one process (no declaration initializer), which
    // keeps Verilator PROCASSINIT quiet without a waiver on any Verilator version
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    raw_byte_tap #(
        .FIFO_DEPTH (FIFO_DEPTH),
        .FF_KEEP    (FF_KEEP)
    ) dut (
        .clk, .rst_n, .rx_byte, .rx_byte_valid,
        .out_data, .out_valid, .out_ready, .overflow_sticky
    );

    byte received[$];

    initial begin
        forever begin
            @(posedge clk);
            if (out_valid && out_ready) received.push_back(out_data);
        end
    end

    task automatic send_byte(byte b);
        @(negedge clk);
        rx_byte       = b;
        rx_byte_valid = 1'b1;
        @(negedge clk);
        rx_byte_valid = 1'b0;
    endtask

    task automatic send_filler(int count);
        repeat (count) send_byte(8'hFF);
    endtask

    // Send a byte list and require exactly the expected list to come out
    task automatic check_output(byte expected[], string label);
        repeat (4) @(negedge clk);
        assert (received.size() == expected.size())
            else $fatal(1, "%s: got %0d bytes, expected %0d", label, received.size(), expected.size());
        foreach (expected[i])
            assert (received[i] == expected[i])
                else $fatal(1, "%s byte %0d: got %02x, expected %02x", label, i, received[i], expected[i]);
        received.delete();
    endtask

    initial begin
        static byte none[] = '{};
        static byte real_bytes[] = '{8'hB5, 8'h62, 8'h01};
        static byte short_run[] = '{8'h11, 8'hFF, 8'hFF, 8'h22};
        static byte kept_run[] = '{8'h33, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'h44};
        static byte truncated_run[] = '{8'h55, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'h66};

        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // Filler from power-up never reaches the host
        send_filler(20);
        check_output(none, "startup filler");

        // Real bytes pass through in order
        foreach (real_bytes[i]) send_byte(real_bytes[i]);
        check_output(real_bytes, "real bytes");

        // After real bytes, filler is kept up to FF_KEEP and then dropped
        send_filler(10);
        check_output('{8'hFF, 8'hFF, 8'hFF, 8'hFF}, "filler after a burst");

        // Back to idle: more filler stays dropped
        send_filler(10);
        check_output(none, "continued filler");

        // 0xFF inside a message survives, which is the reason this isn't a plain filter
        foreach (short_run[i]) send_byte(short_run[i]);
        check_output(short_run, "short 0xFF run");
        // the two 0xFFs above must have reset by the 0x22, so a full FF_KEEP run is kept again
        foreach (kept_run[i]) send_byte(kept_run[i]);
        check_output(kept_run, "run of exactly FF_KEEP");

        // A longer run is cut to FF_KEEP. This is the documented limit of the heuristic.
        send_byte(8'h55);
        send_filler(FF_KEEP + 3);
        send_byte(8'h66);
        check_output(truncated_run, "run longer than FF_KEEP");

        // Stalled consumer: bytes wait in the FIFO, none lost, while it isn't full
        assert (!overflow_sticky) else $fatal(1, "overflow set before the FIFO filled");
        @(negedge clk);
        out_ready = 1'b0;
        for (int i = 0; i < FIFO_DEPTH; i++) send_byte(byte'(32'h80 + i));
        repeat (4) @(negedge clk);
        assert (!overflow_sticky) else $fatal(1, "overflow set at exactly full");

        // One byte too many: dropped and flagged, and the earlier bytes stay intact
        send_byte(8'h7E);
        repeat (2) @(negedge clk);
        assert (overflow_sticky) else $fatal(1, "overflow not flagged");
        out_ready = 1'b1;
        repeat (FIFO_DEPTH + 4) @(negedge clk);
        assert (received.size() == FIFO_DEPTH)
            else $fatal(1, "drained %0d bytes, expected %0d", received.size(), FIFO_DEPTH);
        for (int i = 0; i < FIFO_DEPTH; i++)
            assert (received[i] == byte'(32'h80 + i))
                else $fatal(1, "drain byte %0d = %02x", i, received[i]);

        $display("PASS tb_raw_byte_tap");
        $finish;
    end
endmodule

`default_nettype wire
