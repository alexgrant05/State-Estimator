`timescale 1ns/1ps
`default_nettype none

// Runs the board top with a scaled-down "second" (CLK_HZ = 100k clocks) and
// 10 clocks per UART bit so several messages fit in a short simulation. The
// logic is identical to the hardware build; only the parameters shrink.
module tb_tangnano20k_top;
    localparam int CLK_HZ       = 100_000;
    localparam int BAUD         = 10_000;
    localparam int CLKS_PER_BIT = CLK_HZ / BAUD;

    logic       clk_27m;
    logic       uart_tx;
    logic [5:0] led_n;

    // clk_27m is owned entirely by this one process (no declaration initializer), which
    // keeps Verilator PROCASSINIT quiet without a waiver on any Verilator version
    initial begin
        clk_27m = 1'b0;
        forever #5 clk_27m = ~clk_27m;
    end

    tangnano20k_top #(
        .CLK_HZ    (CLK_HZ),
        .BAUD      (BAUD),
        .POR_WIDTH (4)
    ) dut (
        .clk_27m, .uart_tx, .led_n
    );

    // Receiver model, same as tb_uart_tx: sample in the middle of every bit.
    byte received[$];
    initial begin
        byte b;
        int  i;
        forever begin
            @(negedge uart_tx);
            repeat (CLKS_PER_BIT / 2) @(posedge clk_27m);
            assert (uart_tx == 1'b0) else $fatal(1, "start bit not low at mid-bit");
            for (i = 0; i < 8; i++) begin
                repeat (CLKS_PER_BIT) @(posedge clk_27m);
                b[i] = uart_tx;
            end
            repeat (CLKS_PER_BIT) @(posedge clk_27m);
            assert (uart_tx == 1'b1) else $fatal(1, "stop bit not high at mid-bit");
            received.push_back(b);
        end
    end

    // Pops one line (through LF) off the received queue as a string.
    task automatic wait_for_line(output string line);
        byte b;
        line = "";
        do begin
            wait (received.size() > 0);
            b = received.pop_front();
            line = {line, string'(b)};
        end while (b != 8'h0A);
    endtask

    // watchdog: a stuck design should fail, not hang
    initial begin
        repeat (10 * CLK_HZ) @(posedge clk_27m);
        $fatal(1, "timeout waiting for UART output");
    end

    initial begin
        string line;
        logic  heartbeat_before;

        // nothing should be sent, and all LEDs should be off, while held in reset
        repeat (8) @(posedge clk_27m);
        #1;
        assert (uart_tx == 1'b1) else $fatal(1, "uart_tx not idle-high out of reset");
        assert (led_n == 6'b111111) else $fatal(1, "LEDs not all off at power-up");

        heartbeat_before = led_n[0];

        wait_for_line(line);
        assert (line == "tang nano hello 00\r\n") else $fatal(1, "first line wrong: '%s'", line);
        assert (led_n[0] != heartbeat_before) else $fatal(1, "heartbeat LED did not toggle");

        wait_for_line(line);
        assert (line == "tang nano hello 01\r\n") else $fatal(1, "second line wrong: '%s'", line);

        wait_for_line(line);
        assert (line == "tang nano hello 02\r\n") else $fatal(1, "third line wrong: '%s'", line);

        $display("PASS tb_tangnano20k_top");
        $finish;
    end
endmodule

`default_nettype wire
