`timescale 1ns/1ps
`default_nettype none

// Full bring-up design against a fake ZED-F9P: bytes go in on MISO and must
// come out on the UART, through the real gnss_scc, raw_byte_tap, and uart_tx.
// The parameters are scaled down so a "second" and a UART frame are short; the
// logic is otherwise identical to the hardware build.
module tb_tangnano20k_top;
    localparam int CLK_HZ           = 20_000;
    localparam int BAUD             = 2_000;
    localparam int CLKS_PER_BIT     = CLK_HZ / BAUD;
    localparam int SPI_CLOCK_DIVIDE = 4;
    localparam int FF_KEEP          = 32; // raw_byte_tap's default

    logic       clk_27m;
    logic       gnss_sclk, gnss_mosi, gnss_cs_n;
    logic       gnss_miso = 1'b1;
    logic       gnss_timepulse = 1'b0;
    logic       uart_tx;
    logic [5:0] led_n;

    // clk_27m is owned entirely by this one process (no declaration initializer), which
    // keeps Verilator PROCASSINIT quiet without a waiver on any Verilator version
    initial begin
        clk_27m = 1'b0;
        forever #5 clk_27m = ~clk_27m;
    end

    tangnano20k_top #(
        .CLK_HZ           (CLK_HZ),
        .BAUD             (BAUD),
        .POR_WIDTH        (4),
        .SPI_CLOCK_DIVIDE (SPI_CLOCK_DIVIDE)
    ) dut (
        .clk_27m,
        .gnss_sclk, .gnss_mosi, .gnss_miso, .gnss_cs_n, .gnss_timepulse,
        .uart_tx, .led_n
    );

    // Fake ZED-F9P, SPI mode 0: sends queued bytes, then 0xFF when it has nothing to say.
    byte to_send[$];
    logic [7:0] current_byte = 8'hFF;
    int bit_index = 0;
    int mosi_low_count = 0;

    function automatic void load_next_byte();
        current_byte = (to_send.size() > 0) ? to_send.pop_front() : 8'hFF;
        bit_index = 0;
        gnss_miso = current_byte[7];
    endfunction

    initial begin
        automatic logic previous_cs = 1'b1;
        automatic logic previous_sclk = 1'b0;
        forever begin
            @(gnss_cs_n or gnss_sclk);
            if (gnss_cs_n != previous_cs) begin
                previous_cs = gnss_cs_n;
                if (!gnss_cs_n) load_next_byte();
            end
            if (gnss_sclk != previous_sclk) begin
                previous_sclk = gnss_sclk;
                if (gnss_sclk) begin
                    if (gnss_mosi !== 1'b1) mosi_low_count++;
                end else if (!gnss_cs_n) begin
                    bit_index++;
                    if (bit_index == 8) load_next_byte();
                    else gnss_miso = current_byte[7 - bit_index];
                end
            end
        end
    end

    // Host side UART model: samples the middle of every bit, same as tb_uart_tx.
    byte received[$];
    initial begin
        byte b;
        forever begin
            @(negedge uart_tx);
            repeat (CLKS_PER_BIT / 2) @(posedge clk_27m);
            assert (uart_tx == 1'b0) else $fatal(1, "start bit not low at mid-bit");
            for (int i = 0; i < 8; i++) begin
                repeat (CLKS_PER_BIT) @(posedge clk_27m);
                b[i] = uart_tx;
            end
            repeat (CLKS_PER_BIT) @(posedge clk_27m);
            assert (uart_tx == 1'b1) else $fatal(1, "stop bit not high at mid-bit");
            received.push_back(b);
        end
    end

    // Send a burst from the fake receiver, then require it on the UART followed by
    // exactly FF_KEEP filler bytes and then silence.
    task automatic check_burst(byte burst[], string label);
        int expected_count = burst.size() + FF_KEEP;
        int timeout = 0;

        foreach (burst[i]) to_send.push_back(burst[i]);
        while (received.size() < expected_count) begin
            @(posedge clk_27m);
            timeout++;
            assert (timeout < 20 * CLKS_PER_BIT * (expected_count + 2))
                else $fatal(1, "%s: timeout, %0d of %0d bytes", label, received.size(), expected_count);
        end
        repeat (15 * CLKS_PER_BIT) @(posedge clk_27m);

        assert (received.size() == expected_count)
            else $fatal(1, "%s: %0d bytes, expected %0d", label, received.size(), expected_count);
        foreach (burst[i])
            assert (received[i] == burst[i])
                else $fatal(1, "%s byte %0d: got %02x, expected %02x", label, i, received[i], burst[i]);
        for (int i = burst.size(); i < expected_count; i++)
            assert (received[i] == 8'hFF)
                else $fatal(1, "%s trailing byte %0d = %02x", label, i, received[i]);
        received.delete();
    endtask

    // watchdog: a stuck design should fail, not hang
    initial begin
        repeat (200 * CLK_HZ) @(posedge clk_27m);
        $fatal(1, "watchdog timeout");
    end

    initial begin
        // What the receiver sends by default: NMEA text
        static byte nmea[] = '{"$", "G", "N", "G", "G", "A", 8'h0D, 8'h0A};
        // A UBX frame with 0xFF in its payload, which must survive intact
        static byte ubx[] = '{8'hB5, 8'h62, 8'h01, 8'h07, 8'h04, 8'h00, 8'hFF, 8'hFF, 8'h12, 8'hFF, 8'h34, 8'h56};
        logic heartbeat_before, pps_before;

        // Power-up: LEDs off, UART idle, and polling starts with only filler reaching nothing
        repeat (8) @(posedge clk_27m);
        #1;
        assert (uart_tx == 1'b1) else $fatal(1, "uart_tx not idle-high at power-up");
        assert (led_n == 6'b111111) else $fatal(1, "LEDs not all off at power-up");
        heartbeat_before = led_n[0];

        repeat (200 * SPI_CLOCK_DIVIDE) @(posedge clk_27m);
        assert (gnss_cs_n == 1'b0) else $fatal(1, "not polling after reset");
        assert (received.size() == 0) else $fatal(1, "filler reached the UART");

        check_burst(nmea, "NMEA");
        check_burst(ubx, "UBX");
        assert (mosi_low_count == 0) else $fatal(1, "MOSI low during polling");
        assert (led_n[2] == 1'b1) else $fatal(1, "FIFO overflow LED on during a normal run");

        // TIMEPULSE edge toggles LED4
        pps_before = led_n[4];
        gnss_timepulse = 1'b1;
        repeat (10) @(posedge clk_27m);
        gnss_timepulse = 1'b0;
        repeat (5) @(posedge clk_27m);
        #1;
        assert (led_n[4] != pps_before) else $fatal(1, "TIMEPULSE did not toggle LED4");

        // Heartbeat toggles once per scaled second
        wait (led_n[0] != heartbeat_before);

        $display("PASS tb_tangnano20k_top");
        $finish;
    end
endmodule

`default_nettype wire
