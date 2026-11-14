`timescale 1ns/1ps
`default_nettype none

// Polling path only, which is what the receiver sees until config writes are
// enabled. The config-write path is deliberately not covered yet: its known
// issues (MISO bytes dropped during writes, no CS rise at the poll-to-config
// handoff) need fixing first, and a test here would only pin the wrong behavior.
module tb_gnss_spi_engine;
    localparam int CLOCK_DIVIDE = 4;

    logic clk;
    logic rst_n = 1'b0;
    logic poll_enable = 1'b0;

    logic sclk, mosi, cs_n;
    logic miso = 1'b1;
    logic cfg_cmd_ready;
    logic [7:0] rx_byte;
    logic rx_byte_valid;
    logic engine_busy;

    // clk is owned entirely by this one process (no declaration initializer), which
    // keeps Verilator PROCASSINIT quiet without a waiver on any Verilator version
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    gnss_spi_engine #(
        .CLOCK_DIVIDE (CLOCK_DIVIDE)
    ) dut (
        .clk, .rst_n,
        .sclk, .mosi, .miso, .cs_n,
        .cfg_cmd_data  (8'h00),
        .cfg_cmd_valid (1'b0),
        .cfg_cmd_ready,
        .cfg_cmd_last  (1'b0),
        .poll_enable,
        .rx_byte, .rx_byte_valid, .engine_busy
    );

    // Fake ZED-F9P, SPI mode 0: sends queued bytes MSB-first, then 0xFF when it
    // has nothing to say, as the integration manual describes.
    byte to_send[$];
    logic [7:0] current_byte = 8'hFF;
    int bit_index = 0;
    int mosi_low_count = 0;

    int cs_rise_count = 0;

    function automatic void load_next_byte();
        current_byte = (to_send.size() > 0) ? to_send.pop_front() : 8'hFF;
        bit_index = 0;
        miso = current_byte[7];
    endfunction

    initial begin
        automatic logic previous_cs = 1'b1;
        automatic logic previous_sclk = 1'b0;
        forever begin
            @(cs_n or sclk);
            if (cs_n != previous_cs) begin
                previous_cs = cs_n;
                if (cs_n) cs_rise_count++;
                else load_next_byte();
            end
            if (sclk != previous_sclk) begin
                previous_sclk = sclk;
                if (sclk) begin
                    // polling must write only 0xFF, so the receiver's parser stays idle
                    if (mosi !== 1'b1) mosi_low_count++;
                end else if (!cs_n) begin
                    bit_index++;
                    if (bit_index == 8) load_next_byte();
                    else miso = current_byte[7 - bit_index];
                end
            end
        end
    end

    byte received[$];
    int cfg_ready_count = 0;

    initial begin
        forever begin
            @(posedge clk);
            if (rx_byte_valid) received.push_back(rx_byte);
            if (cfg_cmd_ready) cfg_ready_count++;
        end
    end

    task automatic wait_for_bytes(int count);
        int timeout = 0;
        while (received.size() < count) begin
            @(negedge clk);
            timeout++;
            assert (timeout < 100 * CLOCK_DIVIDE * (count + 1))
                else $fatal(1, "timeout: %0d of %0d bytes received", received.size(), count);
        end
    endtask

    initial begin
        // Sample frame with 0xFF inside the payload: the engine must not treat it specially.
        static byte frame[] = '{8'hB5, 8'h62, 8'h01, 8'h07, 8'h03, 8'h00, 8'hFF, 8'h00, 8'hFF, 8'h0C, 8'h2D};
        int first;

        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // Polling disabled: the bus must stay completely idle
        repeat (10 * CLOCK_DIVIDE) begin
            @(negedge clk);
            assert (cs_n == 1'b1 && sclk == 1'b0 && !engine_busy && !rx_byte_valid)
                else $fatal(1, "bus not idle with polling disabled");
        end

        // Polling enabled: filler flows, with no real data yet
        poll_enable = 1'b1;
        wait_for_bytes(3);
        @(negedge clk);
        assert (cs_n == 1'b0) else $fatal(1, "cs_n not low while polling");
        assert (engine_busy) else $fatal(1, "engine_busy low while polling");
        for (int i = 0; i < 3; i++)
            assert (received[i] == 8'hFF) else $fatal(1, "filler byte %0d = %02x", i, received[i]);

        // The receiver starts talking mid-stream; every byte must arrive in order
        first = received.size();
        foreach (frame[i]) to_send.push_back(frame[i]);
        wait_for_bytes(first + frame.size() + 4);

        // Find the frame (the byte in flight when it was queued still reads as filler)
        while (received[first] == 8'hFF) first++;
        foreach (frame[i])
            assert (received[first + i] == frame[i])
                else $fatal(1, "frame byte %0d: received %02x, expected %02x", i, received[first + i], frame[i]);
        assert (received[first + frame.size()] == 8'hFF) else $fatal(1, "no filler after the frame");

        assert (mosi_low_count == 0) else $fatal(1, "MOSI low on %0d sampled bits while polling", mosi_low_count);
        assert (cs_rise_count == 0) else $fatal(1, "CS rose %0d times while polling", cs_rise_count);
        assert (cfg_ready_count == 0) else $fatal(1, "cfg_cmd_ready pulsed with no config traffic");

        $display("PASS tb_gnss_spi_engine");
        $finish;
    end
endmodule

`default_nettype wire
