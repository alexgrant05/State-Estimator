`timescale 1ns/1ps
`default_nettype none

// Testbench for bmp581_tang20k_top at the Tang Nano 20K's 27 MHz clock: top + BMP581 model.
//   default : init LED on, beat LED blinks, fault LED off; then a sensor reset
//             -> fault LED latches on, init LED comes back, beat resumes
//   -DP_OUT : pressure window set so real readings are "implausible"
//             -> init LED on, beat LED must never toggle
module tb_bmp581_tang20k_top;
    localparam int CLK_HZ  = 27_000_000;
    localparam int ODR_CYC = CLK_HZ / 5_000;  // model: one sample per 200 us
    logic clk = 0;
    always #18.5185 clk = ~clk;               // 27 MHz

    wire cs_n, sclk, mosi, miso, bmp_int;
    pullup   (miso);
    pulldown (bmp_int);
    wire [5:0] led_n;
    logic btn_rst = 1'b0;
    wire led_init = ~led_n[0], led_fault = ~led_n[1], led_beat = ~led_n[2], led_raw = ~led_n[3];
    wire led_spi  = ~led_n[4], led_alive = ~led_n[5];
    wire utx;                                 // serial line from the design
    integer errors = 0, toggles = 0, rtoggles = 0, atoggles = 0, wc = 0;

`ifdef BAD_CHIPID
    localparam logic [7:0] CHIPID = 8'h51;
`else
    localparam logic [7:0] CHIPID = 8'h50;
`endif
`ifdef P_OUT
    localparam logic [23:0] PMAX = 24'(50_000 * 64);
`else
    localparam logic [23:0] PMAX = 24'(110_000 * 64);
`endif

    bmp581_tang20k_top #(
        .CLK_HZ(CLK_HZ), .BEAT_SAMPLES(5), .P_MAX_RAW(PMAX),
        .PWRUP_WAIT_CYC(810), .SOFTRST_WAIT_CYC(810), .STANDBY_WAIT_CYC(810),   // 30 us
        .CS_IDLE_CYC(54), .DRDY_TIMEOUT_CYC(5 * ODR_CYC), .RETRY_WAIT_CYC(1620),
        .ALIVE_HALF_CYC(2700), .SPI_LED_CYC(2700),                                 // 100 us each
        .STALE_CYC(ODR_CYC / 2), .REPORT_CYC(40_000)                               // ~1.5 ms
    ) dut (
        .clk(clk), .btn_rst(btn_rst),
        .bmp_cs_n(cs_n), .bmp_sclk(sclk), .bmp_mosi(mosi), .bmp_miso(miso), .bmp_int(bmp_int),
        .led_n(led_n), .uart_tx(utx)
    );

    // ---- UART decoder: 115200 8N1, prints every line the design sends
    localparam realtime BIT_NS = 234 * (1.0e9 / CLK_HZ);
    string line = "", seen_lines = "";
    integer n_lines = 0;
    always begin : uart_rx
        logic [7:0] b;
        @(negedge utx);
        #(BIT_NS * 1.5);
        for (int i = 0; i < 8; i++) begin b[i] = utx; #(BIT_NS); end
        if (utx !== 1'b1) begin $display("ERROR: UART stop bit missing"); errors++; end
        if (b == 8'h0A) begin
            n_lines++;
            if (n_lines <= 3 || n_lines % 4 == 0) $display("[%t] serial: %s", $realtime, line);
            seen_lines = {seen_lines, line, "|"};
            line = "";
        end else if (b != 8'h0D) line = {line, string'(b)};
    end
    function automatic bit has(string s);
        for (int i = 0; i + s.len() <= seen_lines.len(); i++)
            if (seen_lines.substr(i, i + s.len() - 1) == s) return 1;
        return 0;
    endfunction

    bmp581_model #(.CHIP_ID(CHIPID), .PWRUP_T(20us), .SOFTRST_T(20us), .ODR_PERIOD(200us)) u_bmp (
        .csb(cs_n), .sck(sclk), .sdi(mosi), .sdo(miso), .int_pin(bmp_int)
    );

    always @(led_alive) if ($time > 0) atoggles++;
    always @(led_beat) if ($time > 0) toggles++;
    always @(led_raw)  if ($time > 0) rtoggles++;

    // wait until SIG == VAL or MAX cycles pass (macro: Icarus has no ref ports)
    `define WAIT_FOR(WHAT, MAX, SIG, VAL) \
        begin \
            wc = 0; \
            while (SIG !== VAL && wc < MAX) begin @(posedge clk); wc++; end \
            if (SIG !== VAL) begin $display("ERROR: %s never became %b", WHAT, VAL); errors++; end \
            else $display("[%t] %s = %b", $realtime, WHAT, VAL); \
        end

    initial begin : main
        integer t0;
        $timeformat(-6, 1, " us", 10);
`ifdef BAD_CHIPID
        // bring-up must fail with code 1 and the LEDs must blink it
        repeat (40 * ODR_CYC) @(posedge clk);
        begin : code_chk
            integer on_cnt, on0, on_other;
            on0 = 0; on_other = 0; on_cnt = 0;
            repeat (10 * 2700) begin
                @(posedge clk);
                if (!led_n[0]) on0++;
                if (!led_n[1] || !led_n[2] || !led_n[3]) on_other++;
            end
            $display("code display: LED0 lit %0d cycles, LED1-3 lit %0d cycles, controller fault_code = %0d",
                     on0, on_other, dut.fault_code);
            if (dut.fault_code != 4'd1) begin $display("ERROR: expected fault code 1"); errors++; end
            if (on0 < 10_000 || on0 > 17_000) begin $display("ERROR: LED0 not blinking about half the time"); errors++; end
            if (on_other != 0) begin $display("ERROR: LED1-3 should be dark for code 1"); errors++; end
        end
        if (!has("ID=51 INIT=0 CODE=1")) begin $display("ERROR: serial line with ID=51 CODE=1 not seen"); errors++; end
        else $display("serial reports the wrong chip ID 0x51 and code 1: ok");
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED (%0d errors)", errors);
        $finish;
`endif
        `WAIT_FOR("led_init", 200_000, led_init, 1'b1)
`ifdef P_OUT
        t0 = toggles; wc = rtoggles;
        repeat (30 * ODR_CYC) @(posedge clk);
        if (toggles != t0) begin $display("ERROR: beat toggled with implausible pressure"); errors++; end
        else $display("LED2 stayed dark for 30 samples with pressure outside the window: ok");
        if (rtoggles - wc < 5) begin $display("ERROR: raw beat LED3 not blinking (%0d)", rtoggles - wc); errors++; end
        else $display("LED3 still blinking (%0d toggles): samples are flowing: ok", rtoggles - wc);
        if (led_fault !== 1'b0) begin $display("ERROR: fault LED on"); errors++; end
`else
        t0 = toggles;
        repeat (32 * ODR_CYC) @(posedge clk);   // 32 samples -> 6 toggles at 5 samples/toggle
        $display("beat toggles in 32 sample periods: %0d (expect 6)", toggles - t0);
        if (toggles - t0 < 5 || toggles - t0 > 7) begin $display("ERROR: beat rate wrong"); errors++; end
        if (led_fault !== 1'b0) begin $display("ERROR: fault LED on in normal run"); errors++; end

        $display("--- injecting sensor reset");
        u_bmp.inject_por();
        `WAIT_FOR("led_fault", 20 * ODR_CYC, led_fault, 1'b1)
        `WAIT_FOR("led_init after recovery", 50 * ODR_CYC, led_init, 1'b1)
        t0 = toggles;
        repeat (20 * ODR_CYC) @(posedge clk);
        if (toggles - t0 < 3) begin $display("ERROR: beat did not resume (%0d)", toggles - t0); errors++; end
        else $display("beat resumed: %0d toggles", toggles - t0);
        if (led_fault !== 1'b1) begin $display("ERROR: fault LED did not stay latched"); errors++; end

        $display("--- pressing S1 (reset)");
        btn_rst = 1'b1; repeat (50) @(posedge clk); btn_rst = 1'b0;
        repeat (10) @(posedge clk);
        if (led_fault !== 1'b0 || led_init !== 1'b0) begin $display("ERROR: reset did not clear LEDs"); errors++; end
        `WAIT_FOR("led_init after button reset", 50 * ODR_CYC, led_init, 1'b1)
        if (led_fault !== 1'b0) begin $display("ERROR: fault LED on after clean re-init"); errors++; end
`endif
        // LED5 alive blink: 2700-cycle half period -> about 1 toggle per 100 us
        $display("LED5 alive toggles so far: %0d", atoggles);
        if (atoggles < 50) begin $display("ERROR: alive LED not blinking"); errors++; end
        // LED4 should be on while samples are read (one read per 200 us, hold 100 us)
        wc = 0;
        repeat (4 * ODR_CYC) begin @(posedge clk); if (led_spi) wc++; end
        $display("LED4 SPI-activity on for %0d%% of 4 sample periods", wc * 100 / (4 * ODR_CYC));
        if (wc == 0) begin $display("ERROR: SPI activity LED never on"); errors++; end
        // SCLK rate: measure one period
        begin : sclk_chk
            realtime t1, t2;
            @(negedge cs_n); @(negedge sclk); t1 = $realtime; @(negedge sclk); t2 = $realtime;
            $display("SCLK period %.1f ns = %.1f kHz (expect 1037 ns, 964 kHz)", (t2 - t1), 1.0e6 / (t2 - t1));
            if ((t2 - t1) < 1000.0) begin $display("ERROR: SCLK faster than 1 MHz"); errors++; end
        end
        $display("serial lines received: %0d", n_lines);
        if (!has("ID=50 INIT=1 CODE=0")) begin $display("ERROR: no serial line with ID=50 INIT=1"); errors++; end
        if (!has(" P=6000")) begin $display("ERROR: no pressure value in the serial lines"); errors++; end
        errors += u_bmp.errors;
        $display("model protocol errors: %0d", u_bmp.errors);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED (%0d errors)", errors);
        $finish;
    end
    initial begin #200ms; $display("ERROR: global timeout"); $finish; end
endmodule
`default_nettype wire
