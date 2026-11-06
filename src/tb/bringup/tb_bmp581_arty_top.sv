`timescale 1ns/1ps
`default_nettype none

// Testbench for bmp581_arty_top at the Arty S7's 12 MHz clock: top + BMP581 model.
//   default : init LED on, beat LED blinks, fault LED off; then a sensor reset
//             -> fault LED latches on, init LED comes back, beat resumes
//   -DP_OUT : pressure window set so real readings are "implausible"
//             -> init LED on, beat LED must never toggle
module tb_bmp581_arty_top;
    localparam int CLK_HZ  = 12_000_000;
    localparam int ODR_CYC = CLK_HZ / 5_000;  // model: one sample per 200 us
    logic clk = 0;
    always #41.667 clk = ~clk;                // 12 MHz

    wire cs_n, sclk, mosi, miso, bmp_int;
    pullup   (miso);
    pulldown (bmp_int);
    logic [3:0] led;
    logic btn_rst = 1'b0;
    wire led_init = led[0], led_fault = led[1], led_beat = led[2], led_raw = led[3];

`ifdef P_OUT
    localparam logic [23:0] PMAX = 24'(50_000 * 64);
`else
    localparam logic [23:0] PMAX = 24'(110_000 * 64);
`endif

    bmp581_arty_top #(
        .CLK_HZ(CLK_HZ), .BEAT_SAMPLES(5), .P_MAX_RAW(PMAX),
        .PWRUP_WAIT_CYC(360), .SOFTRST_WAIT_CYC(360), .STANDBY_WAIT_CYC(360),   // 30 us
        .CS_IDLE_CYC(24), .DRDY_TIMEOUT_CYC(5 * ODR_CYC), .RETRY_WAIT_CYC(600),
        .STALE_CYC(ODR_CYC / 2)
    ) dut (
        .clk(clk), .btn_rst(btn_rst),
        .bmp_cs_n(cs_n), .bmp_sclk(sclk), .bmp_mosi(mosi), .bmp_miso(miso), .bmp_int(bmp_int),
        .led(led)
    );

    bmp581_model #(.CHIP_ID(8'h50), .PWRUP_T(20us), .SOFTRST_T(20us), .ODR_PERIOD(200us)) u_bmp (
        .csb(cs_n), .sck(sclk), .sdi(mosi), .sdo(miso), .int_pin(bmp_int)
    );

    integer errors = 0, toggles = 0, rtoggles = 0, wc = 0;
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
        `WAIT_FOR("led_init", 200_000, led_init, 1'b1)
`ifdef P_OUT
        t0 = toggles; wc = rtoggles;
        repeat (30 * ODR_CYC) @(posedge clk);
        if (toggles != t0) begin $display("ERROR: beat toggled with implausible pressure"); errors++; end
        else $display("LD4 stayed dark for 30 samples with pressure outside the window: ok");
        if (rtoggles - wc < 5) begin $display("ERROR: raw beat LD5 not blinking (%0d)", rtoggles - wc); errors++; end
        else $display("LD5 still blinking (%0d toggles): samples are flowing: ok", rtoggles - wc);
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

        $display("--- pressing BTN0 (reset)");
        btn_rst = 1'b1; repeat (50) @(posedge clk); btn_rst = 1'b0;
        repeat (10) @(posedge clk);
        if (led_fault !== 1'b0 || led_init !== 1'b0) begin $display("ERROR: reset did not clear LEDs"); errors++; end
        `WAIT_FOR("led_init after button reset", 50 * ODR_CYC, led_init, 1'b1)
        if (led_fault !== 1'b0) begin $display("ERROR: fault LED on after clean re-init"); errors++; end
`endif
        errors += u_bmp.errors;
        $display("model protocol errors: %0d", u_bmp.errors);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED (%0d errors)", errors);
        $finish;
    end
    initial begin #200ms; $display("ERROR: global timeout"); $finish; end
endmodule
`default_nettype wire
