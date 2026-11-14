`timescale 1ns/1ps
`default_nettype none

// Self-checking testbench: spimaster (mode 3, 1 MHz) + bmp581_controller + BMP581 model.
// Variants:  default  -> bring-up, 20 samples, timeout recovery, POR recovery x2
//            -DBAD_CHIPID -> expect FLT_CHIP_ID and retries
//            -DBAD_OSR    -> expect FLT_OSR_ODR
module tb_bmp581;

    localparam int CLK_PERIOD = 10;          // 100 MHz
    localparam int ODR_CYC    = 20_000;      // model ODR period 200 us

`ifdef BAD_OSR
    localparam logic [2:0] TB_OSR_P = 3'b110;  // x64, invalid at this ODR in the model
`else
    localparam logic [2:0] TB_OSR_P = 3'b011;  // x8
`endif
`ifdef FAST_FIRST
    localparam bit TB_FAST = 1;
`else
    localparam bit TB_FAST = 0;
`endif
`ifdef BAD_CHIPID
    localparam logic [7:0] TB_CHIP_ID = 8'h51;
`else
    localparam logic [7:0] TB_CHIP_ID = 8'h50;
`endif

    logic clk = 0, rst = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    logic [63:0] ts = 0;
    always @(posedge clk) ts <= ts + 1;

    // spimaster <-> controller
    logic       spi_start, spi_hold_cs, spi_busy, spi_rx_valid;
    logic [7:0] spi_tx_data, spi_rx_data;
    // pins
    wire cs_n, sclk, mosi, miso, bmp_int;
    pullup   (miso);        // undriven SDO reads 0xFF, like a pulled-up shared MISO
    pulldown (bmp_int);     // INT is high-Z until INT_CONFIG.int_en = 1

    // simple arbiter: random grant latency, never revokes while req is high
    logic bus_req, bus_gnt = 0, starve = 0;
    always @(posedge clk) begin
        if (!bus_req || starve)               bus_gnt <= 1'b0;
        else if (!bus_gnt && ($urandom % 8 == 0)) bus_gnt <= 1'b1;
    end

    logic        sample_valid, init_done, fault;
    logic [7:0]  sample_sensor_id, fault_count;
    logic [15:0] sample_seq, drdy_timeout_count, overrun_count;
    logic [63:0] sample_timestamp;
    logic [23:0] sample_press_raw, sample_temp_raw;
    logic [7:0]  sample_flags;
    logic [3:0]  fault_code;

    spimaster #(.CLOCK_DIVIDE(100), .CPOL(1), .CPHA(1)) u_spi (
        .sys_clk(clk), .rst(rst),
        .start(spi_start), .tx_data(spi_tx_data), .hold_cs(spi_hold_cs),
        .busy(spi_busy), .rx_data(spi_rx_data), .rx_valid(spi_rx_valid),
        .cs_n(cs_n), .sclk(sclk), .mosi(mosi), .miso(miso)
    );

    bmp581_controller #(
        .CLK_HZ(100_000_000), .SENSOR_ID(8'd2), .OSR_P(TB_OSR_P),
        .PWRUP_WAIT_CYC(3_000), .SOFTRST_WAIT_CYC(3_000), .STANDBY_WAIT_CYC(3_000),
        .CS_IDLE_CYC(200), .DRDY_TIMEOUT_CYC(5 * ODR_CYC), .RETRY_WAIT_CYC(5_000),
        .STALE_CYC(ODR_CYC / 2)
    ) dut (
        .sys_clk(clk), .rst(rst), .timestamp_now(ts), .bmp_int(bmp_int),
        .bus_req(bus_req), .bus_gnt(bus_gnt),
        .spi_start(spi_start), .spi_tx_data(spi_tx_data), .spi_hold_cs(spi_hold_cs),
        .spi_busy(spi_busy), .spi_rx_data(spi_rx_data), .spi_rx_valid(spi_rx_valid),
        .sample_valid(sample_valid), .sample_sensor_id(sample_sensor_id), .sample_seq(sample_seq),
        .sample_timestamp(sample_timestamp), .sample_press_raw(sample_press_raw),
        .sample_temp_raw(sample_temp_raw), .sample_flags(sample_flags),
        .init_done(init_done), .fault(fault), .fault_code(fault_code), .fault_count(fault_count),
        .drdy_timeout_count(drdy_timeout_count), .overrun_count(overrun_count)
    );

    bmp581_model #(.CHIP_ID(TB_CHIP_ID), .PWRUP_T(20us), .SOFTRST_T(20us), .ODR_PERIOD(200us), .FAST_FIRST(TB_FAST)) u_bmp (
        .csb(cs_n), .sck(sclk), .sdi(mosi), .sdo(miso), .int_pin(bmp_int)
    );

    // ---------------- checks ----------------
    integer errors = 0, n_samples = 0, n_stale = 0;
    logic [63:0] int_rise_ts [0:4095];
    logic [15:0] last_seq = 0;

    always @(posedge bmp_int) int_rise_ts[u_bmp.k % 4096] = ts;

    always @(posedge clk) if (sample_valid) begin : chk
        integer kk; longint dt;
        kk = (sample_press_raw - 24'h600000) / 3;
        dt = sample_timestamp - int_rise_ts[kk % 4096];
        if (sample_flags[4]) begin
            // stale: data is a newer sample than the timestamp; expected only in phase 5
            n_stale++;
            dt = sample_timestamp - int_rise_ts[(kk - 1) % 4096];
            $display("[%0t] stale sample flagged: data k=%0d, timestamp is from k=%0d (INT+%0d)", $time, kk, kk - 1, dt);
        end
        if (sample_press_raw !== u_bmp.log_press[kk % 4096] ||
            sample_temp_raw  !== u_bmp.log_temp[kk % 4096]) begin
            $display("[%0t] ERROR: data mismatch k=%0d P=%06h T=%06h", $time, kk, sample_press_raw, sample_temp_raw);
            errors++;
        end
        if (!sample_flags[4] && (dt < 1 || dt > 4)) begin
            $display("[%0t] ERROR: timestamp offset %0d cycles from INT rise (k=%0d)", $time, dt, kk);
            errors++;
        end
        if ((sample_flags & 8'b1110_1111) !== 8'h00) begin
            $display("[%0t] ERROR: flags=%b", $time, sample_flags); errors++;
        end
        if (sample_seq !== last_seq + 1) begin
            $display("[%0t] ERROR: seq jump %0d -> %0d", $time, last_seq, sample_seq); errors++;
        end
        if (sample_sensor_id !== 8'd2) begin
            $display("[%0t] ERROR: sensor id %0d", $time, sample_sensor_id); errors++;
        end
        last_seq = sample_seq;
        n_samples++;
        if (n_samples <= 3 || n_samples % 10 == 0)
            $display("[%0t] sample seq=%0d k=%0d P=0x%06h T=0x%06h ts=%0d (INT+%0d)",
                     $time, sample_seq, kk, sample_press_raw, sample_temp_raw, sample_timestamp, dt);
    end

    // a garbage frame must never be emitted
    always @(posedge clk) if (sample_valid && sample_press_raw == 24'hFFFFFF) begin
        $display("[%0t] ERROR: garbage sample emitted", $time); errors++;
    end

    task automatic wait_samples(input integer n, input integer max_cyc);
        integer start, c;
        start = n_samples; c = 0;
        while (n_samples < start + n && c < max_cyc) begin @(posedge clk); c++; end
        if (n_samples < start + n) begin
            $display("[%0t] ERROR: timed out waiting for %0d samples (got %0d)", $time, n, n_samples - start);
            errors++;
        end
    endtask

    task automatic check_reg(input [7:0] a, input [7:0] exp);
        if (u_bmp.regs[a] !== exp) begin
            $display("ERROR: reg 0x%02h = 0x%02h, expected 0x%02h", a, u_bmp.regs[a], exp); errors++;
        end else $display("  reg 0x%02h = 0x%02h ok", a, exp);
    endtask

    task automatic finish_test;
        errors = errors + u_bmp.errors;
        $display("--------------------------------------------------");
        $display("samples=%0d stale=%0d fault_count=%0d last_fault=%0d timeouts=%0d overruns=%0d model_errors=%0d",
                 n_samples, n_stale, fault_count, fault_code, drdy_timeout_count, overrun_count, u_bmp.errors);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED (%0d errors)", errors);
        $finish;
    endtask

    initial begin : main
        integer c;
        repeat (10) @(posedge clk);
        rst = 1;

`ifdef BAD_CHIPID
        c = 0; while (fault_count < 2 && c < 200_000) begin @(posedge clk); c++; end
        if (fault_code !== 4'd1 || !fault || init_done) begin
            $display("ERROR: expected FLT_CHIP_ID, got code=%0d fault=%b init=%b", fault_code, fault, init_done); errors++;
        end else $display("[%0t] BAD_CHIPID: fault_code=1, retried %0d times, never init_done", $time, fault_count);
        finish_test();
`elsif BAD_OSR
        c = 0; while (!fault && c < 200_000) begin @(posedge clk); c++; end
        if (fault_code !== 4'd11 || init_done) begin
            $display("ERROR: expected FLT_OSR_ODR, got code=%0d", fault_code); errors++;
        end else $display("[%0t] BAD_OSR: fault_code=11 (odr_is_valid=0)", $time);
        finish_test();
`else
        // ---- phase 1: bring-up and steady state ----
        c = 0; while (!init_done && c < 200_000) begin @(posedge clk); c++; end
        if (!init_done) begin $display("ERROR: init_done never rose (code %0d)", fault_code); errors++; finish_test(); end
        $display("[%0t] init_done after %0d us", $time, $time / 1000);
        $display("Register state after bring-up:");
        check_reg(8'h14, 8'h3B);   // INT_CONFIG: pad drive 3 kept, en/push-pull/active-high/latched
        check_reg(8'h15, 8'hF1);   // INT_SOURCE: reserved kept, drdy_en
        check_reg(8'h30, 8'h03);   // DSP_CONFIG: comp P+T, no IIR shadow select
        check_reg(8'h31, 8'hC0);   // DSP_IIR: reserved kept, bypass
        check_reg(8'h36, 8'hD8);   // OSR_CONFIG: reserved kept, press_en, x8, x1
        check_reg(8'h37, 8'hBD);   // ODR_CONFIG: deep_dis, 0x0F, normal
        wait_samples(20, 25 * ODR_CYC);

        // ---- phase 2: sensor stops producing -> timeout -> re-init ----
        $display("[%0t] phase 2: pausing sensor output", $time);
        u_bmp.pause = 1;
        c = 0; while (drdy_timeout_count == 0 && c < 10 * ODR_CYC) begin @(posedge clk); c++; end
        u_bmp.pause = 0;
        if (drdy_timeout_count != 1 || fault_code != 4'd13) begin
            $display("ERROR: expected one DRDY timeout, got %0d (code %0d)", drdy_timeout_count, fault_code); errors++;
        end
        wait_samples(5, 20 * ODR_CYC);

        // ---- phase 3: sensor POR while idle -> INT disabled -> timeout -> re-init ----
        $display("[%0t] phase 3: POR between samples", $time);
        @(posedge clk); repeat (ODR_CYC / 2) @(posedge clk);
        u_bmp.inject_por();
        c = 0; while (drdy_timeout_count < 2 && c < 10 * ODR_CYC) begin @(posedge clk); c++; end
        if (drdy_timeout_count != 2) begin $display("ERROR: POR not caught by timeout"); errors++; end
        wait_samples(5, 20 * ODR_CYC);

        // ---- phase 4: sensor resets between DRDY and the read -> caught by INT_STATUS.por ----
        $display("[%0t] phase 4: reset right after DRDY", $time);
        @(posedge bmp_int);
        repeat (5) @(posedge clk);     // edge has been captured, burst read not yet started
        if (dut.tstate != 0) begin $display("ERROR: phase 4 timing assumption broken"); errors++; end
        u_bmp.load_defaults();         // device already back up, in I2C mode, config lost
        c = 0; while (fault_code != 4'd12 && c < 2 * ODR_CYC) begin @(posedge clk); c++; end
        if (fault_code != 4'd12) begin $display("ERROR: expected FLT_SENSOR_RST, got %0d", fault_code); errors++; end
        else $display("[%0t] reset detected via INT_STATUS.por, sample dropped", $time);
        wait_samples(5, 20 * ODR_CYC);

        // ---- phase 5: bus starvation longer than one ODR period -> stale flag ----
        $display("[%0t] phase 5: starving the bus for 1.5 ODR periods after a DRDY", $time);
        @(posedge bmp_int);
        starve = 1;
        repeat (ODR_CYC * 3 / 2) @(posedge clk);
        starve = 0;
        wait_samples(5, 20 * ODR_CYC);
        if (n_stale != 1) begin $display("ERROR: expected exactly 1 stale sample, saw %0d", n_stale); errors++; end

        if (overrun_count != 0) begin $display("ERROR: unexpected overruns"); errors++; end
        finish_test();
`endif
    end

    initial begin #200ms; $display("ERROR: global timeout"); errors++; finish_test(); end

endmodule

`default_nettype wire
