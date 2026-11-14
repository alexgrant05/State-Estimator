`timescale 1ns/1ps
// ============================================================================
// Behavioral BMP581 SPI model for simulation only (not synthesizable).
//
// Models what the controller has to get right:
//   - powers up in I2C mode; switches to SPI after one CSB-low phase with >= 16
//     SCK edges (that first transaction returns nothing: SDO is not driven)
//   - mode 3 check: SCK must be high when CSB falls
//   - addr bit 7 = 1 read, 0 write; auto-increment on bursts
//   - soft reset (0xB6 -> 0x7E): defaults restored, back to I2C mode, busy
//   - INT_STATUS clear-on-read; latched, active-high, push-pull INT
//   - data-register shadowing: a new sample arriving mid-burst is held until CSB rises
//   - read-only registers; reserved bits set to 1 so lost RMW bits are detectable
// ============================================================================
module bmp581_model #(
    parameter logic [7:0] CHIP_ID     = 8'h50,
    parameter time        PWRUP_T     = 20us,
    parameter time        SOFTRST_T   = 20us,
    parameter time        ODR_PERIOD  = 200us,
    parameter time        MIN_CS_GAP  = 80ns,     // T_wr_idle / T_rd_idle
    parameter time        INTCFG_GAP  = 1us,      // after a write to INT_CONFIG
    parameter bit         FAST_FIRST  = 0
)(
    input  wire csb,
    input  wire sck,
    input  wire sdi,
    output wire sdo,
    output wire int_pin
);
    reg [7:0] regs [0:127];
    reg       spi_mode;
    reg       busy;
    integer   edge_cnt;
    integer   errors;

    reg [7:0] in_sh, out_sh;
    integer   bitc, bytec;
    reg       rw;
    reg [6:0] addr;
    reg       sdo_r, sdo_en;

    reg       int_latch;
    reg       pending_softrst;
    reg       pause;               // test hook: stop producing samples
    reg       shadow_pending;
    reg [23:0] sh_temp, sh_press;
    integer   k;                   // sample index currently in the data registers
    time      last_csb_rise;
    reg       last_was_intcfg_write;

    // expected-sample log for the testbench
    reg [23:0] log_press [0:4095];
    reg [23:0] log_temp  [0:4095];

    task automatic load_defaults;
        integer i;
        begin
            for (i = 0; i < 128; i = i + 1) regs[i] = 8'h00;
            regs[8'h01] = CHIP_ID;
            regs[8'h02] = 8'h32;
            regs[8'h13] = 8'h30;
            regs[8'h14] = 8'h35;   // INT_CONFIG: pad_int_drv=3, open-drain, active low, latched, disabled
            regs[8'h15] = 8'hF0;   // INT_SOURCE: reserved [7:4] set to 1 (model only)
            regs[8'h28] = 8'h03;   // STATUS: core_rdy, nvm_rdy
            regs[8'h27] = 8'h10;   // INT_STATUS: por
            regs[8'h30] = 8'h00;   // DSP_CONFIG: compensation off (controller must turn it on)
            regs[8'h31] = 8'hC0;   // DSP_IIR: reserved [7:6] set to 1 (model only)
            regs[8'h36] = 8'h80;   // OSR_CONFIG: reserved [7] set to 1 (model only)
            regs[8'h37] = 8'h70;   // ODR_CONFIG: odr=0x1C (1 Hz), standby, deep_dis=0
            int_latch   = 1'b0;
            spi_mode    = 1'b0;
            shadow_pending = 1'b0;
        end
    endtask

    function automatic [7:0] rd(input [6:0] a);
        reg [2:0] op;
        begin
            if (a == 7'h38) begin
                op = regs[8'h36][5:3];
                rd = {(op <= 3'b100), 1'b0, regs[8'h36][5:0]};   // odr_is_valid if osr_p <= x16
            end else if (a == 7'h27) begin
                rd = regs[8'h27];
                regs[8'h27] = 8'h00;                            // clear on read
                int_latch   = 1'b0;
            end else rd = regs[a];
        end
    endfunction

    task automatic wr(input [6:0] a, input [7:0] d);
        begin
            case (a)
                7'h7E: if (d == 8'hB6) pending_softrst = 1'b1;
                7'h01, 7'h02, 7'h11, 7'h27, 7'h28, 7'h38,
                7'h1D, 7'h1E, 7'h1F, 7'h20, 7'h21, 7'h22: begin
                    $display("[%0t] MODEL ERROR: write to read-only reg 0x%02h", $time, a);
                    errors = errors + 1;
                end
                7'h30, 7'h31: begin
                    if (regs[8'h37][1:0] != 2'b00) begin
                        $display("[%0t] MODEL ERROR: DSP/IIR written outside standby (lost)", $time);
                        errors = errors + 1;
                    end else regs[a] = d;
                end
                default: regs[a] = d;
            endcase
            last_was_intcfg_write = (a == 7'h14);
        end
    endtask

    task automatic apply_sample;
        begin
            regs[8'h1D] = sh_temp[7:0];   regs[8'h1E] = sh_temp[15:8];   regs[8'h1F] = sh_temp[23:16];
            regs[8'h20] = sh_press[7:0];  regs[8'h21] = sh_press[15:8];  regs[8'h22] = sh_press[23:16];
            if (regs[8'h15][0]) regs[8'h27] = regs[8'h27] | 8'h01;
            if (regs[8'h15][0]) int_latch = 1'b1;
            shadow_pending = 1'b0;
        end
    endtask

    // test hook: brown-out style reset
    task automatic inject_por;
        begin
            $display("[%0t] MODEL: injected POR", $time);
            busy = 1'b1;
            load_defaults();
            #(PWRUP_T) busy = 1'b0;
        end
    endtask

    initial begin
        errors = 0; k = 0; pause = 0; pending_softrst = 0; busy = 1;
        last_csb_rise = 0; last_was_intcfg_write = 0;
        sdo_en = 0; sdo_r = 0; edge_cnt = 0;
        load_defaults();
        #(PWRUP_T) busy = 1'b0;
    end

    // ---------------- SPI ----------------
    always @(negedge csb) begin
        edge_cnt = 0; bitc = 0; bytec = 0; sdo_en = 0;
        if (busy) begin
            $display("[%0t] MODEL ERROR: CSB activity while busy (power-up/reset)", $time);
            errors = errors + 1;
        end
        if (sck !== 1'b1) begin
            $display("[%0t] MODEL ERROR: SCK not high at CSB fall (not mode 3)", $time);
            errors = errors + 1;
        end
        if ($time - last_csb_rise < (last_was_intcfg_write ? INTCFG_GAP : MIN_CS_GAP)) begin
            $display("[%0t] MODEL ERROR: CSB idle gap too short (%0t)", $time, $time - last_csb_rise);
            errors = errors + 1;
        end
    end

    always @(sck) if (!csb) edge_cnt = edge_cnt + 1;

    always @(posedge csb) begin
        last_csb_rise = $time;
        sdo_en = 0;
        if (!spi_mode && !busy && edge_cnt >= 16) spi_mode = 1'b1;
        if (pending_softrst) begin
            pending_softrst = 1'b0;
            busy = 1'b1;
            load_defaults();
            #(SOFTRST_T) busy = 1'b0;
        end
        if (shadow_pending) apply_sample();
    end

    always @(posedge sck) if (!csb && spi_mode && !busy) begin
        in_sh = {in_sh[6:0], sdi};
        bitc  = bitc + 1;
        if (bitc == 8) begin
            bitc = 0;
            if (bytec == 0) begin
                rw   = in_sh[7];
                addr = in_sh[6:0];
                if (rw) begin out_sh = rd(addr); addr = addr + 1; end
            end else if (!rw) begin
                wr(addr, in_sh); addr = addr + 1;
            end else begin
                out_sh = rd(addr); addr = addr + 1;   // prefetch next byte of burst
            end
            bytec = bytec + 1;
        end
    end

    always @(negedge sck) if (!csb && spi_mode && !busy && rw && bytec >= 1) begin
        sdo_r  = out_sh[7];
        out_sh = {out_sh[6:0], 1'b0};
        sdo_en = 1'b1;
    end

    assign sdo = sdo_en ? sdo_r : 1'bz;

    // ---------------- INT pin ----------------
    // INT_CONFIG: [3] en, [2] od, [1] pol, [0] mode
    assign int_pin = !regs[8'h14][3] ? 1'bz :
                      regs[8'h14][2] ? (int_latch ^ regs[8'h14][1] ? 1'bz : 1'b0) :
                                       (regs[8'h14][1] ? int_latch : ~int_latch);

    // ---------------- measurement engine ----------------
    // Timer restarts when NORMAL mode is entered, like the real part.
    // FAST_FIRST: first sample 1 us after entering NORMAL (arming race test).
    task automatic produce_sample;
        begin
            k = k + 1;
            sh_temp  = 24'h190000 + k;          // ~25 degC, unique per sample
            sh_press = 24'h600000 + (k * 3);    // unique per sample
            log_temp[k % 4096]  = sh_temp;
            log_press[k % 4096] = sh_press;
            if (!regs[8'h14][0] && regs[8'h14][3]) begin
                $display("[%0t] MODEL ERROR: INT configured pulsed, controller expects latched", $time);
                errors = errors + 1;
            end
            if (csb === 1'b0) shadow_pending = 1'b1;   // shadowing: hold until burst ends
            else apply_sample();
        end
    endtask

    always begin
        wait (!busy && regs[8'h37][1:0] == 2'b01);
        if (FAST_FIRST) #1us; else #(ODR_PERIOD);
        while (!busy && regs[8'h37][1:0] == 2'b01) begin
            if (!pause) produce_sample();
            #(ODR_PERIOD);
        end
    end
endmodule
