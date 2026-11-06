`timescale 1ns/1ps
`default_nettype none

// tangnano20k_top
//
// Bring-up step 1 top level for the Tang Nano 20K: proves the open-source
// toolchain, the pin constraints, and the FPGA -> host UART path before
// any GNSS logic is attached. Once per second it blinks LED0 and sends
// "tang nano hello XX\r\n" over the on-board USB-serial bridge, where XX
// is a hex message counter (so a frozen design is obvious from the host).
// LED1 toggles once per message.
//
// The board has no reset button wired into this design, so a small
// power-on reset counter generates rst_n for everything else -- see the
// por_count comment below.
//
// Inputs:  clk_27m (on-board 27 MHz oscillator)
// Outputs: uart_tx (to the USB-serial bridge), led_n[5:0] (active-low LEDs)
module tangnano20k_top #(
    parameter int CLK_HZ     = 27_000_000,
    parameter int BAUD       = 115_200,
    parameter int POR_WIDTH  = 10            // 2^10 clocks = ~38 us of reset at 27 MHz
) (
    input  logic       clk_27m,
    output logic       uart_tx,
    output logic [5:0] led_n
);

    // ---------------------------------------------------------------
    // Power-on reset
    //
    // por_count is the one register in the design that cannot use the
    // repo's synchronous-reset convention, because it *is* the reset
    // generator -- there is no earlier reset to clear it. It relies on the
    // FPGA loading every flip-flop with its declared initial value (0)
    // when the bitstream is configured, then counts up to all-ones and
    // holds there. That makes this declaration initializer intentional, so
    // the PROCASSINIT warning is a genuine false positive here and is
    // waived rather than "fixed".
    // ---------------------------------------------------------------
    /* verilator lint_off PROCASSINIT */
    logic [POR_WIDTH-1:0] por_count = '0;
    /* verilator lint_on PROCASSINIT */
    logic                 rst_n;

    always_ff @(posedge clk_27m) begin
        if (!(&por_count)) por_count <= por_count + 1'b1;
    end

    // Combinational is fine here: every consumer samples rst_n synchronously,
    // so any glitch while por_count's bits change settles before the next edge.
    always_comb rst_n = &por_count;

    // ---------------------------------------------------------------
    // 1 Hz tick + heartbeat
    // ---------------------------------------------------------------
    localparam int TICK_WIDTH = $clog2(CLK_HZ);
    localparam logic [TICK_WIDTH-1:0] LAST_TICK = TICK_WIDTH'(CLK_HZ - 1);

    logic [TICK_WIDTH-1:0] tick_count;
    logic                  second_tick;
    logic                  heartbeat;

    always_ff @(posedge clk_27m) begin
        if (!rst_n) begin
            tick_count  <= '0;
            second_tick <= 1'b0;
            heartbeat   <= 1'b0;
        end else begin
            second_tick <= 1'b0;
            if (tick_count == LAST_TICK) begin
                tick_count  <= '0;
                second_tick <= 1'b1;
                heartbeat   <= ~heartbeat;
            end else begin
                tick_count <= tick_count + 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------
    // Hello message: "tang nano hello " + 2 hex digits + "\r\n"
    // ---------------------------------------------------------------
    localparam int GREETING_LEN = 16;
    localparam logic [8*GREETING_LEN-1:0] GREETING = "tang nano hello ";
    localparam logic [4:0] LAST_INDEX = 5'(GREETING_LEN + 3); // + 2 hex digits + CR + LF - 1

    function automatic logic [7:0] hex_digit(input logic [3:0] nibble);
        return (nibble < 4'd10) ? (8'h30 + {4'h0, nibble})   // '0'..'9'
                                : (8'h57 + {4'h0, nibble});  // 'a'..'f' ('a' - 10 = 0x57)
    endfunction

    logic       sending;
    logic [4:0] byte_index;
    logic [7:0] message_count;
    logic [7:0] tx_data;
    logic       tx_ready;

    always_comb begin
        if (byte_index < 5'(GREETING_LEN))
            tx_data = GREETING[8*(GREETING_LEN - 1 - int'(byte_index)) +: 8];
        else if (byte_index == 5'(GREETING_LEN))
            tx_data = hex_digit(message_count[7:4]);
        else if (byte_index == 5'(GREETING_LEN + 1))
            tx_data = hex_digit(message_count[3:0]);
        else if (byte_index == 5'(GREETING_LEN + 2))
            tx_data = 8'h0D; // CR
        else
            tx_data = 8'h0A; // LF
    end

    always_ff @(posedge clk_27m) begin
        if (!rst_n) begin
            sending       <= 1'b0;
            byte_index    <= 5'd0;
            message_count <= 8'h0;
        end else begin
            if (sending && tx_ready) begin
                if (byte_index == LAST_INDEX) begin
                    sending       <= 1'b0;
                    byte_index    <= 5'd0;
                    message_count <= message_count + 8'h1;
                end else begin
                    byte_index <= byte_index + 5'd1;
                end
            end
            if (second_tick && !sending) sending <= 1'b1;
        end
    end

    uart_tx #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (BAUD)
    ) u_uart_tx (
        .clk      (clk_27m),
        .rst_n    (rst_n),
        .in_data  (tx_data),
        .in_valid (sending),
        .in_ready (tx_ready),
        .tx       (uart_tx)
    );

    // LEDs are active-low on this board
    always_comb begin
        led_n    = 6'b111111;
        led_n[0] = ~heartbeat;
        led_n[1] = ~message_count[0];
    end

endmodule

`default_nettype wire
