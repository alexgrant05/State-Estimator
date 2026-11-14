`timescale 1ns/1ps
`default_nettype none

// ubx_frame_sync
//
// Byte-stream UBX protocol framer: finds 0xB5 0x62 sync, captures
// class/id/length, streams payload bytes through, and checks the
// Fletcher-8 checksum (CK_A/CK_B) over class+id+length+payload.
//
// No backpressure on byte_in: the byte rate is paced by the SPI clock
// (CLOCK_DIVIDE cycles per bit, far slower than this module's own
// per-byte processing), so a simple valid-only input is sufficient.
//
// Inputs:  clk, rst_n, byte_in, byte_in_valid
// Outputs: payload_byte/payload_byte_valid/payload_byte_last,
//          msg_class, msg_id, msg_length, frame_sof, frame_done, checksum_fail
module ubx_frame_sync (
    input  logic         clk,
    input  logic         rst_n,

    input  logic [7:0]   byte_in,
    input  logic         byte_in_valid,

    output logic [7:0]   payload_byte,
    output logic         payload_byte_valid,
    output logic         payload_byte_last,

    output logic [7:0]   msg_class,
    output logic [7:0]   msg_id,
    output logic [15:0]  msg_length,

    output logic         frame_sof,   // pulses once length is known, before payload starts
    output logic         frame_done,  // pulses once CK_B has been checked
    output logic         checksum_fail
);

    typedef enum logic [3:0] {
        S_SYNC1   = 4'd0,
        S_SYNC2   = 4'd1,
        S_CLASS   = 4'd2,
        S_ID      = 4'd3,
        S_LEN_LO  = 4'd4,
        S_LEN_HI  = 4'd5,
        S_PAYLOAD = 4'd6,
        S_CK_A    = 4'd7,
        S_CK_B    = 4'd8
    } state_t;

    localparam logic [7:0] UBX_SYNC1 = 8'hB5;
    localparam logic [7:0] UBX_SYNC2 = 8'h62;

    state_t      state;
    logic [15:0] payload_count;
    logic [7:0]  ck_a, ck_b;     // running Fletcher-8 over class/id/len/payload
    logic [7:0]  rx_ck_a;        // CK_A byte as received on the wire

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state              <= S_SYNC1;
            payload_count      <= 16'h0;
            ck_a               <= 8'h0;
            ck_b               <= 8'h0;
            rx_ck_a            <= 8'h0;
            msg_class          <= 8'h0;
            msg_id             <= 8'h0;
            msg_length         <= 16'h0;
            payload_byte       <= 8'h0;
            payload_byte_valid <= 1'b0;
            payload_byte_last  <= 1'b0;
            frame_sof          <= 1'b0;
            frame_done         <= 1'b0;
            checksum_fail      <= 1'b0;
        end else begin
            payload_byte_valid <= 1'b0;
            payload_byte_last  <= 1'b0;
            frame_sof          <= 1'b0;
            frame_done         <= 1'b0;
            checksum_fail      <= 1'b0;

            if (byte_in_valid) begin
                case (state)
                    S_SYNC1: state <= (byte_in == UBX_SYNC1) ? S_SYNC2 : S_SYNC1;

                    S_SYNC2: state <= (byte_in == UBX_SYNC2) ? S_CLASS : S_SYNC1;

                    S_CLASS: begin
                        msg_class <= byte_in;
                        ck_a      <= byte_in;
                        ck_b      <= byte_in;
                        state     <= S_ID;
                    end

                    S_ID: begin
                        msg_id <= byte_in;
                        ck_a   <= ck_a + byte_in;
                        ck_b   <= ck_b + (ck_a + byte_in);
                        state  <= S_LEN_LO;
                    end

                    S_LEN_LO: begin
                        msg_length[7:0] <= byte_in;
                        ck_a            <= ck_a + byte_in;
                        ck_b            <= ck_b + (ck_a + byte_in);
                        state           <= S_LEN_HI;
                    end

                    S_LEN_HI: begin
                        msg_length[15:8] <= byte_in;
                        ck_a             <= ck_a + byte_in;
                        ck_b             <= ck_b + (ck_a + byte_in);
                        payload_count    <= 16'h0;
                        frame_sof        <= 1'b1;
                        // zero-length payload (e.g. poll requests) skips straight to checksum
                        state <= ({byte_in, msg_length[7:0]} == 16'h0) ? S_CK_A : S_PAYLOAD;
                    end

                    S_PAYLOAD: begin
                        payload_byte       <= byte_in;
                        payload_byte_valid <= 1'b1;
                        ck_a               <= ck_a + byte_in;
                        ck_b               <= ck_b + (ck_a + byte_in);
                        payload_count      <= payload_count + 16'h1;
                        if (payload_count + 16'h1 == msg_length) begin
                            payload_byte_last <= 1'b1;
                            state             <= S_CK_A;
                        end
                    end

                    S_CK_A: begin
                        rx_ck_a <= byte_in;
                        state   <= S_CK_B;
                    end

                    S_CK_B: begin
                        frame_done    <= 1'b1;
                        checksum_fail <= (rx_ck_a != ck_a) || (byte_in != ck_b);
                        state         <= S_SYNC1;
                    end

                    default: state <= S_SYNC1;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
