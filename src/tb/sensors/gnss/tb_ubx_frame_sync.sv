`timescale 1ns/1ps
`default_nettype none

module tb_ubx_frame_sync;
    logic clk;
    logic rst_n = 1'b0;
    logic [7:0] byte_in = 8'h00;
    logic byte_in_valid = 1'b0;

    logic [7:0]  payload_byte;
    logic        payload_byte_valid;
    logic        payload_byte_last;
    logic [7:0]  msg_class;
    logic [7:0]  msg_id;
    logic [15:0] msg_length;
    logic        frame_sof;
    logic        frame_done;
    logic        checksum_fail;

    // clk is owned entirely by this one process (no declaration initializer), which
    // keeps Verilator PROCASSINIT quiet without a waiver on any Verilator version
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    ubx_frame_sync dut (
        .clk, .rst_n, .byte_in, .byte_in_valid,
        .payload_byte, .payload_byte_valid, .payload_byte_last,
        .msg_class, .msg_id, .msg_length,
        .frame_sof, .frame_done, .checksum_fail
    );

    // what the DUT actually produced for the frame currently in flight
    byte captured_payload[$];
    bit  captured_payload_last[$]; // payload_byte_last sampled alongside each captured_payload entry
    bit  captured_frame_sof;
    bit  captured_frame_done;
    bit  captured_checksum_fail;
    logic [7:0]  captured_class;
    logic [7:0]  captured_id;
    logic [15:0] captured_length;

    // Fletcher-8 (UBX) checksum: CK_A += byte; CK_B += CK_A -- matches the DUT exactly,
    // including over the class/id/len header bytes, not just the payload.
    function automatic void compute_checksum(byte data[$], output byte ck_a, output byte ck_b);
        byte a, b;
        int  i;
        a = 8'h0;
        b = 8'h0;
        for (i = 0; i < data.size(); i++) begin
            a = a + data[i];
            b = b + a;
        end
        ck_a = a;
        ck_b = b;
    endfunction

    task automatic send_byte(byte b);
        @(negedge clk);
        byte_in       = b;
        byte_in_valid = 1'b1;
        @(posedge clk); #1;
        if (payload_byte_valid) begin
            captured_payload.push_back(payload_byte);
            captured_payload_last.push_back(payload_byte_last);
        end
        if (frame_sof) begin
            captured_frame_sof = 1'b1;
            captured_class     = msg_class;
            captured_id        = msg_id;
            captured_length    = msg_length;
        end
        if (frame_done) begin
            captured_frame_done    = 1'b1;
            captured_checksum_fail = checksum_fail;
        end
        @(negedge clk);
        byte_in_valid = 1'b0;
    endtask

    task automatic send_frame(byte cls, byte idv, byte payload[], bit bad_checksum);
        byte full[$];
        byte ck_a, ck_b;
        int  i;

        full.push_back(cls);
        full.push_back(idv);
        full.push_back(byte'(payload.size()));
        full.push_back(byte'(payload.size() >> 8));
        for (i = 0; i < payload.size(); i++) full.push_back(payload[i]);

        compute_checksum(full, ck_a, ck_b);
        if (bad_checksum) ck_b = ck_b ^ 8'hFF;

        captured_payload.delete();
        captured_payload_last.delete();
        captured_frame_sof     = 1'b0;
        captured_frame_done    = 1'b0;
        captured_checksum_fail = 1'b0;

        send_byte(8'hB5);
        send_byte(8'h62);
        send_byte(cls);
        send_byte(idv);
        send_byte(byte'(payload.size()));
        send_byte(byte'(payload.size() >> 8));
        for (i = 0; i < payload.size(); i++) send_byte(payload[i]);
        send_byte(ck_a);
        send_byte(ck_b);
    endtask

    // checks that a _last flag queue is 0 for every byte except a 1 on the final byte
    task automatic assert_last_marks_only_final(bit last_flags[$], string label);
        int i;
        for (i = 0; i < last_flags.size(); i++) begin
            if (i == last_flags.size() - 1)
                assert (last_flags[i]) else $fatal(1, {label, ": final byte did not assert payload_byte_last"});
            else
                assert (!last_flags[i]) else $fatal(1, {label, ": payload_byte_last asserted on a non-final byte"});
        end
    endtask

    initial begin
        static byte payload1[] = '{8'hDE, 8'hAD, 8'hBE, 8'hEF};
        static byte empty_payload[] = '{};

        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // Scenario 1: a valid NAV-PVT-shaped frame with a correct checksum
        send_frame(8'h01, 8'h07, payload1, 1'b0);
        assert (captured_frame_sof) else $fatal(1, "frame_sof did not pulse");
        assert (captured_class == 8'h01) else $fatal(1, "msg_class wrong");
        assert (captured_id == 8'h07) else $fatal(1, "msg_id wrong");
        assert (captured_length == 16'd4) else $fatal(1, "msg_length wrong");
        assert (captured_payload.size() == 4) else $fatal(1, "payload byte count wrong");
        assert (captured_payload[0] == 8'hDE && captured_payload[1] == 8'hAD &&
                captured_payload[2] == 8'hBE && captured_payload[3] == 8'hEF)
            else $fatal(1, "payload content mismatch");
        assert (captured_frame_done) else $fatal(1, "frame_done did not pulse");
        assert (!captured_checksum_fail) else $fatal(1, "valid frame incorrectly flagged checksum_fail");
        assert_last_marks_only_final(captured_payload_last, "scenario 1");

        // Scenario 2: the same frame, with a deliberately corrupted checksum byte
        send_frame(8'h01, 8'h07, payload1, 1'b1);
        assert (captured_frame_done) else $fatal(1, "frame_done did not pulse on a bad-checksum frame");
        assert (captured_checksum_fail) else $fatal(1, "checksum_fail was not raised for a corrupted frame");

        // Scenario 3: leading noise (idle filler + a false sync start) before a real frame --
        // the state machine must resync rather than getting stuck or false-triggering
        send_byte(8'hFF);
        send_byte(8'hFF);
        send_byte(8'hB5); // sync1 with no following sync2 -- a false start
        send_byte(8'h00); // not sync2, must fall back to hunting for sync1 again
        send_frame(8'h02, 8'h15, payload1, 1'b0);
        assert (captured_frame_sof) else $fatal(1, "failed to resync after leading noise");
        assert (captured_class == 8'h02 && captured_id == 8'h15)
            else $fatal(1, "wrong class/id after resync");
        assert (captured_frame_done && !captured_checksum_fail)
            else $fatal(1, "frame after resync did not complete cleanly");

        // Scenario 4: a zero-length payload (e.g. a poll-type message)
        send_frame(8'h0D, 8'h01, empty_payload, 1'b0);
        assert (captured_frame_sof) else $fatal(1, "frame_sof did not pulse for a zero-length frame");
        assert (captured_payload.size() == 0) else $fatal(1, "zero-length frame produced payload bytes");
        assert (captured_frame_done && !captured_checksum_fail)
            else $fatal(1, "zero-length frame checksum incorrectly failed");

        $display("PASS tb_ubx_frame_sync");
        $finish;
    end
endmodule

`default_nettype wire
