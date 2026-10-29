`timescale 1ns/1ps
`default_nettype none

module tb_gnss_msg_router;
    /* verilator lint_off PROCASSINIT */
    logic clk = 1'b0; // standard free-running clock-gen seed, not a reset-covered value
    /* verilator lint_on PROCASSINIT */
    logic rst_n = 1'b0;

    logic [7:0]  payload_byte = 8'h0;
    logic        payload_byte_valid = 1'b0;
    logic        payload_byte_last = 1'b0;
    logic [7:0]  msg_class = 8'h0;
    logic [7:0]  msg_id = 8'h0;
    logic [15:0] msg_length = 16'h0;
    logic        frame_sof = 1'b0;
    logic        frame_done = 1'b0;
    logic        checksum_fail = 1'b0;

    logic [7:0] nav_byte, bulk_byte;
    logic       nav_byte_valid, nav_byte_last, bulk_byte_valid, bulk_byte_last;
    logic [7:0] nav_msg_id, bulk_msg_id;
    logic       nav_frame_done, nav_checksum_fail, bulk_frame_done, bulk_checksum_fail;

    always #5 clk <= ~clk;

    gnss_msg_router dut (
        .clk, .rst_n,
        .payload_byte, .payload_byte_valid, .payload_byte_last,
        .msg_class, .msg_id, .msg_length,
        .frame_sof, .frame_done, .checksum_fail,
        .nav_byte, .nav_byte_valid, .nav_byte_last, .nav_msg_id, .nav_frame_done, .nav_checksum_fail,
        .bulk_byte, .bulk_byte_valid, .bulk_byte_last, .bulk_msg_id, .bulk_frame_done, .bulk_checksum_fail
    );

    // what the DUT actually produced for the frame currently in flight
    byte captured_nav[$];
    byte captured_bulk[$];
    bit  captured_nav_last[$];  // nav_byte_last sampled alongside each captured_nav entry
    bit  captured_bulk_last[$]; // bulk_byte_last sampled alongside each captured_bulk entry
    bit  nav_done_seen, bulk_done_seen;
    bit  nav_fail_seen, bulk_fail_seen;

    task automatic start_frame(byte cls, byte idv, logic [15:0] len);
        @(negedge clk);
        msg_class  = cls;
        msg_id     = idv;
        msg_length = len;
        frame_sof  = 1'b1;
        @(posedge clk); #1;
        @(negedge clk);
        frame_sof = 1'b0;
    endtask

    task automatic send_payload_byte(byte b, bit last);
        @(negedge clk);
        payload_byte       = b;
        payload_byte_valid = 1'b1;
        payload_byte_last  = last;
        @(posedge clk); #1;
        if (nav_byte_valid) begin
            captured_nav.push_back(nav_byte);
            captured_nav_last.push_back(nav_byte_last);
        end
        if (bulk_byte_valid) begin
            captured_bulk.push_back(bulk_byte);
            captured_bulk_last.push_back(bulk_byte_last);
        end
        @(negedge clk);
        payload_byte_valid = 1'b0;
        payload_byte_last  = 1'b0;
    endtask

    task automatic end_frame(bit fail);
        @(negedge clk);
        frame_done    = 1'b1;
        checksum_fail = fail;
        @(posedge clk); #1;
        if (nav_frame_done)     nav_done_seen  = 1'b1;
        if (bulk_frame_done)    bulk_done_seen = 1'b1;
        if (nav_checksum_fail)  nav_fail_seen  = 1'b1;
        if (bulk_checksum_fail) bulk_fail_seen = 1'b1;
        @(negedge clk);
        frame_done    = 1'b0;
        checksum_fail = 1'b0;
    endtask

    task automatic send_frame(byte cls, byte idv, byte payload[], bit fail);
        int i;
        captured_nav.delete();
        captured_bulk.delete();
        captured_nav_last.delete();
        captured_bulk_last.delete();
        nav_done_seen = 1'b0; bulk_done_seen = 1'b0;
        nav_fail_seen = 1'b0; bulk_fail_seen = 1'b0;

        start_frame(cls, idv, 16'(payload.size()));
        for (i = 0; i < payload.size(); i++)
            send_payload_byte(payload[i], i == payload.size() - 1);
        end_frame(fail);
    endtask

    // checks that a _last flag queue is 0 for every byte except a 1 on the final byte
    task automatic assert_last_marks_only_final(bit last_flags[$], string label);
        int i;
        for (i = 0; i < last_flags.size(); i++) begin
            if (i == last_flags.size() - 1)
                assert (last_flags[i]) else $fatal(1, {label, ": final byte did not assert _last"});
            else
                assert (!last_flags[i]) else $fatal(1, {label, ": _last asserted on a non-final byte"});
        end
    endtask

    initial begin
        static byte data[] = '{8'hAA, 8'hBB, 8'hCC};

        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // NAV-PVT (0x01/0x07) must route to the nav path only
        send_frame(8'h01, 8'h07, data, 1'b0);
        assert (captured_nav.size() == 3 && captured_bulk.size() == 0)
            else $fatal(1, "NAV-PVT did not route exclusively to the nav path");
        assert (captured_nav[0] == 8'hAA && captured_nav[1] == 8'hBB && captured_nav[2] == 8'hCC)
            else $fatal(1, "nav path payload content mismatch");
        assert (nav_done_seen && !bulk_done_seen)
            else $fatal(1, "nav_frame_done did not fire alone for NAV-PVT");
        assert (nav_msg_id == 8'h07) else $fatal(1, "nav_msg_id mismatch");
        assert_last_marks_only_final(captured_nav_last, "NAV-PVT nav path");

        // NAV-COV (0x01/0x36) must also route to the nav path
        send_frame(8'h01, 8'h36, data, 1'b0);
        assert (captured_nav.size() == 3 && captured_bulk.size() == 0)
            else $fatal(1, "NAV-COV did not route to the nav path");

        // RXM-RAWX (0x02/0x15) must route to the bulk path only
        send_frame(8'h02, 8'h15, data, 1'b0);
        assert (captured_bulk.size() == 3 && captured_nav.size() == 0)
            else $fatal(1, "RXM-RAWX did not route exclusively to the bulk path");
        assert (bulk_done_seen && !nav_done_seen)
            else $fatal(1, "bulk_frame_done did not fire alone for RXM-RAWX");
        assert (bulk_msg_id == 8'h15) else $fatal(1, "bulk_msg_id mismatch");
        assert_last_marks_only_final(captured_bulk_last, "RXM-RAWX bulk path");

        // RXM-SFRBX (0x02/0x13) must also route to the bulk path
        send_frame(8'h02, 8'h13, data, 1'b0);
        assert (captured_bulk.size() == 3 && captured_nav.size() == 0)
            else $fatal(1, "RXM-SFRBX did not route to the bulk path");

        // An unrecognized class/id (e.g. TIM-TP, 0x0D/0x01) must route nowhere
        send_frame(8'h0D, 8'h01, data, 1'b0);
        assert (captured_nav.size() == 0 && captured_bulk.size() == 0)
            else $fatal(1, "unrecognized message was routed somewhere");
        assert (!nav_done_seen && !bulk_done_seen)
            else $fatal(1, "frame_done fired on an unrecognized message");

        // checksum_fail must propagate only to whichever path was actually routed
        send_frame(8'h01, 8'h07, data, 1'b1);
        assert (nav_fail_seen && !bulk_fail_seen)
            else $fatal(1, "checksum_fail did not propagate correctly to the nav path");

        $display("PASS tb_gnss_msg_router");
        $finish;
    end
endmodule

`default_nettype wire
