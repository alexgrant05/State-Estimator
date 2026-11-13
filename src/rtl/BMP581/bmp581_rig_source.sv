// ============================================================================
// bmp581_rig_source: turns bmp581_controller's outputs into v2 link frames
//
//   [bmp581_controller] --samples/status--> [bmp581_rig_source] --frames--> [rig_link_arb]
//
// One source = one sensor slot on the link (SRC).
//
// Frames (docs/rig_link_protocol.md, kind 3):
//   type 0x01 SAMPLE, len 13:
//     0-2  pressure raw (uint24, Pa = raw / 64)
//     3-5  temperature raw (int24, degC = raw / 65536)
//     6    sample_flags ([0] press skipped [1] temp skipped [2] overrun
//                        [3] drdy status not set [4] stale)
//     7-8  sample_seq (uint16; a gap means the link dropped samples)
//     9-12 timestamp, low 32 bits of the FPGA tick count at the DRDY edge
//   type 0x00 STATUS, len 11: common 6 bytes, then
//     6    fault_count     7-8 drdy_timeout_count     9-10 overrun_count
//   Common part: fault_cause = fault_code, fault_pc = 0 (the code already
//   names the config step), reinit_count = fault_count (every fault re-inits).
// Status and samples take turns when both are waiting; the latest sample
// waits in a 1-deep holding register and link_drops counts overwrites.
// ============================================================================

module bmp581_rig_source #(
    parameter logic [3:0] SRC              = 4'd2,
    parameter int         HEARTBEAT_CYCLES = 10_000_000    // 100 ms at 100 MHz
) (
    input  logic        clk,
    input  logic        rst,            // active high, like the other sources

    // from bmp581_controller
    input  logic        sample_valid,
    input  logic [15:0] sample_seq,
    input  logic [31:0] sample_ts,      // low 32 bits of sample_timestamp
    input  logic [23:0] sample_press_raw,
    input  logic [23:0] sample_temp_raw,
    input  logic [7:0]  sample_flags,
    input  logic        init_done,
    input  logic        fault,
    input  logic [3:0]  fault_code,
    input  logic [7:0]  fault_count,
    input  logic [15:0] drdy_timeout_count,
    input  logic [15:0] overrun_count,
    input  logic        int_level,      // synchronized pin level, for reporting

    // to rig_link_tx / rig_link_arb
    output logic        frame_start,    // 1-cycle pulse
    output logic [3:0]  frame_src,
    output logic [7:0]  frame_type,
    output logic [5:0]  frame_len,
    input  logic [5:0]  frame_byte_idx,
    output logic [7:0]  frame_byte,
    input  logic        frame_done,

    output logic [7:0]  link_drops      // saturating
);

    localparam logic [7:0] KIND_BMP581 = 8'd3;
    localparam logic [7:0] T_STATUS    = 8'h00;
    localparam logic [7:0] T_SAMPLE    = 8'h01;
    localparam int         SAMPLE_LEN  = 13;
    localparam int         STATUS_LEN  = 11;
    localparam int         SNAP_LEN    = SAMPLE_LEN;      // the longer of the two

    // int_level is reported, not part of the key (it toggles at the sample rate)
    wire [7:0] flags = {5'b0, int_level, fault, init_done};
    wire [1+1+4+8+16+16+8-1:0] status_key = {
        init_done, fault, fault_code, fault_count, drdy_timeout_count, overrun_count, link_drops};

    logic [8*SAMPLE_LEN-1:0]      hold;
    logic                         pend;
    logic [$bits(status_key)-1:0] status_sent;
    logic [8*SNAP_LEN-1:0]        snap;
    logic                         busy, last_was_status;
    int unsigned                  hb_cnt;

    wire status_due  = (status_key != status_sent) || (hb_cnt >= HEARTBEAT_CYCLES);
    wire send_status = status_due && (!pend || !last_was_status);   // take turns

    assign frame_src  = SRC;
    assign frame_byte = snap[8*frame_byte_idx +: 8];

    always_ff @(posedge clk) begin
        frame_start <= 1'b0;

        if (rst) begin
            hold            <= '0;
            pend            <= 1'b0;
            status_sent     <= '1;          // differs from any real status: report at boot
            snap            <= '0;
            busy            <= 1'b0;
            last_was_status <= 1'b0;
            hb_cnt          <= 0;
            frame_type      <= '0;
            frame_len       <= '0;
            link_drops      <= '0;
        end else begin
            if (hb_cnt < HEARTBEAT_CYCLES) hb_cnt <= hb_cnt + 1;

            // ---- start the next frame ----
            if (!busy && (status_due || pend)) begin
                busy        <= 1'b1;
                frame_start <= 1'b1;
                if (send_status) begin
                    frame_type      <= T_STATUS;
                    frame_len       <= 6'(STATUS_LEN);
                    snap            <= (8*SNAP_LEN)'({overrun_count, drdy_timeout_count, fault_count,
                                                      link_drops, fault_count, 8'h00, {4'h0, fault_code},
                                                      flags, KIND_BMP581});
                    status_sent     <= status_key;
                    hb_cnt          <= 0;
                    last_was_status <= 1'b1;
                end else begin
                    frame_type      <= T_SAMPLE;
                    frame_len       <= 6'(SAMPLE_LEN);
                    snap            <= hold;
                    pend            <= 1'b0;
                    last_was_status <= 1'b0;
                end
            end
            if (frame_done) busy <= 1'b0;

            // ---- capture samples (after the clear above, so a sample that
            //      lands on the same cycle stays pending) ----
            if (sample_valid) begin
                hold <= {sample_ts, sample_seq, sample_flags, sample_temp_raw, sample_press_raw};
                if (pend && !(!busy && !send_status) && link_drops != 8'hFF) link_drops <= link_drops + 1;
                pend <= 1'b1;
            end
        end
    end

endmodule
