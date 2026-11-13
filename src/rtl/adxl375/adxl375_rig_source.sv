// ============================================================================
// adxl375_rig_source: turns adxl375_ctrl's outputs into v2 link frames
//
//   [adxl375_ctrl] --samples/status--> [adxl375_rig_source] --frames--> [rig_link_arb]
//
// The v2 counterpart of the packet builder in adxl375_rig_top (which keeps
// sending v1 packets on its own link). One source = one sensor slot (SRC).
//
// Frames (docs/rig_link_protocol.md, kind 1):
//   type 0x01 SAMPLE: X Y Z (int16) sample_count (uint32), len 10
//   type 0x00 STATUS: common 6 bytes + fault_rdata, len 7. Sent when the
//                     status changes and at least every HEARTBEAT_CYCLES.
// The latest sample waits in a 1-deep holding register; if a new sample
// arrives before it was sent, the old one is lost and link_drops counts it
// (the Pico also sees the gap in sample_count). Status and samples take
// turns, so neither can starve the other.
// ============================================================================

module adxl375_rig_source #(
    parameter logic [3:0] SRC              = 4'd0,
    parameter int         HEARTBEAT_CYCLES = 10_000_000    // 100 ms at 100 MHz
) (
    input  logic               clk,
    input  logic               rst,

    // from adxl375_ctrl
    input  logic               sample_valid,
    input  logic signed [15:0] accel_x, accel_y, accel_z,
    input  logic               init_done,
    input  logic               fault,
    input  logic               fault_cause,     // 0 init step, 1 INT1 watchdog
    input  logic [3:0]         fault_pc,
    input  logic [7:0]         fault_rdata,
    input  logic [7:0]         reinit_count,
    input  logic               int1_level,      // synchronized pin level, for reporting

    // to rig_link_tx / rig_link_arb
    output logic               frame_start,     // 1-cycle pulse
    output logic [3:0]         frame_src,
    output logic [7:0]         frame_type,
    output logic [5:0]         frame_len,
    input  logic [5:0]         frame_byte_idx,
    output logic [7:0]         frame_byte,
    input  logic               frame_done,

    output logic [7:0]         link_drops       // saturating
);

    localparam logic [7:0] KIND_ADXL375 = 8'd1;
    localparam logic [7:0] T_STATUS     = 8'h00;
    localparam logic [7:0] T_SAMPLE     = 8'h01;
    localparam int         SAMPLE_LEN   = 10;
    localparam int         STATUS_LEN   = 7;
    localparam int         SNAP_LEN     = SAMPLE_LEN;     // the longer of the two

    // int1_level is reported, not part of the key (it toggles at the sample rate)
    wire [7:0] flags = {5'b0, int1_level, fault, init_done};
    wire [1+1+1+4+8+8+8-1:0] status_key = {
        init_done, fault, fault_cause, fault_pc, fault_rdata, reinit_count, link_drops};

    logic [8*SAMPLE_LEN-1:0]      hold;
    logic                         pend;
    logic [31:0]                  sample_count;
    logic [$bits(status_key)-1:0] status_sent;
    logic [8*SNAP_LEN-1:0]        snap;
    logic                         busy, last_was_status;
    int unsigned                  hb_cnt;

    wire status_due = (status_key != status_sent) || (hb_cnt >= HEARTBEAT_CYCLES);
    // take turns when both are waiting
    wire send_status = status_due && (!pend || !last_was_status);

    assign frame_src  = SRC;
    assign frame_byte = snap[8*frame_byte_idx +: 8];

    always_ff @(posedge clk) begin
        frame_start <= 1'b0;

        if (rst) begin
            hold            <= '0;
            pend            <= 1'b0;
            sample_count    <= '0;
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
                    snap            <= (8*SNAP_LEN)'({fault_rdata, link_drops, reinit_count,
                                                      {4'h0, fault_pc}, {7'h0, fault_cause},
                                                      flags, KIND_ADXL375});
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
                sample_count <= sample_count + 1;
                hold         <= {sample_count + 32'd1, accel_z, accel_y, accel_x};
                if (pend && !(!busy && !send_status) && link_drops != 8'hFF) link_drops <= link_drops + 1;
                pend         <= 1'b1;
            end
        end
    end

endmodule
