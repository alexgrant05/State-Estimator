// ============================================================================
// bno085_rig_source: turns bno085_ctrl's outputs into v2 link frames
//
//   [bno085_ctrl] --reports/status--> [bno085_rig_source] --frames--> [rig_link_tx]
//
// One source = one sensor slot on the link (SRC). In a multi-sensor rig each
// driver gets its own source, and an arbiter shares one rig_link_tx.
//
// Frames (docs/rig_link_protocol.md, kind 2):
//   type 0x14 / 0x15 / 0x16: one raw accel / gyro / mag report, 14 bytes
//   type 0x00 STATUS: 28 bytes, sent when the status changes and at least
//                     every HEARTBEAT_CYCLES
// Each data type has a 1-deep holding register (latest report). If a new
// report arrives before the previous one was sent, the old one is lost and
// link_drops counts it. The four frame classes are served round robin, so
// a busy status can't starve data or the other way round.
// The payload is copied into a snapshot when a frame starts, so it stays
// stable while rig_link_tx reads it out.
// ============================================================================

module bno085_rig_source #(
    parameter logic [3:0] SRC              = 4'd0,
    parameter int         HEARTBEAT_CYCLES = 10_000_000    // 100 ms at 100 MHz
) (
    input  logic               clk,
    input  logic               rst,

    // from bno085_ctrl
    input  logic               accel_valid,
    input  logic signed [15:0] accel_x, accel_y, accel_z,
    input  logic [31:0]        accel_ts,
    input  logic [7:0]         accel_status, accel_seq,
    input  logic               gyro_valid,
    input  logic signed [15:0] gyro_x, gyro_y, gyro_z, gyro_temp,
    input  logic [31:0]        gyro_ts,
    input  logic [7:0]         gyro_status, gyro_seq,
    input  logic               mag_valid,
    input  logic signed [15:0] mag_x, mag_y, mag_z,
    input  logic [31:0]        mag_ts,
    input  logic [7:0]         mag_status, mag_seq,
    input  logic               init_done,
    input  logic               fault,
    input  logic [3:0]         fault_cause,
    input  logic [3:0]         fault_pc,
    input  logic [7:0]         reinit_count,
    input  logic [7:0]         reset_complete_count,
    input  logic [15:0]        unknown_id_count,
    input  logic [15:0]        cont_drop_count,
    input  logic [15:0]        trunc_count,
    input  logic [7:0]         last_chan,
    input  logic [14:0]        last_len,
    input  logic [31:0]        granted_accel_us,
    input  logic [31:0]        granted_gyro_us,
    input  logic [31:0]        granted_mag_us,
    input  logic               intn_level,      // synchronized pin level, for reporting
    input  logic               in_reset,        // sensor RSTN is low

    // to rig_link_tx
    output logic               frame_start,     // 1-cycle pulse
    output logic [3:0]         frame_src,
    output logic [7:0]         frame_type,
    output logic [5:0]         frame_len,
    input  logic [5:0]         frame_byte_idx,
    output logic [7:0]         frame_byte,
    input  logic               frame_done,

    output logic [7:0]         link_drops       // saturating
);

    localparam logic [7:0] KIND_BNO085 = 8'd2;
    localparam logic [7:0] T_STATUS    = 8'h00;
    localparam logic [7:0] T_ACCEL     = 8'h14;
    localparam logic [7:0] T_GYRO      = 8'h15;
    localparam logic [7:0] T_MAG       = 8'h16;
    localparam int         DATA_LEN    = 14;
    localparam int         STATUS_LEN  = 28;

    // data payload: x y z temp ts status seq (little endian), byte i at [8*i +: 8]
    function automatic logic [8*DATA_LEN-1:0] pack_data(
        input logic [15:0] x, y, z, t, input logic [31:0] ts, input logic [7:0] st, sq);
        return {sq, st, ts, t, z, y, x};
    endfunction

    // status that triggers an immediate STATUS frame when it changes
    // (not last_chan / last_len, which change on every SHTP transfer)
    // (intn_level is reported but not part of the key: it toggles on every
    //  SHTP transfer and would turn the heartbeat into a flood)
    wire [7:0] flags = {4'b0, in_reset, intn_level, fault, init_done};
    wire [3+8+4+4+8+8+16+16+16+96-1:0] status_key = {
        in_reset, fault, init_done, reinit_count, fault_cause, fault_pc, link_drops, reset_complete_count,
        unknown_id_count, cont_drop_count, trunc_count,
        granted_accel_us, granted_gyro_us, granted_mag_us};

    logic [8*DATA_LEN-1:0]   hold_a, hold_g, hold_m;
    logic                    pend_a, pend_g, pend_m;
    logic [$bits(status_key)-1:0] status_sent;
    logic [8*STATUS_LEN-1:0] snap;
    logic [1:0]              rr;          // next class to look at: 0 status, 1 accel, 2 gyro, 3 mag
    logic                    busy;
    int unsigned             hb_cnt;

    wire status_due = (status_key != status_sent) || (hb_cnt >= HEARTBEAT_CYCLES);
    wire [3:0] want = {pend_m, pend_g, pend_a, status_due};

    // round robin: first wanted class at or after rr
    logic [1:0] pick;
    logic       any;
    always_comb begin
        pick = rr;
        any  = 1'b0;
        for (int i = 3; i >= 0; i--) begin
            if (want[2'(rr + 2'(i))]) begin
                pick = 2'(rr + 2'(i));
                any  = 1'b1;
            end
        end
    end

    assign frame_src  = SRC;
    assign frame_byte = snap[8*frame_byte_idx +: 8];

    always_ff @(posedge clk) begin
        frame_start <= 1'b0;

        if (rst) begin
            hold_a      <= '0;
            hold_g      <= '0;
            hold_m      <= '0;
            pend_a      <= 1'b0;
            pend_g      <= 1'b0;
            pend_m      <= 1'b0;
            status_sent <= '1;                 // differs from any real status: report at boot
            snap        <= '0;
            rr          <= '0;
            busy        <= 1'b0;
            hb_cnt      <= 0;
            frame_type  <= '0;
            frame_len   <= '0;
            link_drops  <= '0;
        end else begin
            if (hb_cnt < HEARTBEAT_CYCLES) hb_cnt <= hb_cnt + 1;

            // ---- start the next frame (round robin) ----
            if (!busy && any) begin
                busy        <= 1'b1;
                frame_start <= 1'b1;
                rr          <= pick + 2'd1;
                case (pick)
                    2'd0: begin
                        frame_type  <= T_STATUS;
                        frame_len   <= 6'(STATUS_LEN);
                        snap        <= {1'b0, last_len[14:8], last_len[7:0], last_chan,
                                        granted_mag_us, granted_gyro_us, granted_accel_us,
                                        trunc_count, cont_drop_count, unknown_id_count,
                                        reset_complete_count,
                                        link_drops, reinit_count, {4'h0, fault_pc}, {4'h0, fault_cause},
                                        flags, KIND_BNO085};
                        status_sent <= status_key;
                        hb_cnt      <= 0;
                    end
                    2'd1: begin
                        frame_type <= T_ACCEL;
                        frame_len  <= 6'(DATA_LEN);
                        snap       <= (8*STATUS_LEN)'(hold_a);
                        pend_a     <= 1'b0;
                    end
                    2'd2: begin
                        frame_type <= T_GYRO;
                        frame_len  <= 6'(DATA_LEN);
                        snap       <= (8*STATUS_LEN)'(hold_g);
                        pend_g     <= 1'b0;
                    end
                    default: begin
                        frame_type <= T_MAG;
                        frame_len  <= 6'(DATA_LEN);
                        snap       <= (8*STATUS_LEN)'(hold_m);
                        pend_m     <= 1'b0;
                    end
                endcase
            end
            if (frame_done) busy <= 1'b0;

            // ---- capture reports (after the clears above, so a report that
            //      lands on the same cycle stays pending) ----
            if (accel_valid) begin
                hold_a <= pack_data(accel_x, accel_y, accel_z, 16'h0000, accel_ts, accel_status, accel_seq);
                if (pend_a && !(!busy && any && pick == 2'd1) && link_drops != 8'hFF) link_drops <= link_drops + 1;
                pend_a <= 1'b1;
            end
            if (gyro_valid) begin
                hold_g <= pack_data(gyro_x, gyro_y, gyro_z, gyro_temp, gyro_ts, gyro_status, gyro_seq);
                if (pend_g && !(!busy && any && pick == 2'd2) && link_drops != 8'hFF) link_drops <= link_drops + 1;
                pend_g <= 1'b1;
            end
            if (mag_valid) begin
                hold_m <= pack_data(mag_x, mag_y, mag_z, 16'h0000, mag_ts, mag_status, mag_seq);
                if (pend_m && !(!busy && any && pick == 2'd3) && link_drops != 8'hFF) link_drops <= link_drops + 1;
                pend_m <= 1'b1;
            end
        end
    end

endmodule
