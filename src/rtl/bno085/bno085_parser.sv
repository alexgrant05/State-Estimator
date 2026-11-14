// ============================================================================
// BNO085 SH-2 report parser
//
// Consumes bno085_shtp's payload byte stream, one byte at a time, with no
// payload buffer. Channels 2 (control), 3 (input reports) and 4 (wake input
// reports) carry several reports back to back with no per-report length, so
// they are walked with the report-length table (same values as getReportLen
// in CEVA's sh2.c). Only the current report's bytes are held, and only
// until its last byte.
//
//   ID   len  meaning                       action
//   FB    5   base timestamp reference      skipped
//   FA    5   timestamp rebase              skipped
//   01-03 10  calibrated accel/gyro/mag     skipped (enabled alongside the raw ones)
//   14   16   raw accelerometer             accel_* updated, accel_valid
//   15   16   raw gyroscope                 gyro_*  updated, gyro_valid
//   16   16   raw magnetometer              mag_*   updated, mag_valid
//   FC   17   get feature response (ch2)    fc_id / fc_interval_us, fc_valid
//   F1   16   command response (ch2)        skipped (unsolicited init at boot)
//   F8   16   product ID response (ch2)     skipped (answer to the init probe)
//   else      unknown: unknown_id pulses, rest of the packet is skipped
//
// Raw report (16 bytes, little endian):
//   0 id | 1 seq | 2 status (b1:0 accuracy, b7:2 delay[13:8]) | 3 delay[7:0]
//   4-5 X | 6-7 Y | 8-9 Z | 10-11 reserved (gyro: temperature) | 12-15 timestamp us
//
// Channel 1: a 1-byte payload 0x01 is "reset complete" (reset_complete pulse).
// Continuation fragments (cont = 1) are dropped whole: their first byte is
// mid-report, so walking them would misread data as report IDs.
// A packet that ends partway through a report pulses short_report; that
// report is not output.
// ============================================================================

module bno085_parser (
    input  logic               clk,
    input  logic               rst,

    input  logic               rx_valid,
    input  logic [7:0]         rx_data,
    input  logic               rx_sop,
    input  logic               rx_eop,
    input  logic [7:0]         rx_chan,
    input  logic               rx_cont,

    output logic               reset_complete,   // 1-cycle pulses
    output logic               unknown_id,
    output logic               short_report,

    output logic               accel_valid,
    output logic signed [15:0] accel_x, accel_y, accel_z,
    output logic [31:0]        accel_ts,
    output logic [7:0]         accel_status,
    output logic [7:0]         accel_seq,

    output logic               gyro_valid,
    output logic signed [15:0] gyro_x, gyro_y, gyro_z,
    output logic signed [15:0] gyro_temp,
    output logic [31:0]        gyro_ts,
    output logic [7:0]         gyro_status,
    output logic [7:0]         gyro_seq,

    output logic               mag_valid,
    output logic signed [15:0] mag_x, mag_y, mag_z,
    output logic [31:0]        mag_ts,
    output logic [7:0]         mag_status,
    output logic [7:0]         mag_seq,

    output logic               fc_valid,
    output logic [7:0]         fc_id,
    output logic [31:0]        fc_interval_us
);

    localparam logic [7:0] ID_RAW_ACCEL = 8'h14;
    localparam logic [7:0] ID_RAW_GYRO  = 8'h15;
    localparam logic [7:0] ID_RAW_MAG   = 8'h16;
    localparam logic [7:0] ID_GET_FEAT  = 8'hFC;

    function automatic logic [4:0] report_len(input logic [7:0] id);
        case (id)
            8'hFB, 8'hFA:                          return 5'd5;
            8'h01, 8'h02, 8'h03:                   return 5'd10;  // calibrated accel / gyro / mag
            ID_RAW_ACCEL, ID_RAW_GYRO, ID_RAW_MAG: return 5'd16;
            ID_GET_FEAT:                           return 5'd17;
            8'hF1, 8'hF8:                          return 5'd16;
            default:                               return 5'd0;
        endcase
    endfunction

    logic       skip;          // ignore the rest of this packet
    logic [4:0] off;           // offset of the next byte within its report (0 = report ID)
    logic [4:0] rlen;
    logic [7:0] id;
    logic [7:0] r [1:15];      // bytes 1..15 of the current report

    always_ff @(posedge clk) begin
        reset_complete <= 1'b0;
        unknown_id     <= 1'b0;
        short_report   <= 1'b0;
        accel_valid    <= 1'b0;
        gyro_valid     <= 1'b0;
        mag_valid      <= 1'b0;
        fc_valid       <= 1'b0;

        if (rst) begin
            skip           <= 1'b1;
            off            <= '0;
            rlen           <= '0;
            id             <= '0;
            accel_x        <= '0; accel_y <= '0; accel_z <= '0;
            accel_ts       <= '0; accel_status <= '0; accel_seq <= '0;
            gyro_x         <= '0; gyro_y <= '0; gyro_z <= '0; gyro_temp <= '0;
            gyro_ts        <= '0; gyro_status <= '0; gyro_seq <= '0;
            mag_x          <= '0; mag_y <= '0; mag_z <= '0;
            mag_ts         <= '0; mag_status <= '0; mag_seq <= '0;
            fc_id          <= '0;
            fc_interval_us <= '0;
        end else if (rx_valid) begin : byte_in
            logic       sk;
            logic [4:0] o;
            logic [4:0] l;

            // start of packet: decide whether to walk it
            sk = rx_sop ? (rx_cont || !(rx_chan == 8'd2 || rx_chan == 8'd3 || rx_chan == 8'd4)) : skip;
            o  = rx_sop ? 5'd0 : off;

            if (rx_sop && rx_eop && !rx_cont && rx_chan == 8'd1 && rx_data == 8'h01)
                reset_complete <= 1'b1;

            if (!sk) begin
                if (o == 0) begin
                    // report ID
                    l = report_len(rx_data);
                    if (l == 0) begin
                        unknown_id <= 1'b1;
                        sk = 1'b1;
                    end else begin
                        id   <= rx_data;
                        rlen <= l;
                        o    = 5'd1;
                    end
                end else if (o == rlen - 1) begin
                    // last byte of the report (r[] holds bytes 1..o-1)
                    case (id)
                        ID_RAW_ACCEL: begin
                            accel_seq    <= r[1];
                            accel_status <= r[2];
                            accel_x      <= {r[5], r[4]};
                            accel_y      <= {r[7], r[6]};
                            accel_z      <= {r[9], r[8]};
                            accel_ts     <= {rx_data, r[14], r[13], r[12]};
                            accel_valid  <= 1'b1;
                        end
                        ID_RAW_GYRO: begin
                            gyro_seq     <= r[1];
                            gyro_status  <= r[2];
                            gyro_x       <= {r[5], r[4]};
                            gyro_y       <= {r[7], r[6]};
                            gyro_z       <= {r[9], r[8]};
                            gyro_temp    <= {r[11], r[10]};
                            gyro_ts      <= {rx_data, r[14], r[13], r[12]};
                            gyro_valid   <= 1'b1;
                        end
                        ID_RAW_MAG: begin
                            mag_seq      <= r[1];
                            mag_status   <= r[2];
                            mag_x        <= {r[5], r[4]};
                            mag_y        <= {r[7], r[6]};
                            mag_z        <= {r[9], r[8]};
                            mag_ts       <= {rx_data, r[14], r[13], r[12]};
                            mag_valid    <= 1'b1;
                        end
                        ID_GET_FEAT: begin
                            fc_id          <= r[1];
                            fc_interval_us <= {r[8], r[7], r[6], r[5]};
                            fc_valid       <= 1'b1;
                        end
                        default: ;  // FB, FA, F1: nothing to output
                    endcase
                    o = 5'd0;
                end else begin
                    r[o] <= rx_data;
                    o = o + 1;
                end

                if (rx_eop && o != 0 && !sk) short_report <= 1'b1;
            end

            skip <= sk;
            off  <= rx_eop ? 5'd0 : o;
        end
    end

endmodule
