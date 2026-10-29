`timescale 1ns/1ps
`default_nettype none

// gnss_health_monitor
//
// Aggregates link and receiver health for PS visibility: synchronizes
// the three async status GPIOs (RTK_STAT, GEOFENCE_STAT, TX_READY_N),
// counts checksum failures reported by ubx_frame_sync, and watches for
// a stuck link (no frame_done seen within TIMEOUT_CYCLES while polling).
//
// RTK_STAT is documented on some ZED-F9P variants as a pulse-coded
// single pin rather than a clean static level -- TODO: confirm the
// exact encoding against the Interface Description / hardware
// integration manual before rtk_stat is trusted as a real field.
//
// Inputs:  clk, rst_n, frame_done, checksum_fail, rtk_stat_raw,
//          geofence_stat_raw, tx_ready_n_raw
// Outputs: rtk_stat, geofence_stat, tx_ready, checksum_fail_count,
//          link_timeout_sticky
module gnss_health_monitor #(
    parameter logic [31:0] TIMEOUT_CYCLES = 32'd200_000_000 // tune to expected GNSS output rate
) (
    input  logic       clk,
    input  logic       rst_n,

    input  logic       frame_done,
    input  logic       checksum_fail,

    input  logic       rtk_stat_raw,
    input  logic       geofence_stat_raw,
    input  logic       tx_ready_n_raw,

    output logic [1:0] rtk_stat,
    output logic       geofence_stat,
    output logic       tx_ready,
    output logic [31:0] checksum_fail_count,
    output logic       link_timeout_sticky
);

    // 2-FF synchronizers for the async GPIO inputs
    logic rtk0, rtk1;
    logic geo0, geo1;
    logic txr0, txr1;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            {rtk1, rtk0} <= 2'b00;
            {geo1, geo0} <= 2'b00;
            {txr1, txr0} <= 2'b11; // tx_ready_n idles high (no data pending)
        end else begin
            rtk0 <= rtk_stat_raw;      rtk1 <= rtk0;
            geo0 <= geofence_stat_raw; geo1 <= geo0;
            txr0 <= tx_ready_n_raw;    txr1 <= txr0;
        end
    end

    always_comb rtk_stat      = {1'b0, rtk1};
    always_comb geofence_stat = geo1;
    always_comb tx_ready      = !txr1;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            checksum_fail_count <= 32'h0;
        end else if (checksum_fail && (checksum_fail_count != 32'hffff_ffff)) begin
            checksum_fail_count <= checksum_fail_count + 1'b1;
        end
    end

    logic [31:0] timeout_counter;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            timeout_counter     <= 32'h0;
            link_timeout_sticky <= 1'b0;
        end else if (frame_done) begin
            timeout_counter <= 32'h0;
        end else if (timeout_counter == TIMEOUT_CYCLES) begin
            link_timeout_sticky <= 1'b1;
        end else begin
            timeout_counter <= timeout_counter + 1'b1;
        end
    end

endmodule

`default_nettype wire
