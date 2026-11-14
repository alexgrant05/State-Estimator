`default_nettype none

module record_ring_buffer #(
    parameter int unsigned ADDR_WIDTH = 16
) (
    input  logic                  clk,
    input  logic                  resetn,
    input  logic                  flush,

    input  logic                  write_start,
    input  logic                  write_valid,
    input  logic [7:0]            write_data,
    output logic                  write_ready,
    input  logic                  write_commit,
    input  logic                  write_abort,
    output logic                  write_active,
    output logic                  write_overflow,

    output logic                  read_valid,
    output logic [7:0]            read_data,
    input  logic                  read_ready,
    output logic [ADDR_WIDTH:0]   used_bytes
);
    localparam int unsigned DEPTH = 1 << ADDR_WIDTH;
    logic [7:0] memory [0:DEPTH-1];
    logic [ADDR_WIDTH:0] producer_pointer;
    logic [ADDR_WIDTH:0] consumer_pointer;
    logic [ADDR_WIDTH:0] staging_pointer;
    logic [ADDR_WIDTH:0] committed_used;
    logic [ADDR_WIDTH:0] staged_bytes;
    logic [ADDR_WIDTH:0] occupied_bytes;
    localparam logic [ADDR_WIDTH:0] DEPTH_VALUE = {1'b1, {ADDR_WIDTH{1'b0}}};

    always_comb begin
        committed_used = producer_pointer - consumer_pointer;
        staged_bytes = staging_pointer - producer_pointer;
        occupied_bytes = committed_used + staged_bytes;
        used_bytes = committed_used;
        read_valid = committed_used != 0;
        read_data = memory[consumer_pointer[ADDR_WIDTH-1:0]];
        write_ready = write_active && (occupied_bytes < DEPTH_VALUE);
    end

    always_ff @(posedge clk) begin
        if (!resetn || flush) begin
            producer_pointer <= '0;
            consumer_pointer <= '0;
            staging_pointer <= '0;
            write_active <= 1'b0;
            write_overflow <= 1'b0;
        end else begin
            write_overflow <= 1'b0;
            if (read_valid && read_ready) begin
                consumer_pointer <= consumer_pointer + 1'b1;
            end
            if (write_start && !write_active) begin
                staging_pointer <= producer_pointer;
                write_active <= 1'b1;
            end
            if (write_valid && write_active) begin
                if (write_ready) begin
                    memory[staging_pointer[ADDR_WIDTH-1:0]] <= write_data;
                    staging_pointer <= staging_pointer + 1'b1;
                end else begin
                    write_overflow <= 1'b1;
                end
            end
            if (write_abort && write_active) begin
                staging_pointer <= producer_pointer;
                write_active <= 1'b0;
            end else if (write_commit && write_active) begin
                producer_pointer <= staging_pointer;
                write_active <= 1'b0;
            end
        end
    end
endmodule

`default_nettype wire
