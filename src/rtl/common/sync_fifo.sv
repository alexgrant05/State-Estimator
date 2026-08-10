`timescale 1ns/1ps
`default_nettype none

module sync_fifo #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 4,
    localparam int unsigned POINTER_WIDTH = (DEPTH <= 2) ? 1 : $clog2(DEPTH),
    localparam int unsigned LEVEL_WIDTH = $clog2(DEPTH + 1)
) (
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         clear_status,
    input  logic [DATA_WIDTH-1:0]        input_data,
    input  logic                         input_valid,
    output logic                         input_ready,
    output logic [DATA_WIDTH-1:0]        output_data,
    output logic                         output_valid,
    input  logic                         output_ready,
    output logic [LEVEL_WIDTH-1:0]       level,
    output logic                         overflow_pulse,
    output logic                         underflow_pulse,
    output logic                         overflow_sticky,
    output logic                         underflow_sticky
);

    logic [DATA_WIDTH-1:0] memory [0:DEPTH-1];
    logic [POINTER_WIDTH-1:0] write_pointer;
    logic [POINTER_WIDTH-1:0] read_pointer;
    logic push;
    logic pop;
    localparam logic [POINTER_WIDTH-1:0] LAST_POINTER = POINTER_WIDTH'(DEPTH - 1);
    localparam logic [LEVEL_WIDTH-1:0] DEPTH_LEVEL = LEVEL_WIDTH'(DEPTH);

    initial begin
        if (DEPTH < 2) begin
            $error("sync_fifo requires DEPTH >= 2");
        end
    end

    assign output_valid = (level != '0);
    assign output_data  = memory[read_pointer];
    assign input_ready  = (level < DEPTH_LEVEL) || (output_valid && output_ready);
    assign push         = input_valid && input_ready;
    assign pop          = output_valid && output_ready;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            write_pointer    <= '0;
            read_pointer     <= '0;
            level            <= '0;
            overflow_pulse   <= 1'b0;
            underflow_pulse  <= 1'b0;
            overflow_sticky  <= 1'b0;
            underflow_sticky <= 1'b0;
        end else begin
            overflow_pulse  <= input_valid && !input_ready;
            underflow_pulse <= output_ready && !output_valid;

            if (clear_status) begin
                overflow_sticky  <= 1'b0;
                underflow_sticky <= 1'b0;
            end
            if (input_valid && !input_ready) begin
                overflow_sticky <= 1'b1;
            end
            if (output_ready && !output_valid) begin
                underflow_sticky <= 1'b1;
            end

            if (push) begin
                memory[write_pointer] <= input_data;
                if (write_pointer == LAST_POINTER) begin
                    write_pointer <= '0;
                end else begin
                    write_pointer <= write_pointer + 1'b1;
                end
            end

            if (pop) begin
                if (read_pointer == LAST_POINTER) begin
                    read_pointer <= '0;
                end else begin
                    read_pointer <= read_pointer + 1'b1;
                end
            end

            unique case ({push, pop})
                2'b10: level <= level + 1'b1;
                2'b01: level <= level - 1'b1;
                default: level <= level;
            endcase
        end
    end

endmodule

`default_nettype wire
