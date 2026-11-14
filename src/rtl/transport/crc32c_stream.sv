`default_nettype none

module crc32c_stream (
    input  logic        clk,
    input  logic        resetn,
    input  logic        start,
    input  logic        data_valid,
    input  logic [7:0]  data_byte,
    input  logic        data_last,
    output logic [31:0] crc,
    output logic        crc_valid
);
    logic [31:0] state;

    function automatic logic [31:0] update_crc(
        input logic [31:0] previous,
        input logic [7:0] value
    );
        logic [31:0] next_value;
        integer bit_index;
        begin
            next_value = previous ^ {24'b0, value};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                if (next_value[0]) begin
                    next_value = (next_value >> 1) ^ 32'h82F6_3B78;
                end else begin
                    next_value = next_value >> 1;
                end
            end
            return next_value;
        end
    endfunction

    always_ff @(posedge clk) begin
        if (!resetn) begin
            state <= 32'hFFFF_FFFF;
            crc <= 32'b0;
            crc_valid <= 1'b0;
        end else begin
            crc_valid <= 1'b0;
            if (start) begin
                state <= 32'hFFFF_FFFF;
            end
            if (data_valid) begin
                state <= update_crc(start ? 32'hFFFF_FFFF : state, data_byte);
                if (data_last) begin
                    crc <= update_crc(start ? 32'hFFFF_FFFF : state, data_byte) ^ 32'hFFFF_FFFF;
                    crc_valid <= 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
