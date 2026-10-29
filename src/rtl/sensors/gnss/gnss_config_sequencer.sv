`timescale 1ns/1ps
`default_nettype none

// gnss_config_sequencer
//
// Plays out a stored UBX configuration script (CFG-VALSET messages:
// output rate, dynamic model, message enables, port protocol config,
// etc.) one byte at a time on start_sequence, for gnss_spi_engine to
// write. The script contents are generated offline, not in this file --
// see the TODO below.
//
// TODO: config_rom.mem / config_rom_last.mem do not exist yet. Needs a
// small generator script (e.g. tools/gen_gnss_config.py) that takes the
// desired UBX CFG-VALSET key/value list and the per-message length
// markers and emits these two $readmemh files.
//
// Inputs:  clk, rst_n, start_sequence, cfg_cmd_ready
// Outputs: sequence_busy, sequence_done, cfg_cmd_data/valid/last
module gnss_config_sequencer #(
    parameter int NUM_BYTES = 256   // sized to fit the full power-on config script
) (
    input  logic       clk,
    input  logic       rst_n,

    input  logic       start_sequence,   // pulse to (re)play the stored config script
    output logic       sequence_busy,
    output logic       sequence_done,

    output logic [7:0] cfg_cmd_data,
    output logic       cfg_cmd_valid,
    input  logic       cfg_cmd_ready,
    output logic       cfg_cmd_last     // final byte of the current UBX message
);

    localparam int ROM_ADDR_WIDTH = $clog2(NUM_BYTES);
    localparam logic [ROM_ADDR_WIDTH-1:0] LAST_ROM_ADDR = ROM_ADDR_WIDTH'(NUM_BYTES - 1);

    logic [7:0] config_rom      [0:NUM_BYTES-1];
    logic       config_rom_last [0:NUM_BYTES-1]; // 1 marks the final byte of each UBX message
    initial $readmemh("config_rom.mem", config_rom);
    initial $readmemh("config_rom_last.mem", config_rom_last);

    logic [ROM_ADDR_WIDTH-1:0] rom_addr;
    logic                         running;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rom_addr      <= '0;
            running       <= 1'b0;
            cfg_cmd_data  <= 8'h0;
            cfg_cmd_valid <= 1'b0;
            cfg_cmd_last  <= 1'b0;
            sequence_done <= 1'b0;
        end else begin
            sequence_done <= 1'b0;

            if (cfg_cmd_valid && cfg_cmd_ready) begin
                cfg_cmd_valid <= 1'b0;
                if (rom_addr == LAST_ROM_ADDR) begin
                    running       <= 1'b0;
                    sequence_done <= 1'b1;
                end else begin
                    rom_addr <= rom_addr + 1'b1;
                end
            end

            if (running && !cfg_cmd_valid) begin
                cfg_cmd_data  <= config_rom[rom_addr];
                cfg_cmd_last  <= config_rom_last[rom_addr];
                cfg_cmd_valid <= 1'b1;
            end

            if (start_sequence && !running) begin
                rom_addr <= '0;
                running  <= 1'b1;
            end
        end
    end

    always_comb sequence_busy = running || cfg_cmd_valid;

endmodule

`default_nettype wire
