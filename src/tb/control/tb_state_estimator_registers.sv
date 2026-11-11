`timescale 1ns/1ps
`default_nettype none

module tb_state_estimator_registers;
    import state_estimator_abi_pkg::*;

    logic aclk = 1'b0;
    logic aresetn = 1'b0;
    logic [15:0] s_axi_awaddr = 16'b0;
    logic s_axi_awvalid = 1'b0;
    logic s_axi_awready;
    logic [31:0] s_axi_wdata = 32'b0;
    logic [3:0] s_axi_wstrb = 4'hF;
    logic s_axi_wvalid = 1'b0;
    logic s_axi_wready;
    logic [1:0] s_axi_bresp;
    logic s_axi_bvalid;
    logic s_axi_bready = 1'b0;
    logic [15:0] s_axi_araddr = 16'b0;
    logic s_axi_arvalid = 1'b0;
    logic s_axi_arready;
    logic [31:0] s_axi_rdata;
    logic [1:0] s_axi_rresp;
    logic s_axi_rvalid;
    logic s_axi_rready = 1'b0;
    logic pps_pulse = 1'b0;
    logic [31:0] error_set = 32'b0;
    logic [4:0] irq_set = 5'b0;
    logic [31:0] packet_count = 32'd12;
    logic [31:0] r5_drop_count = 32'b0;
    logic [31:0] linux_drop_count = 32'b0;
    logic [31:0] dma_block_count = 32'b0;
    logic [31:0] backpressure_count = 32'b0;
    logic [31:0] host_overrun_count = 32'b0;
    logic [31:0] event_overflow_count = 32'b0;
    logic [31:0] packet_overflow_count = 32'b0;
    logic [31:0] last_fatal = 32'b0;
    logic [31:0] control;
    logic [31:0] session_id;
    logic soft_reset_pulse;
    logic configuration_locked;
    logic [4:0] irq;
    logic saw_soft_reset = 1'b0;

    always #5 aclk <= ~aclk;
    always_ff @(posedge aclk) if (soft_reset_pulse) saw_soft_reset <= 1'b1;

    state_estimator_registers dut (.*);

    task automatic axi_write(input logic [15:0] address, input logic [31:0] value, output logic [1:0] response);
        begin
            @(negedge aclk);
            s_axi_awaddr = address;
            s_axi_wdata = value;
            s_axi_awvalid = 1'b1;
            s_axi_wvalid = 1'b1;
            while (!s_axi_awready || !s_axi_wready) @(negedge aclk);
            @(posedge aclk);
            @(negedge aclk);
            s_axi_awvalid = 1'b0;
            s_axi_wvalid = 1'b0;
            while (!s_axi_bvalid) @(negedge aclk);
            response = s_axi_bresp;
            s_axi_bready = 1'b1;
            @(negedge aclk);
            s_axi_bready = 1'b0;
        end
    endtask

    task automatic axi_read(input logic [15:0] address, output logic [31:0] value, output logic [1:0] response);
        begin
            @(negedge aclk);
            s_axi_araddr = address;
            s_axi_arvalid = 1'b1;
            while (!s_axi_arready) @(negedge aclk);
            @(posedge aclk);
            @(negedge aclk);
            s_axi_arvalid = 1'b0;
            while (!s_axi_rvalid) @(negedge aclk);
            value = s_axi_rdata;
            response = s_axi_rresp;
            s_axi_rready = 1'b1;
            @(negedge aclk);
            s_axi_rready = 1'b0;
        end
    endtask

    initial begin
        logic [31:0] value;
        logic [1:0] response;
        repeat (3) @(posedge aclk);
        aresetn = 1'b1;

        axi_read(REG_CORE_MAGIC, value, response);
        if (response != 0 || value != CORE_MAGIC[31:0]) $fatal(1, "core identity mismatch");
        axi_read(REG_PACKET_COUNT, value, response);
        if (value != 12) $fatal(1, "counter input mismatch");
        axi_write(REG_SCRATCH, 32'hA5A5_5A5A, response);
        axi_read(REG_SCRATCH, value, response);
        if (response != 0 || value != 32'hA5A5_5A5A) $fatal(1, "scratch test failed");
        axi_write(REG_CORE_MAGIC, 32'b0, response);
        if (response != 2'b10) $fatal(1, "RO write did not return SLVERR");

        axi_write(REG_RESET_KEY, RESET_KEY[31:0], response);
        axi_write(REG_RESET_CONTROL, 32'd1, response);
        if (response != 0 || !saw_soft_reset) $fatal(1, "soft reset sequence failed");
        axi_read(REG_RESET_GENERATION, value, response);
        if (value != 1) $fatal(1, "reset generation did not increment");

        axi_write(REG_SESSION_ID, 32'h1234_5678, response);
        if (session_id != 32'h1234_5678) $fatal(1, "session ID write failed");
        axi_write(REG_CONTROL, CONTROL_CONFIGURATION_LOCK, response);
        if (!configuration_locked || control != CONTROL_CONFIGURATION_LOCK) $fatal(1, "lock did not arm");
        axi_write(REG_SCRATCH, 32'hFFFF_FFFF, response);
        if (response != 0) $fatal(1, "scratch must remain available after lock");
        axi_write(REG_CONTROL, 32'b0, response);
        if (response != 2'b10 || !configuration_locked) $fatal(1, "lock was not permanent");
        if (^irq === 1'bx) $fatal(1, "IRQ contains unknown bits");
        $display("PASS tb_state_estimator_registers");
        $finish;
    end
endmodule

`default_nettype wire
