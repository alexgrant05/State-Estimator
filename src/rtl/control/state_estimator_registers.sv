`default_nettype none

module state_estimator_registers #(
    parameter logic [31:0] CORE_VERSION_VALUE = 32'h0001_0000,
    parameter logic [127:0] BUILD_ID_VALUE = 128'b0
) (
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ACLK, ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET aresetn" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ACLK CLK" *)
    input  wire          aclk,
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ARESETN, POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 ARESETN RST" *)
    input  wire          aresetn,

    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI, PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 16, FREQ_HZ 100000000, READ_WRITE_MODE READ_WRITE" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    input  wire [15:0]   s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input  wire          s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output logic         s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input  wire [31:0]   s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input  wire [3:0]    s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input  wire          s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output logic         s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output logic [1:0]   s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output logic         s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire          s_axi_bready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [15:0]   s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire          s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output logic         s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output logic [31:0]  s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output logic [1:0]   s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output logic         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire          s_axi_rready,

    input  wire          pps_pulse,
    input  wire [31:0]   error_set,
    input  wire [4:0]    irq_set,
    input  wire [31:0]   packet_count,
    input  wire [31:0]   r5_drop_count,
    input  wire [31:0]   linux_drop_count,
    input  wire [31:0]   dma_block_count,
    input  wire [31:0]   backpressure_count,
    input  wire [31:0]   host_overrun_count,
    input  wire [31:0]   event_overflow_count,
    input  wire [31:0]   packet_overflow_count,
    input  wire [31:0]   last_fatal,

    output logic [31:0]  control,
    output logic [31:0]  session_id,
    output logic         soft_reset_pulse,
    output logic         configuration_locked,
    output logic [4:0]   irq
);
    import state_estimator_abi_pkg::*;

    localparam logic [1:0] AXI_OKAY = 2'b00;
    localparam logic [1:0] AXI_SLVERR = 2'b10;
    localparam logic [31:0] ABI_VERSION_VALUE = {16'd1, 16'd0};
    localparam logic [31:0] CAPABILITIES_VALUE = 32'h0000_003F;
    localparam logic [31:0] SENSOR_PRESENT_VALUE = 32'h0000_01CD;

    logic [63:0] timebase;
    logic [63:0] last_pps_ticks;
    logic [31:0] pps_count;
    logic [31:0] reset_generation;
    logic [31:0] error_status;
    logic [31:0] error_enable;
    logic [31:0] irq_status;
    logic [31:0] irq_enable;
    logic [31:0] irq_coalesce_blocks;
    logic [31:0] irq_coalesce_ticks;
    logic [31:0] scratch;
    logic [31:0] time_status;
    logic [31:0] time_read_latch;
    logic [31:0] pps_read_latch;
    logic [4:0] reset_key_window;

    logic aw_stored;
    logic [15:0] awaddr_stored;
    logic w_stored;
    logic [31:0] wdata_stored;
    logic [3:0] wstrb_stored;

    function automatic logic readable_address(input logic [15:0] address);
        case (address)
            REG_CORE_MAGIC, REG_ABI_VERSION, REG_CORE_VERSION, REG_CAPABILITIES,
            REG_CLOCK_HZ, REG_SENSOR_PRESENT, REG_BUILD_ID0, REG_BUILD_ID1,
            REG_BUILD_ID2, REG_BUILD_ID3, REG_ABI_HASH_LO, REG_ABI_HASH_HI,
            REG_SESSION_ID, REG_RESET_GENERATION, REG_CONTROL, REG_STATUS,
            REG_ERROR_STATUS, REG_ERROR_ENABLE, REG_IRQ_STATUS, REG_IRQ_ENABLE,
            REG_IRQ_COALESCE_BLOCKS, REG_IRQ_COALESCE_TICKS, REG_SCRATCH,
            REG_TIME_NOW_LO, REG_TIME_NOW_HI, REG_LAST_PPS_TICKS_LO,
            REG_LAST_PPS_TICKS_HI, REG_PPS_COUNT, REG_TIME_STATUS,
            REG_PACKET_COUNT, REG_R5_DROP_COUNT, REG_LINUX_DROP_COUNT,
            REG_DMA_BLOCK_COUNT, REG_BACKPRESSURE_COUNT, REG_HOST_OVERRUN_COUNT,
            REG_EVENT_OVERFLOW_COUNT, REG_PACKET_OVERFLOW_COUNT, REG_LAST_FATAL,
            REG_RESERVED_COUNTER: readable_address = 1'b1;
            default: readable_address = 1'b0;
        endcase
    endfunction

    function automatic logic writable_address(input logic [15:0] address);
        case (address)
            REG_SESSION_ID, REG_CONTROL, REG_ERROR_STATUS, REG_ERROR_ENABLE,
            REG_IRQ_STATUS, REG_IRQ_ENABLE, REG_IRQ_FORCE,
            REG_IRQ_COALESCE_BLOCKS, REG_IRQ_COALESCE_TICKS, REG_SCRATCH,
            REG_RESET_KEY, REG_RESET_CONTROL: writable_address = 1'b1;
            default: writable_address = 1'b0;
        endcase
    endfunction

    function automatic logic [31:0] register_read(input logic [15:0] address);
        case (address)
            REG_CORE_MAGIC: register_read = CORE_MAGIC[31:0];
            REG_ABI_VERSION: register_read = ABI_VERSION_VALUE;
            REG_CORE_VERSION: register_read = CORE_VERSION_VALUE;
            REG_CAPABILITIES: register_read = CAPABILITIES_VALUE;
            REG_CLOCK_HZ: register_read = CLOCK_HZ[31:0];
            REG_SENSOR_PRESENT: register_read = SENSOR_PRESENT_VALUE;
            REG_BUILD_ID0: register_read = BUILD_ID_VALUE[31:0];
            REG_BUILD_ID1: register_read = BUILD_ID_VALUE[63:32];
            REG_BUILD_ID2: register_read = BUILD_ID_VALUE[95:64];
            REG_BUILD_ID3: register_read = BUILD_ID_VALUE[127:96];
            REG_ABI_HASH_LO: register_read = state_estimator_abi_pkg::ABI_HASH_LO;
            REG_ABI_HASH_HI: register_read = state_estimator_abi_pkg::ABI_HASH_HI;
            REG_SESSION_ID: register_read = session_id;
            REG_RESET_GENERATION: register_read = reset_generation;
            REG_CONTROL: register_read = control;
            REG_STATUS: register_read = {25'b0, |error_status, pps_count != 0, control[3], control[2], configuration_locked, |control[2:0], 1'b1};
            REG_ERROR_STATUS: register_read = error_status;
            REG_ERROR_ENABLE: register_read = error_enable;
            REG_IRQ_STATUS: register_read = irq_status;
            REG_IRQ_ENABLE: register_read = irq_enable;
            REG_IRQ_COALESCE_BLOCKS: register_read = irq_coalesce_blocks;
            REG_IRQ_COALESCE_TICKS: register_read = irq_coalesce_ticks;
            REG_SCRATCH: register_read = scratch;
            REG_TIME_NOW_LO: register_read = timebase[31:0];
            REG_TIME_NOW_HI: register_read = time_read_latch;
            REG_LAST_PPS_TICKS_LO: register_read = last_pps_ticks[31:0];
            REG_LAST_PPS_TICKS_HI: register_read = pps_read_latch;
            REG_PPS_COUNT: register_read = pps_count;
            REG_TIME_STATUS: register_read = time_status;
            REG_PACKET_COUNT: register_read = packet_count;
            REG_R5_DROP_COUNT: register_read = r5_drop_count;
            REG_LINUX_DROP_COUNT: register_read = linux_drop_count;
            REG_DMA_BLOCK_COUNT: register_read = dma_block_count;
            REG_BACKPRESSURE_COUNT: register_read = backpressure_count;
            REG_HOST_OVERRUN_COUNT: register_read = host_overrun_count;
            REG_EVENT_OVERFLOW_COUNT: register_read = event_overflow_count;
            REG_PACKET_OVERFLOW_COUNT: register_read = packet_overflow_count;
            REG_LAST_FATAL: register_read = last_fatal;
            default: register_read = 32'b0;
        endcase
    endfunction

    always_comb begin
        s_axi_awready = !aw_stored && !s_axi_bvalid;
        s_axi_wready = !w_stored && !s_axi_bvalid;
        s_axi_arready = !s_axi_rvalid;
        irq = irq_status[4:0] & irq_enable[4:0];
        configuration_locked = control[4];
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            timebase <= 64'b0;
            last_pps_ticks <= 64'b0;
            pps_count <= 32'b0;
            reset_generation <= 32'b0;
            control <= 32'b0;
            session_id <= 32'b0;
            error_status <= 32'b0;
            error_enable <= 32'b0;
            irq_status <= 32'b0;
            irq_enable <= 32'b0;
            irq_coalesce_blocks <= 32'd1;
            irq_coalesce_ticks <= 32'd500_000;
            scratch <= 32'b0;
            time_status <= 32'b0;
            time_read_latch <= 32'b0;
            pps_read_latch <= 32'b0;
            reset_key_window <= 5'b0;
            aw_stored <= 1'b0;
            awaddr_stored <= 16'b0;
            w_stored <= 1'b0;
            wdata_stored <= 32'b0;
            wstrb_stored <= 4'b0;
            s_axi_bresp <= AXI_OKAY;
            s_axi_bvalid <= 1'b0;
            s_axi_rdata <= 32'b0;
            s_axi_rresp <= AXI_OKAY;
            s_axi_rvalid <= 1'b0;
            soft_reset_pulse <= 1'b0;
        end else begin
            timebase <= timebase + 1'b1;
            soft_reset_pulse <= 1'b0;
            error_status <= error_status | error_set;
            irq_status <= irq_status | {27'b0, irq_set};
            if (reset_key_window != 0) reset_key_window <= reset_key_window - 1'b1;
            if (pps_pulse) begin
                last_pps_ticks <= timebase;
                pps_count <= pps_count + 1'b1;
                time_status[0] <= 1'b1;
                irq_status <= irq_status | IRQ_TIMEPULSE;
            end

            if (s_axi_awready && s_axi_awvalid) begin
                awaddr_stored <= s_axi_awaddr;
                aw_stored <= 1'b1;
            end
            if (s_axi_wready && s_axi_wvalid) begin
                wdata_stored <= s_axi_wdata;
                wstrb_stored <= s_axi_wstrb;
                w_stored <= 1'b1;
            end
            if (aw_stored && w_stored && !s_axi_bvalid) begin
                s_axi_bresp <= AXI_OKAY;
                if ((awaddr_stored[1:0] != 2'b00) || (wstrb_stored != 4'hF) || !writable_address(awaddr_stored)) begin
                    s_axi_bresp <= AXI_SLVERR;
                    error_status <= error_status | ERROR_ACCESS_VIOLATION;
                end else begin
                    case (awaddr_stored)
                        REG_SESSION_ID: begin
                            if (wdata_stored == 0 || configuration_locked) begin
                                s_axi_bresp <= AXI_SLVERR;
                                error_status <= error_status | ERROR_ACCESS_VIOLATION;
                            end else session_id <= wdata_stored;
                        end
                        REG_CONTROL: begin
                            if (configuration_locked && wdata_stored != control) begin
                                s_axi_bresp <= AXI_SLVERR;
                                error_status <= error_status | ERROR_ACCESS_VIOLATION;
                            end else control <= control[4] ? control : (wdata_stored & 32'h0000_001F);
                        end
                        REG_ERROR_STATUS: error_status <= (error_status | error_set) & ~wdata_stored;
                        REG_ERROR_ENABLE: error_enable <= wdata_stored;
                        REG_IRQ_STATUS: irq_status <= (irq_status | {27'b0, irq_set}) & ~wdata_stored;
                        REG_IRQ_ENABLE: irq_enable <= wdata_stored & 32'h0000_001F;
                        REG_IRQ_FORCE: begin
                            if (!control[3] || configuration_locked) begin
                                s_axi_bresp <= AXI_SLVERR;
                                error_status <= error_status | ERROR_ACCESS_VIOLATION;
                            end else irq_status <= irq_status | (wdata_stored & 32'h0000_001F);
                        end
                        REG_IRQ_COALESCE_BLOCKS: irq_coalesce_blocks <= wdata_stored;
                        REG_IRQ_COALESCE_TICKS: irq_coalesce_ticks <= wdata_stored;
                        REG_SCRATCH: scratch <= wdata_stored;
                        REG_RESET_KEY: begin
                            if (wdata_stored == RESET_KEY[31:0]) reset_key_window <= 5'd16;
                            else begin
                                s_axi_bresp <= AXI_SLVERR;
                                error_status <= error_status | ERROR_ACCESS_VIOLATION;
                            end
                        end
                        REG_RESET_CONTROL: begin
                            if (reset_key_window == 0 || configuration_locked || wdata_stored != 1) begin
                                s_axi_bresp <= AXI_SLVERR;
                                error_status <= error_status | ERROR_ACCESS_VIOLATION;
                            end else begin
                                soft_reset_pulse <= 1'b1;
                                reset_generation <= reset_generation + 1'b1;
                                control <= 32'b0;
                                reset_key_window <= 5'b0;
                            end
                        end
                        default: error_status <= error_status | ERROR_ACCESS_VIOLATION;
                    endcase
                end
                aw_stored <= 1'b0;
                w_stored <= 1'b0;
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (s_axi_arready && s_axi_arvalid) begin
                s_axi_rvalid <= 1'b1;
                if (s_axi_araddr[1:0] != 2'b00 || !readable_address(s_axi_araddr)) begin
                    s_axi_rdata <= 32'b0;
                    s_axi_rresp <= AXI_SLVERR;
                    error_status <= error_status | ERROR_ACCESS_VIOLATION;
                end else begin
                    s_axi_rdata <= register_read(s_axi_araddr);
                    s_axi_rresp <= AXI_OKAY;
                    if (s_axi_araddr == REG_TIME_NOW_LO) time_read_latch <= timebase[63:32];
                    if (s_axi_araddr == REG_LAST_PPS_TICKS_LO) pps_read_latch <= last_pps_ticks[63:32];
                end
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
endmodule

`default_nettype wire
