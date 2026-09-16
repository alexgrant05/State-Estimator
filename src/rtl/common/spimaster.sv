module spimaster #(
     parameter CLOCK_DIVIDE = 100, // 100 mhz / 100 = 1 mhz clock
     parameter CPOL = 0,
     parameter CPHA = 0
) (
input sys_clk,
input rst,

//inputs from fabric
input start,
input [7:0] tx_data,
input hold_cs,

//outputs to fabric
output logic busy,
output logic [7:0] rx_data,
output logic rx_valid,

//pin outputs to external device
output logic cs_n,
output logic sclk,
output logic mosi,
input logic miso
);

parameter HALF_CLOCK_PERIOD = CLOCK_DIVIDE / 2;

// state machine: idle -> txn -> idle

typedef enum logic [1:0] {

     IDLE = 2'b00,
     TXN = 2'b01
     //RECIEVE_ONLY = 2'b10;

} state_t;

state_t current_state;

//registers
logic [7:0] tx_data_reg;
logic [7:0] rx_data_reg;

logic hold_cs_reg;


//sclk
bit sclk_internal = CPOL;

//timing bookkeeping
//allow up to clock divide  = 2^16 for very slow signals
logic [15:0] clks_since_last_sclk_edge = 0;
logic [5:0] sclk_edge_count = 0;



always_comb begin
     busy = (current_state == TXN);
     sclk = sclk_internal;
end

always_ff @(posedge sys_clk) begin

     if(rst) begin
          //internal registers
          current_state <= IDLE;
          sclk_internal <= CPOL;
          tx_data_reg <= 0;
          rx_data_reg <= 0;
          clks_since_last_sclk_edge <= 0;
          sclk_edge_count <= 0;
          hold_cs_reg <= 0;

          //output signals
          rx_data <= 0;
          rx_valid <= 0;
          cs_n <= 1;
          mosi <= 0;

     end
     //state machine: handle state transitions

     else if(current_state == IDLE) begin
          //if we recieve a start signal, shift to TXN,
          //initialize it, and latch inputs

          rx_valid <= 0;


          if(start) begin
          //latch start data
          tx_data_reg <= tx_data;
          hold_cs_reg <= hold_cs;

          cs_n <= 0;

          //if CPHA = 0: immediately send out MOSI
          mosi <= tx_data[7]; 

          current_state <= TXN;
          end
     end

     else if(current_state == TXN) begin
          //increment counter
          //if overflow, we want to flip sclk and do any updates
          //cpha will change when we sample and transmit
          if(clks_since_last_sclk_edge != HALF_CLOCK_PERIOD - 1) begin
               clks_since_last_sclk_edge <= clks_since_last_sclk_edge + 1;
               //not on any important clock edge, so do nothing
               rx_valid <= 0;
          end
          else if(sclk_edge_count == 16) begin
               //on next clock tick the transaction is complete,
               //so we do not flip the edge and instead we set cs_n to hold_cs
               //and transition out of the state.
               sclk_edge_count <= 0;
               cs_n <= ~hold_cs_reg;
               current_state <= IDLE;
               rx_valid <= 1;
               rx_data <= rx_data_reg;
          end
          else begin
               rx_valid <= 0;

               sclk_internal <= ~sclk_internal;
               clks_since_last_sclk_edge <= 0;
               sclk_edge_count <= sclk_edge_count + 1;

               //change MOSI if even and CPHA = 0, (nonzero) or if CPHA = 1 and odd
               //keep in mind that sclk_edge_count is one behind.
               //this should be rigorously analyzed in vivado to check the waveform.
               if(CPHA == 0) begin
                    //if previous sclk edge count was even, this one is odd, so read miso
                    if(sclk_edge_count[0] == 0) rx_data_reg <= {rx_data_reg[6:0], miso}; 
                    else if (sclk_edge_count[5:1] < 7) mosi <= tx_data_reg[6-sclk_edge_count[5:1]];
               end
               else begin
                    //CPHA == 1
                    if(sclk_edge_count[0] == 0) mosi <= tx_data_reg[7-sclk_edge_count[5:1]]; 
                    else rx_data_reg <= {rx_data_reg[6:0], miso}; 
               end
          end
     end
end

endmodule