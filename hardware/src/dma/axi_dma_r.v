`timescale 1ns / 1ps

`include "axi_dma.vh"
`include "system.vh"

// Log2 number of states
`define STATES_W 2

// FSM States
`define IDLE       `STATES_W'h0
`define STANDBY    `STATES_W'h1
`define READ_DATA  `STATES_W'h2

`define NUM_TR `AXI_LEN_W'd16

module axi_dma_r (

	// system inputs
	input                          clk,
	input                          rst,
   
    	// Databus interface
    	output reg                     ready,
    	input	                       valid,
    	input [`DDR_ADDR_W-1:0]        addr,
    	output [`MIG_BUS_W-1:0]        rdata,
   
        // Master Interface Read Address
        output wire [`AXI_ID_W-1:0]    m_axi_arid,
	output wire [`DDR_ADDR_W-1:0]  m_axi_araddr,
	output wire [`AXI_LEN_W-1:0]   m_axi_arlen,
	output wire [`AXI_SIZE_W-1:0]  m_axi_arsize,
	output wire [`AXI_BURST_W-1:0] m_axi_arburst,
	output wire [`AXI_LOCK_W-1:0]  m_axi_arlock,
	output wire [`AXI_CACHE_W-1:0] m_axi_arcache,
	output wire [`AXI_PROT_W-1:0]  m_axi_arprot,
	output wire [`AXI_QOS_W-1:0]   m_axi_arqos,
	output reg                     m_axi_arvalid,
	input wire                     m_axi_arready,

	// Master Interface Read Data
	input wire [`AXI_ID_W-1:0]     m_axi_rid,
	input wire [`MIG_BUS_W-1:0]    m_axi_rdata,
	input wire [`AXI_RESP_W-1:0]   m_axi_rresp,
	input wire                     m_axi_rlast,
	input wire                     m_axi_rvalid,
	output reg                     m_axi_rready
	);
   
   // counter, state and error regs
   reg [`AXI_LEN_W:0]                  counter_int, counter_int_nxt;
   reg [`STATES_W-1:0]                 state, state_nxt;
   reg                                 error, error_nxt;

   // Address read constants
   assign m_axi_arid = `AXI_ID_W'b0;
   assign m_axi_araddr = addr;
   assign m_axi_arlen = `NUM_TR; //number of trasfers per burst
   assign m_axi_arsize = $clog2(`MIG_BUS_W/8); //INCR interval
   assign m_axi_arburst = `AXI_BURST_W'b01; //INCR
   assign m_axi_arlock = `AXI_LOCK_W'b0;
   assign m_axi_arcache = `AXI_CACHE_W'h2;
   assign m_axi_arprot = `AXI_PROT_W'b010;
   assign m_axi_arqos = `AXI_QOS_W'h0;

   // Data read constant
   assign rdata = m_axi_rdata;

   // Counter, error and state registers
   always @ (posedge clk, posedge rst)
     if (rst) begin
       state <= `IDLE;
       counter_int <= {`AXI_LEN_W{1'b0}};
       error <= 1'b0;
     end else begin
       state <= state_nxt;
       counter_int <= counter_int_nxt;
       error <= error_nxt;
     end
   
   // State machine
   always @ * begin
      state_nxt = state;
      error_nxt = error;
      counter_int_nxt = counter_int;
      ready = 1'b0;
      m_axi_arvalid = 1'b0;
      m_axi_rready = 1'b0;
      case (state)
	    `IDLE: begin
       	       counter_int_nxt <= {`AXI_LEN_W{1'b0}};
	       if (valid) begin
	          state_nxt = `STANDBY;
	       end
	    end
	    //addr handshake
	    `STANDBY: begin
	       if (m_axi_arready == 1'b1)
	          state_nxt = `READ_DATA;
	       m_axi_arvalid = 1'b1;
	    end
	    //data read
	    `READ_DATA: begin
	       if (counter_int == `NUM_TR) begin
	          if (m_axi_rlast == 1'b1)
		     error_nxt = 1'b0;
	          else
		     error_nxt = 1'b1;
	          state_nxt = `IDLE;
	       end
	       m_axi_rready = 1'b1;
	       if (m_axi_rvalid == 1'b1) begin
	          ready = 1'b1;
	          counter_int_nxt = counter_int + 1'b1;
	       end
	    end
      endcase
   end
   
endmodule
