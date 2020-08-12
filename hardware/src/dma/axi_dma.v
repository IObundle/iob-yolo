`timescale 1ns / 1ps

`include "axi_dma.vh"
`include "system.vh"

// Log2 number of states
`define STATES_W 2

// FSM States
`define IDLE       `STATES_W'h0
`define STANDBY    `STATES_W'h1
`define INT2EXT    `STATES_W'h2
`define W_RESPONSE `STATES_W'h3

`define NUM_TR `AXI_LEN_W'd16

module axi_dma # (

	parameter		       USE_RAM = 1

	) (

	// system inputs
	input                          clk,
	input                          rst,
   
    	// Databus interface
    	output reg                     ready,
    	input	                       valid,
    	input [`DDR_ADDR_W-1:0]        addr,
    	input [`MIG_BUS_W-1:0]         wdata,
    	input [`MIG_BUS_W/8-1:0]       wstrb,
   
	// Master Interface Write Address
	output wire [`AXI_ID_W-1:0]    m_axi_awid,
	output wire [`DDR_ADDR_W-1:0]  m_axi_awaddr,
	output wire [`AXI_LEN_W-1:0]   m_axi_awlen,
	output wire [`AXI_SIZE_W-1:0]  m_axi_awsize,
	output wire [`AXI_BURST_W-1:0] m_axi_awburst,
	output wire [`AXI_LOCK_W-1:0]  m_axi_awlock,
	output wire [`AXI_CACHE_W-1:0] m_axi_awcache,
	output wire [`AXI_PROT_W-1:0]  m_axi_awprot,
	output wire [`AXI_QOS_W-1:0]   m_axi_awqos,
	output reg                     m_axi_awvalid,
	input wire                     m_axi_awready,

	// Master Interface Write Data
	output wire [`MIG_BUS_W-1:0]   m_axi_wdata,
	output reg [`MIG_BUS_W/8-1:0]  m_axi_wstrb,
	output reg                     m_axi_wlast,
	output reg                     m_axi_wvalid,
	input wire                     m_axi_wready,

	// Master Interface Write Response
	input wire [`AXI_ID_W-1:0]     m_axi_bid,
	input wire [`AXI_RESP_W-1:0]   m_axi_bresp,
	input wire                     m_axi_bvalid,
	output reg                     m_axi_bready
	);
   
   // counter, state and errorsregs
   reg [`AXI_LEN_W:0]                  counter_int, counter_int_nxt;
   reg [`STATES_W-1:0]                 state, state_nxt;
   reg                                 error, error_nxt;

   // data write delay regs
   reg                                 m_axi_wvalid_int;
   reg                                 m_axi_wlast_int;
   reg				       ready_int, ready_r;
   assign 			       ready = USE_RAM ? ready_int : ready_r;

   // Address write constants
   assign m_axi_awid = `AXI_ID_W'b0;
   assign m_axi_awaddr = addr;
   assign m_axi_awlen = `NUM_TR; //number of trasfers per burst
   assign m_axi_awsize = $clog2(`MIG_BUS_W/8); //INCR interval
   assign m_axi_awburst = `AXI_BURST_W'b01; //INCR
   assign m_axi_awlock = `AXI_LOCK_W'b0;
   assign m_axi_awcache = `AXI_CACHE_W'h2;
   assign m_axi_awprot = `AXI_PROT_W'b010;
   assign m_axi_awqos = `AXI_QOS_W'h0;

   // Data write constants
   assign m_axi_wdata = wdata;

   // delays
   always @ (posedge rst, posedge clk)
     if (rst) begin
       m_axi_wstrb <= {`MIG_BUS_W/8{1'b0}};
       m_axi_wlast <= 0;
       m_axi_wvalid <= 0;
     end else begin
       m_axi_wstrb <= wstrb;
       m_axi_wlast <= m_axi_wlast_int;
       m_axi_wvalid <= m_axi_wvalid_int;
     end
   
   // Counter, error, state and ready registers
   always @ (posedge clk, posedge rst)
     if (rst) begin
       state <= `IDLE;
       counter_int <= {`AXI_LEN_W{1'b0}};
       error <= 1'b0;
       ready_r <= 1'b0;
     end else begin
       state <= state_nxt;
       counter_int <= counter_int_nxt;
       error <= error_nxt;
       ready_r <= ready_int;
     end
   
   // State machine
   always @ * begin
      state_nxt = state;
      error_nxt = error;
      counter_int_nxt = counter_int;
      ready_int = 1'b0;
      m_axi_awvalid = 1'b0;
      m_axi_wvalid_int = 1'b0;
      m_axi_wlast_int = 1'b0;
      m_axi_bready = 1'b1;
      case (state)
	    `IDLE: begin
       	       counter_int_nxt <= {`AXI_LEN_W{1'b0}};
	       if (valid) begin
	          state_nxt = `STANDBY;
	       end
	    end
	    //addr handshake
	    `STANDBY: begin
	       if (m_axi_awready == 1'b1)
	          state_nxt = `INT2EXT;
	       m_axi_awvalid = 1'b1;
	    end
	    //data write
	    `INT2EXT: begin
	       if (counter_int == `NUM_TR) begin
	          m_axi_wlast_int = 1'b1;
	          state_nxt = `W_RESPONSE;
	       end
	       if (m_axi_wready == 1'b1) begin
	          m_axi_wvalid_int = 1'b1;
	          ready_int = 1'b1;
	          counter_int_nxt = counter_int + 1'b1;
	       end
	    end
	    //write response
	    `W_RESPONSE: begin
	       if (m_axi_bvalid == 1'b1) begin
	          if (m_axi_bresp == `AXI_RESP_W'b00)
		    error_nxt = 1'b0;
	          else
		    error_nxt = 1'b1;
	          state_nxt = `IDLE;
	       end
	    end
      endcase
   end
   
endmodule
