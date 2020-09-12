`timescale 1ns / 1ps

`include "axi_dma.vh"
`include "system.vh"

// Log2 number of states
`define ALIGN_STATES_W 3

//FSM States
`define ALGN_IDLE `ALIGN_STATES_W'h0 // Default, reset, end write, wait for dbus_valid
`define ALGN_CONFIG `ALIGN_STATES_W'h1 // set LEN, register dbus data
`define ALGN_FIRST `ALIGN_STATES_W'h2 // Send first message
`define ALGN_TRANSFER `ALIGN_STATES_W'h3 // Send other burst messages

module wdata_aligner #(
		       parameter ADDR_W = 32,
		       parameter DATA_W = 32
		       )(
			 // system inputs
			 input 		       clk,
			 input 		       rst,

			 //control
			 input 		       clear,
			 input 		       run,
			 input [ADDR_W-1:0]    endAddr,
			 
			 // Databus interface
			 input 		       dbus_valid,
			 input [ADDR_W-1:0]    dbus_addr,
			 input [DATA_W-1:0]    dbus_wdata,
			 input [DATA_W/8-1:0]  dbus_wstrb,
			 output 	       dbus_ready,

 			 // DMA interface
			 output 	       dma_w_valid,
			 output [ADDR_W-1:0]   dma_w_addr,
			 output [DATA_W-1:0]   dma_w_wdata,
			 output [DATA_W/8-1:0] dma_w_wstrb,
			 input 		       dma_w_ready,
			 // DMA len
			 output [`AXI_LEN_W-1:0]   dma_w_len
			 );

   localparam OFFSET_W = $clog2(DATA_W/8);
   
   
   // aux registers
   reg 						   cfg_en, buffer_en, len_cnt_en;
   reg [DATA_W/8-1:0] 				   first_wstrb, last_wstrb;
   reg [ADDR_W-1:0] 				   len, len_cnt;
   reg [(OFFSET_W+3)-1:0] 			   wdata_offset;
   
   //register dbus
   reg 						   buffer_r0_valid, buffer_r1_valid;
   reg [ADDR_W-1:0] 				   buffer_r0_addr, buffer_r1_addr;
   reg [2*DATA_W-8-1:0] 			   buffer_r0_wdata, buffer_r1_wdata;
   reg [DATA_W/8-1:0] 				   buffer_r0_wstrb, buffer_r1_wstrb;
   
   //state registers
   reg [`ALIGN_STATES_W-1:0] 			   state, state_nxt;
   reg 						   w_align_valid;
   reg [DATA_W/8-1:0] 				   w_align_wstrb;
   
   
   // update registers
   always @(posedge clk, posedge rst)
     if(rst) begin
	state <= `ALGN_IDLE;
	len <= {ADDR_W{1'b0}};
	len_cnt <= {ADDR_W{1'b0}};
	first_wstrb <= {DATA_W/8{1'b0}};
	last_wstrb <= {DATA_W/8{1'b0}};
     end else begin
	state <= state_nxt;
	if(cfg_en) begin
	   // calculate len (aligned end- alined start)/Bytes per transfer
	   len <= ({endAddr[ADDR_W-1:OFFSET_W], {OFFSET_W{1'b0}}} - {buffer_r0_addr[ADDR_W-1:OFFSET_W], {OFFSET_W{1'b0}}}) >> OFFSET_W;
	   len_cnt <= ({endAddr[ADDR_W-1:OFFSET_W], {OFFSET_W{1'b0}}} - {buffer_r0_addr[ADDR_W-1:OFFSET_W], {OFFSET_W{1'b0}}}) >> OFFSET_W;
	   // calculate first and last wstrbs
	   first_wstrb <= {DATA_W/8{1'b1}} >> buffer_r0_addr[0+:OFFSET_W];
	   last_wstrb <= $signed({1'b1, {(DATA_W/8-1){1'b0}}}) >>> endAddr[0+:OFFSET_W];
	end else if(len_cnt_en) begin
	   len <= len;
	   first_wstrb <= first_wstrb;
	   last_wstrb <= last_wstrb;
	   if(dma_w_ready) begin
	      len_cnt = len_cnt - 1;
	   end else begin
	      len_cnt <= len_cnt;
	   end
	end else begin
	   len <= len;
	   len_cnt <= len_cnt;
	   first_wstrb <= first_wstrb;
	   last_wstrb <= last_wstrb;
	end
	
     end
   
   // Stage machine
   always @ * begin
      state_nxt = state;
      cfg_en = 1'b0;
      buffer_en = 1'b0;
      w_align_valid = 1'b0;
      w_align_wstrb = {DATA_W/8{1'b0}};
      wdata_offset = wdata_offset;
      case (state)
	`ALGN_IDLE: begin //wait for dbus_valid
	   if(dbus_valid) begin
	      state_nxt = `ALGN_CONFIG;
	      buffer_en = 1'b1; //register first write request
	      wdata_offset = {~dbus_addr[0+:OFFSET_W], 3'b0};
	   end
	end
	`ALGN_CONFIG: begin //set len, offset, first and last wstrbs
	   buffer_en = 1'b1;
	   cfg_en = 1'b1;
	   state_nxt = `ALGN_FIRST;	   
	end
	`ALGN_FIRST: begin //send first transfer
	   len_cnt_en = 1'b1;
	   w_align_valid = buffer_r1_valid;
	   if(len_cnt=={ADDR_W{1'b0}}) begin // case of single transfer: burst length==1
	      w_align_wstrb = buffer_r1_wstrb & first_wstrb & last_wstrb;
	      if(dma_w_ready) begin
		 state_nxt = `ALGN_IDLE;
	      end
	   end else begin // first != last transfer
	      w_align_wstrb = buffer_r1_wstrb & first_wstrb;
	      if(dma_w_ready) begin //wait for ready
		 state_nxt = `ALGN_TRANSFER;
	      end
	   end
	end
	`ALGN_TRANSFER: begin
	   w_align_valid = buffer_r1_valid;
	   if(dma_w_ready && len_cnt == {ADDR_W{1'b0}}) begin // last transfer
	      w_align_wstrb = buffer_r1_wstrb & last_wstrb;
	      if(dma_w_ready) begin
		 state_nxt = `ALGN_IDLE;
	      end
	   end else begin // middle transfers
	      w_align_wstrb = buffer_r1_wstrb;
	   end
	end
	default: begin
	   state_nxt = `ALGN_IDLE;
	end
      endcase
   end // always @ *

   // Update buffer registers
   always @(posedge clk, posedge rst) begin
      if(rst || (state_nxt == `ALGN_IDLE)) begin
	 buffer_r0_valid <= 1'b0;
	 buffer_r1_valid <= 1'b0;
	 buffer_r0_addr <= {ADDR_W{1'b0}};
	 buffer_r1_addr <= {ADDR_W{1'b0}};
	 buffer_r0_wdata <= {(2*DATA_W-8){1'b0}};
	 buffer_r1_wdata <= {(2*DATA_W-8){1'b0}};
	 buffer_r0_wstrb <= {(DATA_W/8){1'b0}};
	 buffer_r1_wstrb <= {(DATA_W/8){1'b0}};	 
      end else if(buffer_en || dma_w_ready) begin
	 buffer_r0_valid <= dbus_valid;
	 buffer_r0_addr <= dbus_addr;
	 buffer_r0_wdata <= {buffer_r0_wdata[0+:(DATA_W-8)], dbus_wdata};
	 buffer_r0_wstrb <= dbus_wstrb;
	 buffer_r1_valid <= buffer_r0_valid;
	 buffer_r1_addr <= {buffer_r0_addr[ADDR_W-1:OFFSET_W], {OFFSET_W{1'b0}}}; //Align with DATA_W
	 buffer_r1_wdata <= buffer_r0_wdata[0+:(2*DATA_W-8)] << wdata_offset;
	 buffer_r1_wstrb <= buffer_r0_wstrb;
      end
   end

   // update DMA length
   assign dma_w_len = |len_cnt[ADDR_W-1:`AXI_LEN_W] ? {`AXI_LEN_W{1'b1}} : len[`AXI_LEN_W-1:0];
   
   // remaining dma side outputs
   assign dma_w_addr = buffer_r1_addr;
   assign dma_w_wdata = buffer_r1_wdata[2*DATA_W-8-1 -: DATA_W]; //DATA_W most significant bits
   assign dma_w_valid = w_align_valid;
   assign dma_w_wstrb = w_align_wstrb;
   
   // dbus_ready
   assign dbus_ready = buffer_en ? 1'b1 : dma_w_ready;
   
   
endmodule
