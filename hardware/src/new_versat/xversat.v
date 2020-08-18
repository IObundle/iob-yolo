`timescale 1ns / 1ps

// top defines
`include "xversat.vh"
`include "interconnect.vh"

// FU defines
`include "xyolo_write.vh"
`include "xyolo_read.vh"

module xversat # (
	parameter  			ADDR_W = 32,
	parameter			DATABUS_W = 256
    ) (
    	input                         	clk,
    	input                         	rst,

    	// cpu interface
    	input                           valid,
    	input [ADDR_W-1:0]              addr,
    	input                           wstrb,
    	input [`IO_ADDR_W-1:0]        	wdata,
    	output	                        ready,
    	output [`IO_ADDR_W-1:0]         rdata,

    	// databus interface
    	input [2:0]                	databus_ready,
    	input [3*DATABUS_W-1:0]		databus_rdata,
    	output [2:0]           		databus_valid,
    	output [3*`IO_ADDR_W-1:0]   	databus_addr,
    	output [3*DATABUS_W-1:0]  	databus_wdata,
    	output [3*DATABUS_W/8-1:0] 	databus_wstrb,

	// DMA configurations
	output [2*`AXI_LEN_W-1:0]       dma_len
    );

   // local parameters
   localparam				N_SLAVES = 2;
   localparam				DATA_W = `IO_ADDR_W;

   // run_clear address mapping
   wire 				run_clear = addr[ADDR_W-1-$clog2(N_SLAVES)] & valid;
   wire					run = run_clear & ~addr[ADDR_W-2-$clog2(N_SLAVES)];
   wire					clear = run_clear & addr[ADDR_W-2-$clog2(N_SLAVES)];

   // done signals
   wire					read_done, write_done;

   // run signals
   reg                                  run_r0, run_r1;
   wire                                 global_run = run_r0 & ~run_r1; //ensure is high only one clock cycle

   // data flow between FUs
   wire [`nYOLOvect*`DATAPATH_W-1:0]	flow_bias, flow_weight;

   // select DMA len
   wire [`AXI_LEN_W-1:0] 		read_dma_len, write_dma_len;
   assign				dma_len[`AXI_LEN_W-1:0] = databus_valid[0] ? read_dma_len :  write_dma_len;

   // split request interface
   wire [`REQ_W-1:0]    		m_req;
   wire [N_SLAVES*`REQ_W-1:0] 		s_req;

   // concatenate native interface to split master interface
   assign m_req[`valid(0)] = valid & ~run_clear;
   assign m_req[`address(0, ADDR_W)] = addr;
   assign m_req[`wdata(0)] = wdata;
   assign m_req[`wstrb(0)] = wstrb;

   // register versat run
   always @(posedge clk, posedge rst)
      if(rst) begin
         run_r0 <= 1'b0;
         run_r1 <= 1'b0;
      end else begin
         run_r0 <= run;
         run_r1 <= run_r0;
      end

   //versat only returns done signal in cpu interface and is always ready
   assign rdata = {{`IO_ADDR_W-1{1'b0}}, &{read_done, write_done}};
   assign ready = 1'b1;

   // instantiate split
   split # (
      .N_SLAVES(N_SLAVES),
      .DATA_W(DATA_W),
      .ADDR_W(ADDR_W)
   ) split (
      //master interface
      .m_req(m_req),
      .m_resp(),
      //slave interface
      .s_req(s_req),
      .s_resp({2*`RESP_W{1'b0}})
   );

   // instantiate xyolo_read FU
   xyolo_read # (
      .DATAPATH_W(`DATAPATH_W),
      .DATABUS_W(DATABUS_W)
   ) xyolo_read (
      .clk(clk),
      .rst(rst),
      // control
      .clear(clear),
      .run(global_run),
      .done(read_done),
      // cpu interface (only request)
      .valid(s_req[`valid(0)]),
      .addr(s_req[`address(0, `XYOLO_READ_ADDR_W)]),
      .wdata(s_req[`wdata(0)]),
      .wstrb(|s_req[`wstrb(0)]),
      // databus interface
      .databus_ready(databus_ready[0]),
      .databus_valid(databus_valid[0]),
      .databus_addr(databus_addr[`IO_ADDR_W-1:0]),
      .databus_rdata(databus_rdata[DATABUS_W-1:0]),
      .databus_wdata(databus_wdata[DATABUS_W-1:0]),
      .databus_wstrb(databus_wstrb[DATABUS_W/8-1:0]),
      // output data
      .flow_out_bias(flow_bias),
      .flow_out_weight(flow_weight),
      // dma burst
      .dma_len(read_dma_len)
   );

   // instantiate xyolo_write FU
   xyolo_write # (
      .DATAPATH_W(`DATAPATH_W),
      .DATABUS_W(DATABUS_W)
   ) xyolo_write (
      .clk(clk),
      .rst(rst),
      // control
      .clear(clear),
      .run(global_run),
      .done(write_done),
      // cpu interface (only request)
      .valid(s_req[`valid(1)]),
      .addr(s_req[`address(1, `XYOLO_WRITE_ADDR_W)]),
      .wdata(s_req[`wdata(1)]),
      .wstrb(|s_req[`wstrb(1)]),
      // vread databus interface
      .databus_ready(databus_ready[2:1]),
      .databus_valid(databus_valid[2:1]),
      .databus_addr(databus_addr[3*`IO_ADDR_W-1:`IO_ADDR_W]),
      .databus_rdata(databus_rdata[3*DATABUS_W-1:DATABUS_W]),
      .databus_wdata(databus_wdata[3*DATABUS_W-1:DATABUS_W]),
      .databus_wstrb(databus_wstrb[3*DATABUS_W/8-1:DATABUS_W/8]),
      // output data
      .flow_in_bias(flow_bias),
      .flow_in_weight(flow_weight),
      // dma burst
      .dma_len({dma_len[2*`AXI_LEN_W-1:`AXI_LEN_W], write_dma_len})
   );

endmodule
