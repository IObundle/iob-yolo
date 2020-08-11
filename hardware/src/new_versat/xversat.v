`timescale 1ns / 1ps

// top defines
`include "xversat.vh"
`include "interconnect.vh"

// FU defines
`include "xyolo_write.vh"
`include "xyolo_read.vh"

module xversat # (
	parameter  			ADDR_W = 32
    ) (
    	input                         	clk,
    	input                         	rst,

    	// cpu interface
    	input                           valid,
    	input [ADDR_W-1:0]              addr,
    	input                           wstrb,
    	input [`IO_ADDR_W-1:0]        	wdata,
    	output reg                      ready,
    	output [`IO_ADDR_W-1:0]         rdata,

    	// vread databus interface
    	input [1:0]                	databus_ready,
    	input [2*`DATAPATH_W-1:0]	databus_rdata,
    	output [1:0]          		databus_valid,
    	output [2*`IO_ADDR_W-1:0]   	databus_addr,
    	output [2*`DATAPATH_W-1:0]  	databus_wdata,
    	output [2*`DATAPATH_W/8-1:0] 	databus_wstrb,

    	// vwrite databus interface
    	input                 		vwrite_databus_ready,
    	input [256-1:0]			vwrite_databus_rdata,
    	output           		vwrite_databus_valid,
    	output [`IO_ADDR_W-1:0]   	vwrite_databus_addr,
    	output [256-1:0]  		vwrite_databus_wdata,
    	output [256/8-1:0] 		vwrite_databus_wstrb
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

   // split request interface
   wire [`REQ_W-1:0]    		m_req;
   wire [N_SLAVES*`REQ_W-1:0] 		s_req;

   //concatenate native interface to split master interface
   assign m_req[`valid(0)] = valid & ~run_clear;
   assign m_req[`address(0, ADDR_W)] = addr;
   assign m_req[`wdata(0)] = wdata;
   assign m_req[`wstrb(0)] = wstrb;

   // cpu interface ready and versat run
   always @(posedge clk, posedge rst)
      if(rst) begin
         ready <= 1'b0;
         run_r0 <= 1'b0;
         run_r1 <= 1'b0;
      end else begin
         ready <= valid;
         run_r0 <= run;
         run_r1 <= run_r0;
      end

   //versat only returns done signal in cpu interface
   assign rdata = {{`IO_ADDR_W-1{1'b0}}, &{read_done, write_done}};

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
      .DATA_W(`DATAPATH_W)
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
      .databus_rdata(databus_rdata[`DATAPATH_W-1:0]),
      .databus_wdata(databus_wdata[`DATAPATH_W-1:0]),
      .databus_wstrb(databus_wstrb[`DATAPATH_W/8-1:0]),
      // output data
      .flow_out_bias(flow_bias),
      .flow_out_weight(flow_weight)
   );

   // instantiate xyolo_write FU
   xyolo_write # (
      .DATA_W(`DATAPATH_W)
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
      .vread_databus_ready(databus_ready[1]),
      .vread_databus_valid(databus_valid[1]),
      .vread_databus_addr(databus_addr[2*`IO_ADDR_W-1:`IO_ADDR_W]),
      .vread_databus_rdata(databus_rdata[2*`DATAPATH_W-1:`DATAPATH_W]),
      .vread_databus_wdata(databus_wdata[2*`DATAPATH_W-1:`DATAPATH_W]),
      .vread_databus_wstrb(databus_wstrb[2*`DATAPATH_W/8-1:`DATAPATH_W/8]),
      // vread databus interface
      .vwrite_databus_ready(vwrite_databus_ready),
      .vwrite_databus_valid(vwrite_databus_valid),
      .vwrite_databus_addr(vwrite_databus_addr),
      .vwrite_databus_rdata(vwrite_databus_rdata),
      .vwrite_databus_wdata(vwrite_databus_wdata),
      .vwrite_databus_wstrb(vwrite_databus_wstrb),
      // output data
      .flow_in_bias(flow_bias),
      .flow_in_weight(flow_weight)
   );

endmodule
