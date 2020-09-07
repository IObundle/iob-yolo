`timescale 1ns / 1ps

// top defines
`include "xversat.vh"
`include "interconnect.vh"

// FU defines
`include "xyolo_write.vh"
`include "xyolo_read.vh"
`include "axi_dma.vh"

module xversat # (
	parameter  			ADDR_W = 32,
	parameter			DATABUS_W = 256
    ) (
    	input 			   clk,
    	input 			   rst,

    	// cpu interface
    	input 			   valid,
    	input [ADDR_W-1:0] 	   addr,
    	input 			   wstrb,
    	input [`IO_ADDR_W-1:0] 	   wdata,
    	output 			   ready,
    	output [`IO_ADDR_W-1:0]    rdata,

       // AXI interface 
       // Address write
	output [1-1:0] 		   axi_awid, 
	output [`DDR_ADDR_W-1:0]   axi_awaddr,
	output [8-1:0] 		   axi_awlen,
	output [3-1:0] 		   axi_awsize,
	output [2-1:0] 		   axi_awburst,
	output [1-1:0] 		   axi_awlock,
	output [4-1:0] 		   axi_awcache,
	output [3-1:0] 		   axi_awprot,
	output [4-1:0] 		   axi_awqos,
	output [1-1:0] 		   axi_awvalid,
	input [1-1:0] 		   axi_awready,
       //Write
	output [`MIG_BUS_W-1:0]    axi_wdata,
	output [`MIG_BUS_W/8-1:0]  axi_wstrb,
	output [1-1:0] 		   axi_wlast,
	output [1-1:0] 		   axi_wvalid, 
	input [1-1:0] 		   axi_wready,
       // input [1-1:0]		axi_bid,
	input [2-1:0] 		   axi_bresp,
	input [1-1:0] 		   axi_bvalid,
	output [1-1:0] 		   axi_bready,
       //Address Read
	output [1-1:0] 		   axi_arid,
	output [`DDR_ADDR_W-1:0]   axi_araddr, 
	output [8-1:0] 		   axi_arlen,
	output [3-1:0] 		   axi_arsize,
	output [2-1:0] 		   axi_arburst,
	output [1-1:0] 		   axi_arlock,
	output [4-1:0] 		   axi_arcache,
	output [3-1:0] 		   axi_arprot,
	output [4-1:0] 		   axi_arqos,
	output [1-1:0] 		   axi_arvalid, 
	input [1-1:0] 		   axi_arready,
       //Read
       // input [1-1:0]		axi_rid,
	input [`MIG_BUS_W-1:0] 	   axi_rdata,
	input [2-1:0] 		   axi_rresp,
	input [1-1:0] 		   axi_rlast, 
	input [1-1:0] 		   axi_rvalid, 
	output [1-1:0] 		   axi_rready

    );

   // local parameters
   localparam				N_SLAVES = 3;
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
   wire [`nYOLOvect*`DATAPATH_W-1:0]	flow_bias;
   wire [`nYOLOvect*`nYOLOmacs*`DATAPATH_W-1:0] flow_weight;

   // databus to DMA
   wire [2:0]                   databus_ready, databus_valid;
   wire [3*`MIG_BUS_W-1:0]     	databus_wdata, databus_rdata;
   wire [3*`IO_ADDR_W-1:0]      databus_addr;
   wire [3*`MIG_BUS_W/8-1:0]   	databus_wstrb;
   wire [2*`AXI_LEN_W-1:0] 	dma_len;


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
      .clk (clk),
      .rst (rst),
      //master interface
      .m_req(m_req),
      .m_resp(),
      //slave interface
      .s_req(s_req),
      .s_resp({N_SLAVES*`RESP_W{1'b0}})
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

   //
   // VERSAT DMAs
   //
   axi_dma #(
	     .N_IFACES_R(2),
	     .N_IFACES_W(1)
	     ) dma (
		    .clk(clk),
		    .rst(rst),
		    // control
		    .clear(clear),
		    .run(global_run),
		    // cpu interface (only request)
		    .valid(s_req[`valid(2)]),
		    .addr(s_req[`address(2, `DMA_ADDR_W)]),
		    .wdata(s_req[`wdata(2)]),
		    .wstrb(|s_req[`wstrb(2)]),
		    // Native interface
		    .databus_valid    (databus_valid),
		    .databus_addr     (databus_addr),
		    .databus_wdata    (databus_wdata),
		    .databus_wstrb    (databus_wstrb),
		    .databus_rdata    (databus_rdata),
		    .databus_ready    (databus_ready),
		    // DMA configuration
		    .len      (dma_len),
		    //AXI Interface
		    // Address write
		    .m_axi_awid(axi_awid), 
		    .m_axi_awaddr(axi_awaddr), 
		    .m_axi_awlen(axi_awlen), 
		    .m_axi_awsize(axi_awsize), 
		    .m_axi_awburst(axi_awburst), 
		    .m_axi_awlock(axi_awlock), 
		    .m_axi_awcache(axi_awcache), 
		    .m_axi_awprot(axi_awprot),
		    .m_axi_awqos(axi_awqos), 
		    .m_axi_awvalid(axi_awvalid), 
		    .m_axi_awready(axi_awready),
		    //write
		    .m_axi_wdata(axi_wdata), 
		    .m_axi_wstrb(axi_wstrb), 
		    .m_axi_wlast(axi_wlast), 
		    .m_axi_wvalid(axi_wvalid), 
		    .m_axi_wready(axi_wready), 
		    //write response
		    // .m_axi_bid(axi_bid), 
		    .m_axi_bresp(axi_bresp), 
		    .m_axi_bvalid(axi_bvalid), 
		    .m_axi_bready(axi_bready),
		    //address read
		    .m_axi_arid(axi_arid), 
		    .m_axi_araddr(axi_araddr), 
		    .m_axi_arlen(axi_arlen), 
		    .m_axi_arsize(axi_arsize), 
		    .m_axi_arburst(axi_arburst), 
		    .m_axi_arlock(axi_arlock), 
		    .m_axi_arcache(axi_arcache), 
		    .m_axi_arprot(axi_arprot), 
		    .m_axi_arqos(axi_arqos), 
		    .m_axi_arvalid(axi_arvalid), 
		    .m_axi_arready(axi_arready), 
		    //read 
		    // .m_axi_rid(axi_rid), 
		    .m_axi_rdata(axi_rdata), 
		    .m_axi_rresp(axi_rresp), 
		    .m_axi_rlast(axi_rlast), 
		    .m_axi_rvalid(axi_rvalid),  
		    .m_axi_rready(axi_rready)
		    );

   
endmodule
