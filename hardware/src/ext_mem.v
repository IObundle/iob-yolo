
`timescale 1 ns / 1 ps

`include "system.vh"
`include "interconnect.vh"
`ifdef USE_NEW_VERSAT
  `include "xversat.vh"
`endif

// REQ/RESP MIG BUS DEFINES
`define REQ_MIG_BUS_W (`VALID_W+ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8))
`define RESP_MIG_BUS_W (`MIG_BUS_W+`READY_W)
`define WDATA_MIG_BUS_P (`MIG_BUS_W/8)
`define ADDR_MIG_BUS_P (`WDATA_MIG_BUS_P + `MIG_BUS_W)
`define valid_MIG_BUS(I) (I+1)*`REQ_MIG_BUS_W-1
`define address_MIG_BUS(I,W) I*`REQ_MIG_BUS_W+`ADDR_MIG_BUS_P+W-1 -: W
`define wdata_MIG_BUS(I) I*`REQ_MIG_BUS_W+`WDATA_MIG_BUS_P +: `MIG_BUS_W
`define wstrb_MIG_BUS(I) I*`REQ_MIG_BUS_W +: (`MIG_BUS_W/8)
`define rdata_MIG_BUS(I) I*`RESP_MIG_BUS_W+`RDATA_P +: `MIG_BUS_W
`define ready_MIG_BUS(I) I*`RESP_MIG_BUS_W

// REQ/RESP VERSAT BUS DEFINES
`ifdef USE_NEW_VERSAT
  `define REQ_VERSAT_W (`VALID_W+`IO_ADDR_W+`DATAPATH_W+(`DATAPATH_W/8))
  `define RESP_VERSAT_W (`DATAPATH_W+`READY_W)
  `define WDATA_VERSAT_P (`DATAPATH_W/8)
  `define ADDR_VERSAT_P (`WDATA_VERSAT_P + `DATAPATH_W)
  `define valid_VERSAT(I) (I+1)*`REQ_VERSAT_W-1
  `define address_VERSAT(I,W) I*`REQ_VERSAT_W+`ADDR_VERSAT_P+W-1 -: W
  `define wdata_VERSAT(I) I*`REQ_VERSAT_W+`WDATA_VERSAT_P +: `DATAPATH_W
  `define wstrb_VERSAT(I) I*`REQ_VERSAT_W +: (`DATAPATH_W/8)
  `define rdata_VERSAT(I) I*`RESP_VERSAT_W+`RDATA_P +: `DATAPATH_W
  `define ready_VERSAT(I) I*`RESP_VERSAT_W
`endif

module ext_mem
  #(
    parameter ADDR_W=32,
    parameter DATA_W=32
    )
  (
   input 			clk,
   input 			rst,

`ifdef RUN_DDR_USE_SRAM
   // Instruction bus
   input [`REQ_W-1:0] 		i_req,
   output [`RESP_W-1:0] 	i_resp,
`endif

   // Data bus
   input [`REQ_W-1:0] 		d_req,
   output [`RESP_W-1:0] 	d_resp,

`ifdef USE_NEW_VERSAT
   //Versat bus
   output 	 		databus_ready,
   output [`DATAPATH_W-1:0]  	databus_rdata,
   input  		   	databus_valid,
   input [`IO_ADDR_W-1:0]    	databus_addr,
   input [`DATAPATH_W-1:0]   	databus_wdata,
   input [`DATAPATH_W/8-1:0] 	databus_wstrb,
   //vwrite versat bus
   output [1:0] 		ywrite_databus_ready,
   output [2*`MIG_BUS_W-1:0]  	ywrite_databus_rdata,
   input [1:0] 		   	ywrite_databus_valid,
   input [2*`IO_ADDR_W-1:0]    	ywrite_databus_addr,
   input [2*`MIG_BUS_W-1:0]   	ywrite_databus_wdata,
   input [2*`MIG_BUS_W/8-1:0] 	ywrite_databus_wstrb,
`endif
   
   // AXI interface 
   // Address write
   output [0:0] 	 	axi_awid, 
   output [`DDR_ADDR_W-1:0] 	axi_awaddr,
   output [7:0] 		axi_awlen,
   output [2:0] 		axi_awsize,
   output [1:0] 		axi_awburst,
   output [0:0] 		axi_awlock,
   output [3:0] 		axi_awcache,
   output [2:0] 		axi_awprot,
   output [3:0] 		axi_awqos,
   output 			axi_awvalid,
   input 			axi_awready,
   //Write
   output [`MIG_BUS_W-1:0] 	axi_wdata,
   output [`MIG_BUS_W/8-1:0] 	axi_wstrb,
   output 			axi_wlast,
   output 			axi_wvalid, 
   input 			axi_wready,
   input [0:0] 			axi_bid,
   input [1:0] 			axi_bresp,
   input 			axi_bvalid,
   output 			axi_bready,
   //Address Read
   output [0:0] 		axi_arid,
   output [`DDR_ADDR_W-1:0] 	axi_araddr, 
   output [7:0] 		axi_arlen,
   output [2:0] 		axi_arsize,
   output [1:0] 		axi_arburst,
   output [0:0] 		axi_arlock,
   output [3:0] 		axi_arcache,
   output [2:0] 		axi_arprot,
   output [3:0] 		axi_arqos,
   output 			axi_arvalid, 
   input 			axi_arready,
   //Read
   input [0:0] 			axi_rid,
   input [`MIG_BUS_W-1:0] 	axi_rdata,
   input [1:0] 			axi_rresp,
   input 			axi_rlast, 
   input 			axi_rvalid, 
   output 			axi_rready
   );

   
`ifdef RUN_DDR_USE_SRAM
   //
   // INSTRUCTION CACHE
   //

   // Front-end bus
   wire [`REQ_W-1:0]      icache_fe_req;
   wire [`RESP_W-1:0]     icache_fe_resp;
   assign icache_fe_req = i_req;
   assign i_resp = icache_fe_resp;

   // Back-end bus
   wire [`REQ_MIG_BUS_W-1:0]      icache_be_req;
   wire [`RESP_MIG_BUS_W-1:0]     icache_be_resp;

   // Instruction cache instance
   iob_cache # (
                .FE_ADDR_W(`DDR_ADDR_W),
                .N_WAYS(2),        //Number of ways
                .LINE_OFF_W(4),    //Cache Line Offset (number of lines)
                .WORD_OFF_W(4),    //Word Offset (number of words per line)
                .WTBUF_DEPTH_W(4), //FIFO's depth
                .CTRL_CACHE (0),
                .CTRL_CNT(0),       //Removes counters - Needed to find a way to access the icache controller (using c-drive ctrl functions will always result in the access of dcache) - Change this to 1 when solution found.
		.BE_DATA_W(`MIG_BUS_W)
                )
   icache (
           .clk   (clk),
           .reset (rst),

           // Front-end interface
           .valid (icache_fe_req[`valid(0)]),
           .addr  (icache_fe_req[`address(0, `DDR_ADDR_W+1)-2]),
           .wdata (icache_fe_req[`wdata(0)]),
           .wstrb (icache_fe_req[`wstrb(0)]),
           .rdata (icache_fe_resp[`rdata(0)]),
           .ready (icache_fe_resp[`ready(0)]),

           // Back-end interface
           .mem_valid (icache_be_req[`valid_MIG_BUS(0)]),
           .mem_addr  (icache_be_req[`address_MIG_BUS(0, `DDR_ADDR_W)]),
           .mem_wdata (icache_be_req[`wdata_MIG_BUS(0)]),
           .mem_wstrb (icache_be_req[`wstrb_MIG_BUS(0)]),
           .mem_rdata (icache_be_resp[`rdata_MIG_BUS(0)]),
           .mem_ready (icache_be_resp[`ready_MIG_BUS(0)])
           );
`endif

   //
   // DATA CACHE
   //

   // Front-end bus
   wire [`REQ_W-1:0]      dcache_fe_req;
   wire [`RESP_W-1:0]     dcache_fe_resp;

   assign dcache_fe_req = d_req;
   assign d_resp = dcache_fe_resp;

   // Back-end bus
   wire [`REQ_MIG_BUS_W-1:0]      dcache_be_req;
   wire [`RESP_MIG_BUS_W-1:0]     dcache_be_resp;

   // Data cache instance
   iob_cache # 
     (
      .FE_ADDR_W(`DDR_ADDR_W),
      .N_WAYS(1),        //Number of ways
      .LINE_OFF_W(4),    //Cache Line Offset (number of lines)
      .WORD_OFF_W(4),    //Word Offset (number of words per line)
      .WTBUF_DEPTH_W(4), //FIFO's depth
      .CTRL_CNT(1),       //Counters for hits and misses (since previous parameter is 0)
      .BE_DATA_W(`MIG_BUS_W)
      )
   dcache (
           .clk   (clk),
           .reset (rst),

           // Front-end interface
           .valid (dcache_fe_req[`valid(0)]),
           .addr  (dcache_fe_req[`address(0,`DDR_ADDR_W+1)-2]),
           .wdata (dcache_fe_req[`wdata(0)]),
           .wstrb (dcache_fe_req[`wstrb(0)]),
           .rdata (dcache_fe_resp[`rdata(0)]),
           .ready (dcache_fe_resp[`ready(0)]),
	   
           // Back-end interface
           .mem_valid (dcache_be_req[`valid_MIG_BUS(0)]),
           .mem_addr  (dcache_be_req[`address_MIG_BUS(0, `DDR_ADDR_W)]),
           .mem_wdata (dcache_be_req[`wdata_MIG_BUS(0)]),
           .mem_wstrb (dcache_be_req[`wstrb_MIG_BUS(0)]),
           .mem_rdata (dcache_be_resp[`rdata_MIG_BUS(0)]),
           .mem_ready (dcache_be_resp[`ready_MIG_BUS(0)])
           );

`ifdef USE_NEW_VERSAT

   //
   // VERSAT L1 Caches
   //

   //Back-end bus
   wire [3*`REQ_MIG_BUS_W-1:0] 	  vcache_be_req;
   wire [3*`RESP_MIG_BUS_W-1:0]	  vcache_be_resp;

   //Connect xyolo_write databus directly to backend
   genvar			  l;
   generate
      for(l = 0; l < 2; l++) begin : ywrite_backend
         assign vcache_be_req[`valid_MIG_BUS(l)] = ywrite_databus_valid[2-l-1 -: 1];
         assign vcache_be_req[`address_MIG_BUS(l, `DDR_ADDR_W)] = ywrite_databus_addr[2*`IO_ADDR_W-l*`IO_ADDR_W-1 -: `IO_ADDR_W];
         assign vcache_be_req[`wdata_MIG_BUS(l)] = ywrite_databus_wdata[2*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W];
         assign vcache_be_req[`wstrb_MIG_BUS(l)] = ywrite_databus_wstrb[2*`MIG_BUS_W/8-l*`MIG_BUS_W/8-1 -: `MIG_BUS_W/8];
      	 assign ywrite_databus_rdata[2*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W] = vcache_be_resp[`rdata_MIG_BUS(l)];
   	 assign ywrite_databus_ready[2-l-1 -: 1] = vcache_be_resp[`ready_MIG_BUS(l)];
   	 assign vcache_be_req[`address_MIG_BUS(l, `ADDR_W)-`DDR_ADDR_W] = 0;
      end
   endgenerate
   
   //Connect xyolo_read vread to L1 cache
   iob_cache # (
      .FE_ADDR_W(`DDR_ADDR_W),
      .N_WAYS(1),        //Number of ways
      .LINE_OFF_W(1),    //Cache Line Offset (number of lines)
      .WORD_OFF_W(5),    //Word Offset (number of words per line)
      .WTBUF_DEPTH_W(4), //FIFO's depth
      .CTRL_CNT(1),       //Counters for hits and misses (since previous parameter is 0)
      .FE_DATA_W(`DATAPATH_W), //DATAPATH_W = 16 front-end
      .BE_DATA_W(`MIG_BUS_W)
   ) vcache (
      .clk   (clk),
      .reset (rst),

      // Front-end interface
      .valid (databus_valid),
      .addr  (databus_addr[`DDR_ADDR_W:1]),
      .wdata (databus_wdata),
      .wstrb (databus_wstrb),
      .rdata (databus_rdata),
      .ready (databus_ready),
		 
      // Back-end interface
      .mem_valid (vcache_be_req[`valid_MIG_BUS((2))]),
      .mem_addr  (vcache_be_req[`address_MIG_BUS((2), `DDR_ADDR_W)]),
      .mem_wdata (vcache_be_req[`wdata_MIG_BUS((2))]),
      .mem_wstrb (vcache_be_req[`wstrb_MIG_BUS((2))]),
      .mem_rdata (vcache_be_resp[`rdata_MIG_BUS((2))]),
      .mem_ready (vcache_be_resp[`ready_MIG_BUS((2))])
  );
  assign vcache_be_req[`address_MIG_BUS(2, `ADDR_W)-`DDR_ADDR_W] = 0;

`endif
   
   // Merge caches back-ends
   wire [`REQ_MIG_BUS_W-1:0]      l2cache_req;
   wire [`RESP_MIG_BUS_W-1:0]     l2cache_resp;

`ifdef RUN_DDR_USE_SRAM
   assign icache_be_req[`address_MIG_BUS(0,`ADDR_W)-`DDR_ADDR_W] = 0;
`endif
   assign dcache_be_req[`address_MIG_BUS(0,`ADDR_W)-`DDR_ADDR_W] = 0;
   
   sync_merge
     #(
`ifdef RUN_DDR_USE_SRAM
     .N_MASTERS(2),
`else
 `ifdef USE_NEW_VERSAT
       .N_MASTERS(1+3),
 `else
       .N_MASTERS(1),
 `endif
`endif
       .DATA_W(`MIG_BUS_W)
       )
     merge_i_d_buses_into_l2
       (
	//inputs
	.clk(clk),
	.rst(rst),
        // masters
`ifdef RUN_DDR_USE_SRAM
        .m_req  ({icache_be_req, dcache_be_req}),
        .m_resp ({icache_be_resp, dcache_be_resp}),
`else
 `ifdef USE_NEW_VERSAT
	.m_req  ({vcache_be_req, dcache_be_req}),
	.m_resp ({vcache_be_resp, dcache_be_resp}),
 `else
        .m_req  (dcache_be_req),
        .m_resp (dcache_be_resp),
 `endif
`endif                 
        // slave
        .s_req  (l2cache_req),
        .s_resp (l2cache_resp)
        );

   // L2 cache instance
   iob_cache_axi # 
     (
      .FE_ADDR_W(`DDR_ADDR_W),
      .N_WAYS(1),        //Number of Ways
      .LINE_OFF_W(4),    //Cache Line Offset (number of lines)
      .WORD_OFF_W(4),    //Word Offset (number of words per line)
      .WTBUF_DEPTH_W(4), //FIFO's depth
      .BE_ADDR_W (`DDR_ADDR_W),
      .CTRL_CACHE (0),
      .CTRL_CNT(0),       //Remove counters
      .FE_DATA_W(`MIG_BUS_W),
      .BE_DATA_W(`MIG_BUS_W)
      )
   l2cache (
            .clk   (clk),
            .reset (rst),
      
            // Native interface
            .valid    (l2cache_req[`valid_MIG_BUS(0)]),
            .addr     (l2cache_req[`address_MIG_BUS(0,`DDR_ADDR_W+1)-$clog2(`MIG_BUS_W/8)]),
            .wdata    (l2cache_req[`wdata_MIG_BUS(0)]),
	    .wstrb    (l2cache_req[`wstrb_MIG_BUS(0)]),
            .rdata    (l2cache_resp[`rdata_MIG_BUS(0)]),
            .ready    (l2cache_resp[`ready_MIG_BUS(0)]),

	    
            // AXI interface
            // Address write
            .axi_awid(axi_awid), 
            .axi_awaddr(axi_awaddr), 
            .axi_awlen(axi_awlen), 
            .axi_awsize(axi_awsize), 
            .axi_awburst(axi_awburst), 
            .axi_awlock(axi_awlock), 
            .axi_awcache(axi_awcache), 
            .axi_awprot(axi_awprot),
            .axi_awqos(axi_awqos), 
            .axi_awvalid(axi_awvalid), 
            .axi_awready(axi_awready), 
            //write
            .axi_wdata(axi_wdata), 
            .axi_wstrb(axi_wstrb), 
            .axi_wlast(axi_wlast), 
            .axi_wvalid(axi_wvalid), 
            .axi_wready(axi_wready), 
            //write response
            //.axi_bid(axi_bid), 
            .axi_bresp(axi_bresp), 
            .axi_bvalid(axi_bvalid), 
            .axi_bready(axi_bready), 
            //address read
            .axi_arid(axi_arid), 
            .axi_araddr(axi_araddr), 
            .axi_arlen(axi_arlen), 
            .axi_arsize(axi_arsize), 
            .axi_arburst(axi_arburst), 
            .axi_arlock(axi_arlock), 
            .axi_arcache(axi_arcache), 
            .axi_arprot(axi_arprot), 
            .axi_arqos(axi_arqos), 
            .axi_arvalid(axi_arvalid), 
            .axi_arready(axi_arready), 
            //read 
            //.axi_rid(axi_rid), 
            .axi_rdata(axi_rdata), 
            .axi_rresp(axi_rresp), 
            .axi_rlast(axi_rlast), 
            .axi_rvalid(axi_rvalid),  
            .axi_rready(axi_rready)
            );

endmodule

