
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
`define req_MIG_BUS(I) I*`REQ_MIG_BUS_W +: `REQ_MIG_BUS_W
`define resp_MIG_BUS(I) I*`RESP_MIG_BUS_W +: `RESP_MIG_BUS_W

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

   // AXI interface 
   // Address write
   output [1-1:0] 	 	axi_awid, 
   output [`DDR_ADDR_W-1:0] 	axi_awaddr,
   output [8-1:0] 		axi_awlen,
   output [3-1:0] 		axi_awsize,
   output [2-1:0] 		axi_awburst,
   output [1-1:0] 		axi_awlock,
   output [4-1:0] 		axi_awcache,
   output [3-1:0] 		axi_awprot,
   output [4-1:0] 		axi_awqos,
   output [1-1:0]		axi_awvalid,
   input [1-1:0]		axi_awready,
   //Write
   output [`MIG_BUS_W-1:0] 	axi_wdata,
   output [`MIG_BUS_W/8-1:0] 	axi_wstrb,
   output [1-1:0]		axi_wlast,
   output [1-1:0]		axi_wvalid, 
   input [1-1:0]		axi_wready,
   // input [1-1:0]		axi_bid,
   input [2-1:0]		axi_bresp,
   input [1-1:0]		axi_bvalid,
   output [1-1:0]		axi_bready,
   //Address Read
   output [1-1:0] 		axi_arid,
   output [`DDR_ADDR_W-1:0] 	axi_araddr, 
   output [8-1:0] 		axi_arlen,
   output [3-1:0] 		axi_arsize,
   output [2-1:0] 		axi_arburst,
   output [1-1:0] 		axi_arlock,
   output [4-1:0] 		axi_arcache,
   output [3-1:0] 		axi_arprot,
   output [4-1:0] 		axi_arqos,
   output [1-1:0]		axi_arvalid, 
   input [1-1:0]		axi_arready,
   //Read
   // input [1-1:0]		axi_rid,
   input [`MIG_BUS_W-1:0] 	axi_rdata,
   input [2-1:0]		axi_rresp,
   input [1-1:0]		axi_rlast, 
   input [1-1:0]		axi_rvalid, 
   output [1-1:0]		axi_rready
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
           .addr  (dcache_fe_req[`address(0,`DDR_ADDR_W)-2]),
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

   // set address offset to zero
   assign icache_be_req[`address_MIG_BUS(0,`ADDR_W)-`DDR_ADDR_W] = 0;
   assign dcache_be_req[`address_MIG_BUS(0,`ADDR_W)-`DDR_ADDR_W] = 0;

   //
   // L2 Cache
   //

   //Merge instruction and data caches
   //L2 buses
   wire [`REQ_MIG_BUS_W-1:0]      l2cache_req;
   wire [`RESP_MIG_BUS_W-1:0]     l2cache_resp;
       
   merge #(
      .N_MASTERS(2),
      .DATA_W(`MIG_BUS_W)
   ) merge_i_d_buses_into_l2_read (
      //inputs
      .clk(clk),
      .rst(rst),
      // masters
      .m_req  ({icache_be_req, dcache_be_req}),
      .m_resp ({icache_be_resp, dcache_be_resp}),
      // slave
      .s_req  (l2cache_req),
      .s_resp (l2cache_resp)
   );
`endif

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
    `ifdef RUN_DDR_USE_SRAM
      .FE_DATA_W(`MIG_BUS_W),
    `endif
      .BE_DATA_W(`MIG_BUS_W)
      )
   l2cache_read (
            .clk   (clk),
            .reset (rst),
      
            // Native interface
	 `ifdef RUN_DDR_USE_SRAM
            .valid    (l2cache_req[`valid_MIG_BUS(0)]),
            .addr     (l2cache_req[`address_MIG_BUS(0,`DDR_ADDR_W)-$clog2(`MIG_BUS_W/8)]),
            .wdata    (l2cache_req[`wdata_MIG_BUS(0)]),
   	    .wstrb    (l2cache_req[`wstrb_MIG_BUS(0)]),
            .rdata    (l2cache_resp[`rdata_MIG_BUS(0)]),
            .ready    (l2cache_resp[`ready_MIG_BUS(0)]),
	 `else
            .valid    (d_req[`valid(0)]),
            .addr     (d_req[`address(0,`DDR_ADDR_W)-$clog2(`ADDR_W/8)]),
            .wdata    (d_req[`wdata(0)]),
   	    .wstrb    (d_req[`wstrb(0)]),
            .rdata    (d_resp[`rdata(0)]),
            .ready    (d_resp[`ready(0)]),
	 `endif
	    
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
            // .axi_bid(axi_bid), 
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
