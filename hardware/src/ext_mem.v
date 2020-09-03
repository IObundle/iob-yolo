
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

`ifdef USE_NEW_VERSAT
   //Versat bus
   output [2:0] 		databus_ready,
   output [3*`MIG_BUS_W-1:0]  	databus_rdata,
   input [2:0] 		   	databus_valid,
   input [3*`IO_ADDR_W-1:0]    	databus_addr,
   input [3*`MIG_BUS_W-1:0]   	databus_wdata,
   input [3*`MIG_BUS_W/8-1:0] 	databus_wstrb,

   input [2*`AXI_LEN_W-1:0]     dma_len,

   // AXI interface 
   // Address write
   output [2*1-1:0] 	 	axi_awid, 
   output [2*`DDR_ADDR_W-1:0] 	axi_awaddr,
   output [2*8-1:0] 		axi_awlen,
   output [2*3-1:0] 		axi_awsize,
   output [2*2-1:0] 		axi_awburst,
   output [2*1-1:0] 		axi_awlock,
   output [2*4-1:0] 		axi_awcache,
   output [2*3-1:0] 		axi_awprot,
   output [2*4-1:0] 		axi_awqos,
   output [2*1-1:0]		axi_awvalid,
   input [2*1-1:0]		axi_awready,
   //Write
   output [2*`MIG_BUS_W-1:0] 	axi_wdata,
   output [2*`MIG_BUS_W/8-1:0] 	axi_wstrb,
   output [2*1-1:0]		axi_wlast,
   output [2*1-1:0]		axi_wvalid, 
   input [2*1-1:0]		axi_wready,
   // input [2*1-1:0]		axi_bid,
   input [2*2-1:0]		axi_bresp,
   input [2*1-1:0]		axi_bvalid,
   output [2*1-1:0]		axi_bready,
   //Address Read
   output [2*1-1:0] 		axi_arid,
   output [2*`DDR_ADDR_W-1:0] 	axi_araddr, 
   output [2*8-1:0] 		axi_arlen,
   output [2*3-1:0] 		axi_arsize,
   output [2*2-1:0] 		axi_arburst,
   output [2*1-1:0] 		axi_arlock,
   output [2*4-1:0] 		axi_arcache,
   output [2*3-1:0] 		axi_arprot,
   output [2*4-1:0] 		axi_arqos,
   output [2*1-1:0]		axi_arvalid, 
   input [2*1-1:0]		axi_arready,
   //Read
   // input [2*1-1:0]		axi_rid,
   input [2*`MIG_BUS_W-1:0] 	axi_rdata,
   input [2*2-1:0]		axi_rresp,
   input [2*1-1:0]		axi_rlast, 
   input [2*1-1:0]		axi_rvalid, 
   output [2*1-1:0]		axi_rready
   
`else   
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
`endif //ifdef USE_NEW_VERSAT
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
	`ifdef USE_NEW_VERSAT
            // Address write
            .axi_awid(axi_awid[1*1+:1]), 
            .axi_awaddr(axi_awaddr[1*`DDR_ADDR_W+:`DDR_ADDR_W]), 
            .axi_awlen(axi_awlen[1*8+:8]), 
            .axi_awsize(axi_awsize[1*3+:3]), 
            .axi_awburst(axi_awburst[1*2+:2]), 
            .axi_awlock(axi_awlock[1*1+:1]), 
            .axi_awcache(axi_awcache[1*4+:4]), 
            .axi_awprot(axi_awprot[1*3+:3]),
            .axi_awqos(axi_awqos[1*4+:4]), 
            .axi_awvalid(axi_awvalid[1*1+:1]), 
            .axi_awready(axi_awready[1*1+:1]),
            //write
            .axi_wdata(axi_wdata[1*`MIG_BUS_W+:`MIG_BUS_W]), 
            .axi_wstrb(axi_wstrb[1*`MIG_BUS_W/8+:`MIG_BUS_W/8]), 
            .axi_wlast(axi_wlast[1*1+:1]), 
            .axi_wvalid(axi_wvalid[1*1+:1]), 
            .axi_wready(axi_wready[1*1+:1]), 
            //write response
            // .axi_bid(axi_bid), 
            .axi_bresp(axi_bresp[1*2+:2]), 
            .axi_bvalid(axi_bvalid[1*1+:1]), 
            .axi_bready(axi_bready[1*1+:1]), 
            //address read
            .axi_arid(axi_arid[1*1+:1]), 
            .axi_araddr(axi_araddr[1*`DDR_ADDR_W+:`DDR_ADDR_W]), 
            .axi_arlen(axi_arlen[1*8+:8]), 
            .axi_arsize(axi_arsize[1*3+:3]), 
            .axi_arburst(axi_arburst[1*2+:2]), 
            .axi_arlock(axi_arlock[1*1+:1]), 
            .axi_arcache(axi_arcache[1*4+:4]), 
            .axi_arprot(axi_arprot[1*3+:3]), 
            .axi_arqos(axi_arqos[1*4+:4]), 
            .axi_arvalid(axi_arvalid[1*1+:1]), 
            .axi_arready(axi_arready[1*1+:1]), 
            //read 
            //.axi_rid(axi_rid), 
            .axi_rdata(axi_rdata[1*`MIG_BUS_W+:`MIG_BUS_W]), 
            .axi_rresp(axi_rresp[1*2+:2]), 
            .axi_rlast(axi_rlast[1*1+:1]), 
            .axi_rvalid(axi_rvalid[1*1+:1]),  
            .axi_rready(axi_rready[1*1+:1])

	`else
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
	`endif
            );

   //
   // VERSAT DMAs
   //

`ifdef USE_NEW_VERSAT

   //Merge master interface
   wire [2*`REQ_MIG_BUS_W-1:0] 	  vread_m_req;
   wire [2*`RESP_MIG_BUS_W-1:0]	  vread_m_resp;

   //Merge slave interface
   wire [`REQ_MIG_BUS_W-1:0] 	  vread_s_req;
   wire [`RESP_MIG_BUS_W-1:0]	  vread_s_resp;

   //Concatenate vread databuses to merge master interface
   genvar			  l;
   generate
      for(l = 0; l < 2; l++) begin : vread_merge
         assign vread_m_req[`valid_MIG_BUS(l)] = databus_valid[2-l-1 -: 1];
         assign vread_m_req[`address_MIG_BUS(l, `DDR_ADDR_W)] = databus_addr[2*`IO_ADDR_W-l*`IO_ADDR_W-1 -: `IO_ADDR_W];
         assign vread_m_req[`wdata_MIG_BUS(l)] = databus_wdata[2*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W];
         assign vread_m_req[`wstrb_MIG_BUS(l)] = databus_wstrb[2*`MIG_BUS_W/8-l*`MIG_BUS_W/8-1 -: `MIG_BUS_W/8];
      	 assign databus_rdata[2*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W] = vread_m_resp[`rdata_MIG_BUS(l)];
   	 assign databus_ready[2-l-1 -: 1] = vread_m_resp[`ready_MIG_BUS(l)];
   	 assign vread_m_req[`address_MIG_BUS(l, `ADDR_W)-`DDR_ADDR_W] = 0;
      end
   endgenerate

   //Merge vreads
   merge #(
      .N_MASTERS(2),
      .DATA_W(`MIG_BUS_W)
   ) vreads_merge (
      .clk (clk),
      .rst (rst),
      // masters
      .m_req  (vread_m_req),
      .m_resp (vread_m_resp),
      // slave
      .s_req  (vread_s_req),
      .s_resp (vread_s_resp)
   );

   // AXI_DMA READ
   axi_dma_r dma_r (
		    .clk(clk),
		    .rst(rst),
		    // Native interface
		    .valid    (vread_s_req[`valid_MIG_BUS(0)]),
		    .addr     (vread_s_req[`address_MIG_BUS(0,`DDR_ADDR_W)]),
		    .rdata    (vread_s_resp[`rdata_MIG_BUS(0)]),
		    .ready    (vread_s_resp[`ready_MIG_BUS(0)]),
		    // DMA configuration
		    .len      (dma_len[`AXI_LEN_W-1:0]),

		    //address read
		    .m_axi_arid(axi_arid[0*1+:1]), 
		    .m_axi_araddr(axi_araddr[0*`DDR_ADDR_W+:`DDR_ADDR_W]), 
		    .m_axi_arlen(axi_arlen[0*8+:8]), 
		    .m_axi_arsize(axi_arsize[0*3+:3]), 
		    .m_axi_arburst(axi_arburst[0*2+:2]), 
		    .m_axi_arlock(axi_arlock[0*1+:1]), 
		    .m_axi_arcache(axi_arcache[0*4+:4]), 
		    .m_axi_arprot(axi_arprot[0*3+:3]), 
		    .m_axi_arqos(axi_arqos[0*4+:4]), 
		    .m_axi_arvalid(axi_arvalid[0*1+:1]), 
		    .m_axi_arready(axi_arready[0*1+:1]), 
		    //read 
		    // .m_axi_rid(axi_rid[0*1+:1]), 
		    .m_axi_rdata(axi_rdata[0*`MIG_BUS_W+:`MIG_BUS_W]), 
		    .m_axi_rresp(axi_rresp[0*2+:2]), 
		    .m_axi_rlast(axi_rlast[0*1+:1]), 
		    .m_axi_rvalid(axi_rvalid[0*1+:1]),  
		    .m_axi_rready(axi_rready[0*1+:1])
		    );
   
   // AXI_DMA WRITE
   axi_dma_w # (
      .USE_RAM(0)
   ) dma_w (
	    .clk(clk),
	    .rst(rst),
	    // Native interface
	    .valid    (databus_valid[2]),
	    .addr     (databus_addr[3*`IO_ADDR_W-1-(`IO_ADDR_W-`DDR_ADDR_W) -: `DDR_ADDR_W]),
	    .wdata    (databus_wdata[3*`MIG_BUS_W-1 -: `MIG_BUS_W]),
	    .wstrb    (databus_wstrb[3*`MIG_BUS_W/8-1 -: `MIG_BUS_W/8]),
	    .ready    (databus_ready[2]),
	    // DMA configurations
	    .len	(dma_len[2*`AXI_LEN_W-1:`AXI_LEN_W]),
	    // Address write
	    .m_axi_awid(axi_awid[0*1+:1]), 
            .m_axi_awaddr(axi_awaddr[0*`DDR_ADDR_W+:`DDR_ADDR_W]), 
            .m_axi_awlen(axi_awlen[0*8+:8]), 
            .m_axi_awsize(axi_awsize[0*3+:3]), 
            .m_axi_awburst(axi_awburst[0*2+:2]), 
            .m_axi_awlock(axi_awlock[0*1+:1]), 
            .m_axi_awcache(axi_awcache[0*4+:4]), 
            .m_axi_awprot(axi_awprot[0*3+:3]),
            .m_axi_awqos(axi_awqos[0*4+:4]), 
            .m_axi_awvalid(axi_awvalid[0*1+:1]), 
            .m_axi_awready(axi_awready[0*1+:1]),
            //write
            .m_axi_wdata(axi_wdata[0*`MIG_BUS_W+:`MIG_BUS_W]), 
            .m_axi_wstrb(axi_wstrb[0*`MIG_BUS_W/8+:`MIG_BUS_W/8]), 
            .m_axi_wlast(axi_wlast[0*1+:1]), 
            .m_axi_wvalid(axi_wvalid[0*1+:1]), 
            .m_axi_wready(axi_wready[0*1+:1]), 
            //write response
            // .m_axi_bid(axi_bid[0*1+:1]), 
            .m_axi_bresp(axi_bresp[0*2+:2]), 
            .m_axi_bvalid(axi_bvalid[0*1+:1]), 
            .m_axi_bready(axi_bready[0*1+:1])
	    );
`endif

endmodule
