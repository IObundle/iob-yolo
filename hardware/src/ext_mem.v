
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

   //Connect versat databus directly to backend
   genvar			  l;
   generate
      for(l = 0; l < 3; l++) begin : ywrite_backend
         assign vcache_be_req[`valid_MIG_BUS(l)] = databus_valid[3-l-1 -: 1];
         assign vcache_be_req[`address_MIG_BUS(l, `DDR_ADDR_W)] = databus_addr[3*`IO_ADDR_W-l*`IO_ADDR_W-1 -: `IO_ADDR_W];
         assign vcache_be_req[`wdata_MIG_BUS(l)] = databus_wdata[3*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W];
         assign vcache_be_req[`wstrb_MIG_BUS(l)] = databus_wstrb[3*`MIG_BUS_W/8-l*`MIG_BUS_W/8-1 -: `MIG_BUS_W/8];
      	 assign databus_rdata[3*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W] = vcache_be_resp[`rdata_MIG_BUS(l)];
   	 assign databus_ready[3-l-1 -: 1] = vcache_be_resp[`ready_MIG_BUS(l)];
   	 assign vcache_be_req[`address_MIG_BUS(l, `ADDR_W)-`DDR_ADDR_W] = 0;
      end
   endgenerate
   
`endif
   
   // Merge caches back-ends
   wire [2*`REQ_MIG_BUS_W-1:0]      l2cache_req;
   wire [2*`RESP_MIG_BUS_W-1:0]     l2cache_resp;
   
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
	     .N_MASTERS(1+2),
 `else
	     .N_MASTERS(1),
 `endif
`endif
	     .DATA_W(`MIG_BUS_W)
	     )
	 merge_i_d_buses_into_l2_read
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
	     .m_req  ({vcache_be_req[`req_MIG_BUS(2)],vcache_be_req[`req_MIG_BUS(1)], dcache_be_req}),
	     .m_resp ({vcache_be_resp[`resp_MIG_BUS(2)],vcache_be_resp[`resp_MIG_BUS(1)], dcache_be_resp}),
 `else
	     .m_req  (dcache_be_req),
	     .m_resp (dcache_be_resp),
 `endif
`endif                 
	     // slave
	     .s_req  (l2cache_req[`req_MIG_BUS(0)]),
	     .s_resp (l2cache_resp[`resp_MIG_BUS(0)])
	     );
	    
   merge
     #(
`ifdef RUN_DDR_USE_SRAM
       .N_MASTERS(2),
`else
 `ifdef USE_NEW_VERSAT
       .N_MASTERS(1),
 `else
       .N_MASTERS(1),
 `endif
`endif
       .DATA_W(`MIG_BUS_W)
       )
   merge_i_d_buses_into_l2_write
     (
      // masters
`ifdef RUN_DDR_USE_SRAM
      .m_req  ({icache_be_req, dcache_be_req}),
      .m_resp ({icache_be_resp, dcache_be_resp}),
`else
 `ifdef USE_NEW_VERSAT
      .m_req  (vcache_be_req[`req_MIG_BUS(0)]),
      .m_resp (vcache_be_resp[`resp_MIG_BUS(0)]),
 `else
      .m_req  (dcache_be_req),
      .m_resp (dcache_be_resp),
 `endif
`endif                 
      // slave
      .s_req  (l2cache_req[`req_MIG_BUS(1)]),
      .s_resp (l2cache_resp[`resp_MIG_BUS(1)])
      );

   // AXI MASTER WRITE WIRES
   // axi address write
   wire [0:0] 			  m_l2_awid, m_dma_awid;
   wire [`DDR_ADDR_W-1:0] 	  m_l2_awaddr, m_dma_awaddr;
   wire [7:0] 			  m_l2_awlen, m_dma_awlen;
   wire [2:0] 			  m_l2_awsize, m_dma_awsize;
   wire [1:0] 			  m_l2_awburst, m_dma_awburst;
   wire [0:0] 			  m_l2_awlock, m_dma_awlock;
   wire [3:0] 			  m_l2_awcache, m_dma_awcache;
   wire [2:0] 			  m_l2_awprot, m_dma_awprot;
   wire [3:0] 			  m_l2_awqos, m_dma_awqos;
   reg 				  m_l2_awuser = 1'b0, m_dma_awuser = 1'b0;
   wire 			  m_l2_awvalid, m_dma_awvalid;
   wire 			  m_l2_awready, m_dma_awready;
   // axi write
   wire [`MIG_BUS_W-1:0] 	  m_l2_wdata, m_dma_wdata;
   wire [`MIG_BUS_W/8-1:0] 	  m_l2_wstrb, m_dma_wstrb;
   wire 			  m_l2_wlast, m_dma_wlast;
   reg 				  m_l2_wuser = 1'b0, m_dma_wuser = 1'b0;
   wire 			  m_l2_wvalid, m_dma_wvalid;
   wire 			  m_l2_wready, m_dma_wready;
   wire [0:0] 			  m_l2_bid, m_dma_bid;
   wire [1:0] 			  m_l2_bresp, m_dma_bresp;
   wire 			  m_l2_bvalid, m_dma_bvalid;
   wire 			  m_l2_bready, m_dma_bready;
   // axi address read
   wire [0:0] 			  m_l2_arid, m_dma_arid;
   wire [`DDR_ADDR_W-1:0] 	  m_l2_araddr, m_dma_araddr;
   wire [7:0] 			  m_l2_arlen, m_dma_arlen;
   wire [2:0] 			  m_l2_arsize, m_dma_arsize;
   wire [1:0] 			  m_l2_arburst, m_dma_arburst;
   wire [0:0] 			  m_l2_arlock, m_dma_arlock;
   wire [3:0] 			  m_l2_arcache, m_dma_arcache;
   wire [2:0] 			  m_l2_arprot, m_dma_arprot;
   wire [3:0] 			  m_l2_arqos, m_dma_arqos;
   reg 				  m_l2_aruser = 1'b0, m_dma_aruser = 1'b0;
   wire 			  m_l2_arvalid, m_dma_arvalid;
   wire 			  m_l2_arready, m_dma_arready;
   // axi read
   wire [0:0] 			  m_l2_rid, m_dma_rid;
   wire [`MIG_BUS_W-1:0] 	  m_l2_rdata, m_dma_rdata;
   wire [1:0] 			  m_l2_rresp, m_dma_rresp;
   wire 			  m_l2_rlast, m_dma_rlast;
   wire 			  m_l2_rvalid, m_dmna_rvalid;
   wire 			  m_l2_rready, m_dma_rready;
   
// AXI SLAVE WIRES
   reg 				  s_axi_xbar_buser = 1'b0;
   reg 				  s_axi_xbar_ruser = 1'b0;
				  
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
   l2cache_read (
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
            .axi_awid(m_l2_awid), 
            .axi_awaddr(m_l2_awaddr), 
            .axi_awlen(m_l2_awlen), 
            .axi_awsize(m_l2_awsize), 
            .axi_awburst(m_l2_awburst), 
            .axi_awlock(m_l2_awlock), 
            .axi_awcache(m_l2_awcache), 
            .axi_awprot(m_l2_awprot),
            .axi_awqos(m_l2_awqos), 
            .axi_awvalid(m_l2_awvalid), 
            .axi_awready(m_l2_awready),
            //write
            .axi_wdata(m_l2_wdata), 
            .axi_wstrb(m_l2_wstrb), 
            .axi_wlast(m_l2_wlast), 
            .axi_wvalid(m_l2_wvalid), 
            .axi_wready(m_l2_wready), 
            //write response
            // .axi_bid(m_l2_bid), 
            .axi_bresp(m_l2_bresp), 
            .axi_bvalid(m_l2_bvalid), 
            .axi_bready(m_l2_bready), 
            //address read
            .axi_arid(m_l2_arid), 
            .axi_araddr(m_l2_araddr), 
            .axi_arlen(m_l2_arlen), 
            .axi_arsize(m_l2_arsize), 
            .axi_arburst(m_l2_arburst), 
            .axi_arlock(m_l2_arlock), 
            .axi_arcache(m_l2_arcache), 
            .axi_arprot(m_l2_arprot), 
            .axi_arqos(m_l2_arqos), 
            .axi_arvalid(m_l2_arvalid), 
            .axi_arready(m_l2_arready), 
            //read 
            //.axi_rid(axi_rid), 
            .axi_rdata(m_l2_rdata), 
            .axi_rresp(m_l2_rresp), 
            .axi_rlast(m_l2_rlast), 
            .axi_rvalid(m_l2_rvalid),  
            .axi_rready(m_l2_rready)
            );

   // L2 cache instance
   iob_cache_axi # 
     (
      .FE_ADDR_W(`DDR_ADDR_W),
      .N_WAYS(1),        //Number of Ways
      .LINE_OFF_W(1),    //Cache Line Offset (number of lines)
      .WORD_OFF_W(1),    //Word Offset (number of words per line)
      .WTBUF_DEPTH_W(4), //FIFO's depth
      .BE_ADDR_W (`DDR_ADDR_W),
      .CTRL_CACHE (0),
      .CTRL_CNT(0),       //Remove counters
      .FE_DATA_W(`MIG_BUS_W),
      .BE_DATA_W(`MIG_BUS_W)
      )
   l2cache_write (
            .clk   (clk),
            .reset (rst),
      
            // Native interface
            .valid    (l2cache_req[`valid_MIG_BUS(1)]),
            .addr     (l2cache_req[`address_MIG_BUS(1,`DDR_ADDR_W+1)-$clog2(`MIG_BUS_W/8)]),
            .wdata    (l2cache_req[`wdata_MIG_BUS(1)]),
   	    .wstrb    (l2cache_req[`wstrb_MIG_BUS(1)]),
            .rdata    (),
            .ready    (l2cache_resp[`ready_MIG_BUS(1)]),

	    
            // AXI interface
            // Address write
            .axi_awid(m_dma_awid), 
            .axi_awaddr(m_dma_awaddr), 
            .axi_awlen(m_dma_awlen), 
            .axi_awsize(m_dma_awsize), 
            .axi_awburst(m_dma_awburst), 
            .axi_awlock(m_dma_awlock), 
            .axi_awcache(m_dma_awcache), 
            .axi_awprot(m_dma_awprot),
            .axi_awqos(m_dma_awqos), 
            .axi_awvalid(m_dma_awvalid), 
            .axi_awready(m_dma_awready), 
            //write
            .axi_wdata(m_dma_wdata), 
            .axi_wstrb(m_dma_wstrb), 
            .axi_wlast(m_dma_wlast), 
            .axi_wvalid(m_dma_wvalid), 
            .axi_wready(m_dma_wready), 
            //write response
            // .axi_bid(m_dma_bid), 
            .axi_bresp(m_dma_bresp), 
            .axi_bvalid(m_dma_bvalid), 
            .axi_bready(m_dma_bready), 
            //address read
            .axi_arid(m_dma_arid), 
            .axi_araddr(m_dma_araddr), 
            .axi_arlen(m_dma_arlen), 
            .axi_arsize(m_dma_arsize), 
            .axi_arburst(m_dma_arburst), 
            .axi_arlock(m_dma_arlock), 
            .axi_arcache(m_dma_arcache), 
            .axi_arprot(m_dma_arprot), 
            .axi_arqos(m_dma_arqos), 
            .axi_arvalid(m_dma_arvalid), 
            .axi_arready(1'b0), //m_dma_arready), 
            //read 
            // .axi_rid(axi_rid), 
            .axi_rdata({`MIG_BUS_W{1'b0}}),//m_dma_ardata), 
            .axi_rresp(2'b0), //m_dma_rresp), 
            .axi_rlast(1'b0), //m_dma_rlast), 
            .axi_rvalid(1'b0), //m_dma_rvalid),  
            .axi_rready(m_dma_rready)
            );

   // AXI INTERCONNECT
   axi_interconnect #(
   		      .S_COUNT(2),
   		      .M_COUNT(1),
   		      .DATA_WIDTH(`MIG_BUS_W),
   		      .ADDR_WIDTH(`DDR_ADDR_W),
   		      .ID_WIDTH(1)
   		      )
   interconnect_inst (
   		      .clk(clk),
   		      .rst(rst),
   		      // slave interface
   		      // address write
   		      .s_axi_awid({m_l2_awid, m_dma_awid}),
   		      .s_axi_awaddr({m_l2_awaddr, m_dma_awaddr}),
   		      .s_axi_awlen({m_l2_awlen, m_dma_awlen}),
   		      .s_axi_awsize({m_l2_awsize, m_dma_awsize}),
   		      .s_axi_awburst({m_l2_awburst, m_dma_awburst}),
   		      .s_axi_awlock({m_l2_awlock, m_dma_awlock}),
   		      .s_axi_awcache({m_l2_awcache, m_dma_awcache}),
   		      .s_axi_awprot({m_l2_awprot, m_dma_awprot}),
   		      .s_axi_awqos({m_l2_awqos, m_dma_awqos}),
   		      .s_axi_awuser({m_l2_awuser, m_dma_awuser}), //input reg = 0
   		      .s_axi_awvalid({m_l2_awvalid, m_dma_awvalid}),
   		      .s_axi_awready({m_l2_awready, m_dma_awready}),
   		      // write
   		      .s_axi_wdata({m_l2_wdata, m_dma_wdata}),
   		      .s_axi_wstrb({m_l2_wstrb, m_dma_wstrb}),
   		      .s_axi_wlast({m_l2_wlast, m_dma_wlast}),
   		      .s_axi_wuser({m_l2_wuser, m_dma_wuser}), //input reg = 0
   		      .s_axi_wvalid({m_l2_wvalid, m_dma_wvalid}),
   		      .s_axi_wready({m_l2_wready, m_dma_wready}),
   		      .s_axi_bid({m_l2_bid, m_dma_bid}),
   		      .s_axi_bresp({m_l2_bresp, m_dma_bresp}),
   		      .s_axi_bvalid({m_l2_bvalid, m_dma_bvalid}),
   		      .s_axi_bready({m_l2_bready, m_dma_bready}),
   		      // address read
   		      .s_axi_arid({m_l2_arid, m_dma_arid}),
   		      .s_axi_araddr({m_l2_araddr, m_dma_araddr}),
   		      .s_axi_arlen({m_l2_arlen, m_dma_arlen}),
   		      .s_axi_arsize({m_l2_arsize, m_dma_arsize}),
   		      .s_axi_arburst({m_l2_arburst, m_dma_arburst}),
   		      .s_axi_arlock({m_l2_arlock, m_dma_arlock}),
   		      .s_axi_arcache({m_l2_arcache, m_dma_arcache}),
   		      .s_axi_arprot({m_l2_arprot, m_dma_arprot}),
   		      .s_axi_arqos({m_l2_arqos, m_dma_arqos}),
   		      .s_axi_aruser({m_l2_aruser, m_dma_aruser}), //input reg = 0
   		      .s_axi_arvalid({m_l2_arvalid, m_dma_arvalid}),
   		      .s_axi_arready({m_l2_arready, m_dma_arready}),

   		      // read
   		      .s_axi_rid({m_l2_rid, m_dma_rid}),
   		      .s_axi_rdata({m_l2_rdata, m_dma_rdata}),
   		      .s_axi_rresp({m_l2_rresp, m_dma_rresp}),
   		      .s_axi_rlast({m_l2_rlast, m_dma_rlast}),
   		      // .s_axi_ruser(),
   		      .s_axi_rvalid({m_l2_rvalid, m_dma_rvalid}),
   		      .s_axi_rready({m_l2_rready, m_dma_rready}),
		      
   		      // master interface
   		      // address write
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
   		      // write
   		      .m_axi_wdata(axi_wdata),
   		      .m_axi_wstrb(axi_wstrb),
   		      .m_axi_wlast(axi_wlast),
   		      .m_axi_wvalid(axi_wvalid),
   		      .m_axi_wready(axi_wready),
   		      .m_axi_bid(axi_bid),
   		      .m_axi_bresp(axi_bresp),
   		      .m_axi_buser(s_axi_xbar_buser), //input reg = 0
   		      .m_axi_bvalid(axi_bvalid),
   		      .m_axi_bready(axi_bready),			 
   		      // address read
   		      .m_axi_arid(axi_arid),
   		      .m_axi_araddr(axi_araddr),
   		      .m_axi_arlen(axi_arlen),
   		      .m_axi_arsize(axi_arsize),
   		      .m_axi_arburst(axi_arburst),
   		      .m_axi_arlock(axi_arlock),
   		      .m_axi_arcache(axi_arcache),
   		      .m_axi_arprot(axi_arprot),
   		      .m_axi_arqos(axi_arqos),
   		      .m_axi_aruser(),
   		      .m_axi_arvalid(axi_arvalid),
   		      .m_axi_arready(axi_arready),
   		      // read
   		      .m_axi_rid(axi_rid),
   		      .m_axi_rdata(axi_rdata),
   		      .m_axi_rresp(axi_rresp),
   		      .m_axi_rlast(axi_rlast),
   		      .m_axi_ruser(s_axi_xbar_ruser), //input reg = 0
   		      .m_axi_rvalid(axi_rvalid),
   		      .m_axi_rready(axi_rready)
   		      );
   
endmodule
