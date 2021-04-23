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
    parameter ADDR_W=`ADDR_W,
    parameter DATA_W=`DATA_W
    )
   (
    input                                     clk,
    input                                     rst,

`ifdef RUN_DDR_USE_SRAM
    // Instruction bus
    input [1+`FIRM_ADDR_W-2+`WRITE_W-1:0]     i_req,
    output [`RESP_W-1:0]                      i_resp,
`endif

    // Data bus
    input [1+1+`DCACHE_ADDR_W-2+`WRITE_W-1:0] d_req,
    output [`RESP_W-1:0]                      d_resp,

    // AXI interface 
    // Address write
    output [0:0]                              axi_awid, 
    output [`DDR_ADDR_W-1:0]                  axi_awaddr,
    output [7:0]                              axi_awlen,
    output [2:0]                              axi_awsize,
    output [1:0]                              axi_awburst,
    output [0:0]                              axi_awlock,
    output [3:0]                              axi_awcache,
    output [2:0]                              axi_awprot,
    output [3:0]                              axi_awqos,
    output                                    axi_awvalid,
    input                                     axi_awready,
    //Write
    output [`MIG_BUS_W-1:0]                   axi_wdata,
    output [`MIG_BUS_W/8-1:0]                 axi_wstrb,
    output                                    axi_wlast,
    output                                    axi_wvalid, 
    input                                     axi_wready,
    input [1:0]                               axi_bresp,
    input                                     axi_bvalid,
    output                                    axi_bready,
    //Address Read
    output [0:0]                              axi_arid,
    output [`DDR_ADDR_W-1:0]                  axi_araddr, 
    output [7:0]                              axi_arlen,
    output [2:0]                              axi_arsize,
    output [1:0]                              axi_arburst,
    output [0:0]                              axi_arlock,
    output [3:0]                              axi_arcache,
    output [2:0]                              axi_arprot,
    output [3:0]                              axi_arqos,
    output                                    axi_arvalid, 
    input                                     axi_arready,
    //Read
    input [`MIG_BUS_W-1:0]                    axi_rdata,
    input [1:0]                               axi_rresp,
    input                                     axi_rlast, 
    input                                     axi_rvalid, 
    output                                    axi_rready
    );

`ifdef RUN_DDR_USE_SRAM
   //
   // INSTRUCTION CACHE
   //

   // Back-end bus
   wire [1+`DCACHE_ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8)-1:0] icache_be_req;
   wire [`RESP_MIG_BUS_W-1:0]                            icache_be_resp;


   // Instruction cache instance
   iob_cache # 
     (
      .FE_ADDR_W(`FIRM_ADDR_W),
      .BE_ADDR_W(`DCACHE_ADDR_W),
      .N_WAYS(2),        //Number of ways
      .LINE_OFF_W(4),    //Cache Line Offset (number of lines)
      .WORD_OFF_W(4),    //Word Offset (number of words per line)
      .WTBUF_DEPTH_W(5), //FIFO's depth -- 5 minimum for BRAM implementation
      .CTRL_CACHE (0),   //Cache-Control can't be accessed
      .CTRL_CNT(0),      //Remove counters
      .BE_DATA_W(`MIG_BUS_W)
      )
   icache (
           .clk   (clk),
           .reset (rst),

           // Front-end interface
           .valid (i_req[1+`FIRM_ADDR_W-2+`WRITE_W-1]),
           .addr  (i_req[`address(0, `FIRM_ADDR_W-2)]),
           .wdata (i_req[`wdata(0)]),
           .wstrb (i_req[`wstrb(0)]),
           .rdata (i_resp[`rdata(0)]),
           .ready (i_resp[`ready(0)]),
           //Control IO
           .force_inv_in(1'b0),
           .force_inv_out(),
           .wtb_empty_in(1'b1),
           .wtb_empty_out(),
           // Back-end interface
           .mem_valid (icache_be_req[1+`DCACHE_ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8)-1]),
           .mem_addr  (icache_be_req[`address_MIG_BUS(0, `DCACHE_ADDR_W)]),
           .mem_wdata (icache_be_req[`wdata_MIG_BUS(0)]),
           .mem_wstrb (icache_be_req[`wstrb_MIG_BUS(0)]),
           .mem_rdata (icache_be_resp[`rdata_MIG_BUS(0)]),
           .mem_ready (icache_be_resp[`ready_MIG_BUS(0)])
           );
`endif //  `ifdef RUN_DDR_USE_SRAM

   //l2 cache interface signals
   wire [1+`DCACHE_ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8)-1:0] l2cache_req;
   wire [`RESP_MIG_BUS_W-1:0]                            l2cache_resp;
   
   //ext_mem control signals
   wire                                      l2_wtb_empty;
   wire                                      invalidate;
   reg                                       invalidate_reg;
   wire                                      l2_valid = l2cache_req[1+`DCACHE_ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8)-1];
   //Necessary logic to avoid invalidating L2 while it's being accessed by a request
   always @(posedge clk, posedge rst)
     if (rst)
       invalidate_reg <= 1'b0;
     else 
       if (invalidate)
         invalidate_reg <= 1'b1;
       else 
         if(~l2_valid)
           invalidate_reg <= 1'b0;
         else
           invalidate_reg <= invalidate_reg;
   
   //
   // DATA CACHE
   //

   // Back-end bus
   wire [1+`DCACHE_ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8)-1:0]      dcache_be_req;
   wire [`RESP_MIG_BUS_W-1:0]     dcache_be_resp;
   
   // Data cache instance
   iob_cache # 
     (
      .FE_ADDR_W(`DCACHE_ADDR_W),
      .N_WAYS(2),        //Number of ways
      .LINE_OFF_W(4),    //Cache Line Offset (number of lines)
      .WORD_OFF_W(4),    //Word Offset (number of words per line)
      .WTBUF_DEPTH_W(5), //FIFO's depth -- 5 minimum for BRAM implementation
      .CTRL_CACHE (1),   //Either 1 to enable cache-control or 0 to disable
      .CTRL_CNT(1),      //do not change (it's implementation depends on the previous)
      .BE_DATA_W(`MIG_BUS_W)
      )
   dcache (
           .clk   (clk),
           .reset (rst),

           // Front-end interface
           .valid (d_req[2+`DCACHE_ADDR_W-2+`WRITE_W-1]),
           .addr  (d_req[`address(0,1+`DCACHE_ADDR_W-2)]),
           .wdata (d_req[`wdata(0)]),
           .wstrb (d_req[`wstrb(0)]),
           .rdata (d_resp[`rdata(0)]),
           .ready (d_resp[`ready(0)]),
           //Control IO
           .force_inv_in(1'b0),
           .force_inv_out(invalidate),
           .wtb_empty_in(l2_wtb_empty),
           .wtb_empty_out(),
           // Back-end interface
           .mem_valid (dcache_be_req[1+`DCACHE_ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8)-1]),
           .mem_addr  (dcache_be_req[`address_MIG_BUS(0,`DCACHE_ADDR_W)]),
           .mem_wdata (dcache_be_req[`wdata_MIG_BUS(0)]),
           .mem_wstrb (dcache_be_req[`wstrb_MIG_BUS(0)]),
           .mem_rdata (dcache_be_resp[`rdata_MIG_BUS(0)]),
           .mem_ready (dcache_be_resp[`ready_MIG_BUS(0)])
           );

   // Merge cache back-ends
   merge
     #(
       .ADDR_W(`DCACHE_ADDR_W),
`ifdef RUN_DDR_USE_SRAM
       .N_MASTERS(2),
`else
       .N_MASTERS(1),
`endif
      .DATA_W(`MIG_BUS_W)
       )
   merge_i_d_buses_into_l2
     (
      .clk(clk),
      .rst(rst),
      // masters
`ifdef RUN_DDR_USE_SRAM
      .m_req  ({icache_be_req, dcache_be_req}),
      .m_resp ({icache_be_resp, dcache_be_resp}),
`else
      .m_req  (dcache_be_req),
      .m_resp (dcache_be_resp),
`endif                 
      // slave
      .s_req  (l2cache_req),
      .s_resp (l2cache_resp)
      );

   
   // L2 cache instance
   iob_cache_axi # 
     (
      .FE_ADDR_W(`DCACHE_ADDR_W),
      .BE_ADDR_W (`DDR_ADDR_W),
      .N_WAYS(4),        //Number of Ways
      .LINE_OFF_W(4),    //Cache Line Offset (number of lines)
      .WORD_OFF_W(4),    //Word Offset (number of words per line)
      .WTBUF_DEPTH_W(5), //FIFO's depth -- 5 minimum for BRAM implementation
      .CTRL_CACHE (0),   //Cache-Control can't be accessed
      .CTRL_CNT(0),      //Remove counters
    `ifdef RUN_DDR_USE_SRAM
      .FE_DATA_W(`MIG_BUS_W),
    `endif
      .BE_DATA_W(`MIG_BUS_W)
      )
   l2cache (
            .clk   (clk),
            .reset (rst),
      
            // Native interface
`ifdef RUN_DDR_USE_SRAM
            .valid    (l2cache_req[1+`DCACHE_ADDR_W+`MIG_BUS_W+(`MIG_BUS_W/8)-1]),
            .addr     (l2cache_req[`address_MIG_BUS(0, `DCACHE_ADDR_W)-$clog2(`MIG_BUS_W/8)]),
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
            //Control IO
            .force_inv_in(invalidate_reg & ~l2_valid),
            .force_inv_out(),
            .wtb_empty_in(1'b1),
            .wtb_empty_out(l2_wtb_empty),
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
            .axi_rdata(axi_rdata), 
            .axi_rresp(axi_rresp), 
            .axi_rlast(axi_rlast), 
            .axi_rvalid(axi_rvalid),  
            .axi_rready(axi_rready)
            );

endmodule
