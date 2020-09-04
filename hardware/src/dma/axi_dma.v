`timescale 1ns / 1ps

`include "axi_dma.vh"
`include "system.vh"
`include "xversat.vh"

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

module axi_dma #(
		 parameter N_IFACES_R = 1,
		 parameter N_IFACES_W = 1,
		 parameter N_IFACES = (N_IFACES_R+N_IFACES_W),
		 parameter ADDR_W=32
		 ) (

	// system inputs
	input 				  clk,
	input 				  rst,
   
    	// Databus interface {N_IFACE_W, N_IFACE_R}
    	input [N_IFACES-1:0] 		  databus_valid,
    	input [N_IFACES*`IO_ADDR_W-1:0]  databus_addr,
	input [N_IFACES*`MIG_BUS_W-1:0]   databus_wdata,
	input [N_IFACES*`MIG_BUS_W/8-1:0] databus_wstrb,
    	output [N_IFACES*`MIG_BUS_W-1:0]  databus_rdata,
    	output reg [N_IFACES-1:0] 	  databus_ready,

        // DMA configuration
	input [2*`AXI_LEN_W-1:0]   len,

	// AXI Interface
	// Master Interface Write Address
	output wire [`AXI_ID_W-1:0] 	  m_axi_awid,
	output wire [`DDR_ADDR_W-1:0] 	  m_axi_awaddr,
	output wire [`AXI_LEN_W-1:0] 	  m_axi_awlen,
	output wire [`AXI_SIZE_W-1:0] 	  m_axi_awsize,
	output wire [`AXI_BURST_W-1:0] 	  m_axi_awburst,
	output wire [`AXI_LOCK_W-1:0] 	  m_axi_awlock,
	output wire [`AXI_CACHE_W-1:0] 	  m_axi_awcache,
	output wire [`AXI_PROT_W-1:0] 	  m_axi_awprot,
	output wire [`AXI_QOS_W-1:0] 	  m_axi_awqos,
	output reg 			  m_axi_awvalid,
	input wire 			  m_axi_awready,

	// Master Interface Write Data
	output wire [`MIG_BUS_W-1:0] 	  m_axi_wdata,
	output reg [`MIG_BUS_W/8-1:0] 	  m_axi_wstrb,
	output reg 			  m_axi_wlast,
	output reg 			  m_axi_wvalid,
	input wire 			  m_axi_wready,

	// Master Interface Write Response
	// input wire [`AXI_ID_W-1:0]     m_axi_bid,
	input wire [`AXI_RESP_W-1:0] 	  m_axi_bresp,
	input wire 			  m_axi_bvalid,
	output reg 			  m_axi_bready,

   
        // Master Interface Read Address
        output wire [`AXI_ID_W-1:0] 	  m_axi_arid,
	output wire [`DDR_ADDR_W-1:0] 	  m_axi_araddr,
	output wire [`AXI_LEN_W-1:0] 	  m_axi_arlen,
	output wire [`AXI_SIZE_W-1:0] 	  m_axi_arsize,
	output wire [`AXI_BURST_W-1:0] 	  m_axi_arburst,
	output wire [`AXI_LOCK_W-1:0] 	  m_axi_arlock,
	output wire [`AXI_CACHE_W-1:0] 	  m_axi_arcache,
	output wire [`AXI_PROT_W-1:0] 	  m_axi_arprot,
	output wire [`AXI_QOS_W-1:0] 	  m_axi_arqos,
	output reg 			  m_axi_arvalid,
	input wire 			  m_axi_arready,

	// Master Interface Read Data
	// input wire [`AXI_ID_W-1:0]     m_axi_rid,
	input wire [`MIG_BUS_W-1:0] 	  m_axi_rdata,
	input wire [`AXI_RESP_W-1:0] 	  m_axi_rresp,
	input wire 			  m_axi_rlast,
	input wire 			  m_axi_rvalid,
	output reg 			  m_axi_rready
	);

   //Merge master interface
   wire [N_IFACES_R*`REQ_MIG_BUS_W-1:0]  read_m_req;
   wire [N_IFACES_R*`RESP_MIG_BUS_W-1:0] read_m_resp;

   //Merge slave interface
   wire [`REQ_MIG_BUS_W-1:0] 		 read_s_req;
   wire [`RESP_MIG_BUS_W-1:0] 		 read_s_resp;

   //Concatenate vread databuses to merge master interface
   genvar			  l;
   generate
      for(l = 0; l < N_IFACES_R; l++) begin : read_merge
         assign read_m_req[`valid_MIG_BUS(l)] = databus_valid[N_IFACES_R-l-1 -: 1];
         assign read_m_req[`address_MIG_BUS(l, `DDR_ADDR_W)] = databus_addr[N_IFACES_R*`IO_ADDR_W-l*`IO_ADDR_W-1 -: `IO_ADDR_W];
         assign read_m_req[`wdata_MIG_BUS(l)] = databus_wdata[N_IFACES_R*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W];
         assign read_m_req[`wstrb_MIG_BUS(l)] = databus_wstrb[N_IFACES_R*`MIG_BUS_W/8-l*`MIG_BUS_W/8-1 -: `MIG_BUS_W/8];
      	 assign databus_rdata[N_IFACES_R*`MIG_BUS_W-l*`MIG_BUS_W-1 -: `MIG_BUS_W] = read_m_resp[`rdata_MIG_BUS(l)];
   	 assign databus_ready[N_IFACES_R-l-1 -: 1] = read_m_resp[`ready_MIG_BUS(l)];
   	 assign read_m_req[`address_MIG_BUS(l, `ADDR_W)-`DDR_ADDR_W] = 0;
      end
   endgenerate

   //Merge dma_reads
   merge #(
      .N_MASTERS(N_IFACES_R),
      .DATA_W(`MIG_BUS_W)
   ) vreads_merge (
      .clk (clk),
      .rst (rst),
      // masters
      .m_req  (read_m_req),
      .m_resp (read_m_resp),
      // slave
      .s_req  (read_s_req),
      .s_resp (read_s_resp)
   );

   // AXI_DMA READ
   axi_dma_r dma_r (
		    .clk(clk),
		    .rst(rst),
		    // Native interface
		    .valid    (read_s_req[`valid_MIG_BUS(0)]),
		    .addr     (read_s_req[`address_MIG_BUS(0,`DDR_ADDR_W)]),
		    .rdata    (read_s_resp[`rdata_MIG_BUS(0)]),
		    .ready    (read_s_resp[`ready_MIG_BUS(0)]),
		    // DMA configuration
		    .len      (len[`AXI_LEN_W-1:0]),

		    //address read
		    .m_axi_arid(m_axi_arid), 
		    .m_axi_araddr(m_axi_araddr), 
		    .m_axi_arlen(m_axi_arlen), 
		    .m_axi_arsize(m_axi_arsize), 
		    .m_axi_arburst(m_axi_arburst), 
		    .m_axi_arlock(m_axi_arlock), 
		    .m_axi_arcache(m_axi_arcache), 
		    .m_axi_arprot(m_axi_arprot), 
		    .m_axi_arqos(m_axi_arqos), 
		    .m_axi_arvalid(m_axi_arvalid), 
		    .m_axi_arready(m_axi_arready), 
		    //read 
		    // .m_axi_rid(m_axi_rid), 
		    .m_axi_rdata(m_axi_rdata), 
		    .m_axi_rresp(m_axi_rresp), 
		    .m_axi_rlast(m_axi_rlast), 
		    .m_axi_rvalid(m_axi_rvalid),  
		    .m_axi_rready(m_axi_rready)
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
	    .len	(len[2*`AXI_LEN_W-1:`AXI_LEN_W]),
	    // Address write
	    .m_axi_awid(m_axi_awid), 
            .m_axi_awaddr(m_axi_awaddr), 
            .m_axi_awlen(m_axi_awlen), 
            .m_axi_awsize(m_axi_awsize), 
            .m_axi_awburst(m_axi_awburst), 
            .m_axi_awlock(m_axi_awlock), 
            .m_axi_awcache(m_axi_awcache), 
            .m_axi_awprot(m_axi_awprot),
            .m_axi_awqos(m_axi_awqos), 
            .m_axi_awvalid(m_axi_awvalid), 
            .m_axi_awready(m_axi_awready),
            //write
            .m_axi_wdata(m_axi_wdata), 
            .m_axi_wstrb(m_axi_wstrb), 
            .m_axi_wlast(m_axi_wlast), 
            .m_axi_wvalid(m_axi_wvalid), 
            .m_axi_wready(m_axi_wready), 
            //write response
            // .m_axi_bid(m_axi_bid), 
            .m_axi_bresp(m_axi_bresp), 
            .m_axi_bvalid(m_axi_bvalid), 
            .m_axi_bready(m_axi_bready)
	    );
   
endmodule
