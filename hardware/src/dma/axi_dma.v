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

	// control
        input 				  clear,
	input 				  run,

	// Cpu interface (only request)
	input 				  valid,
        input [`DMA_ADDR_W-1:0] 	  addr,
	input [`IO_ADDR_W-1:0] 		  wdata,
	input 				  wstrb,
   
    	// Databus interface {N_IFACE_W, N_IFACE_R}
    	input [N_IFACES-1:0] 		  databus_valid,
    	input [N_IFACES*`IO_ADDR_W-1:0]   databus_addr,
	input [N_IFACES*`MIG_BUS_W-1:0]   databus_wdata,
	input [N_IFACES*`MIG_BUS_W/8-1:0] databus_wstrb,
    	output [N_IFACES*`MIG_BUS_W-1:0]  databus_rdata,
    	output reg [N_IFACES-1:0] 	  databus_ready,

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

   // dma configuration enables
   // read
   reg 					  yread_len_en;
   reg 					  ywrite_read_len_en;
   // write
   reg 					  ywrite_write_nBytesW_en;

   // dma configuration parameters
   // read
   reg [`IO_ADDR_W/2-1:0] 		  yread_len, yread_len_shadow;
   reg [`IO_ADDR_W/2-1:0] 		  ywrite_read_len, ywrite_read_len_shadow;
   // write
   reg [`IO_ADDR_W-1:0] 		  ywrite_write_nBytesW, ywrite_write_nBytesW_shadow, ywrite_write_nBytesW_pip0, ywrite_write_nBytesW_pip1;

   // define number of transactions (DMA)
   reg [N_IFACES_R*`IO_ADDR_W/2-1:0] 		  dma_cnt;
   wire [N_IFACES_R*`AXI_LEN_W-1:0] 		  dma_read_lengths;
   wire [N_IFACES_R*`IO_ADDR_W/2-1:0] 		  read_len, read_len_shadow;
   wire [`AXI_LEN_W-1:0] 			  dma_r_len;		  
   
   
   //Merge master interface
   wire [N_IFACES_R*`REQ_MIG_BUS_W-1:0]  read_m_req;
   wire [N_IFACES_R*`RESP_MIG_BUS_W-1:0] read_m_resp;

   //Merge slave interface
   wire [`REQ_MIG_BUS_W-1:0] 		 read_s_req;
   wire [`RESP_MIG_BUS_W-1:0] 		 read_s_resp;

   // DMA W native interface
   wire 				 dma_w_valid, dma_w_ready;
   wire [`IO_ADDR_W-1:0] 		 dma_w_addr;
   wire [`MIG_BUS_W-1:0] 		 dma_w_wdata;
   wire [`MIG_BUS_W/8-1:0] 		 dma_w_wstrb;
   wire [`AXI_LEN_W-1:0] 		 dma_w_len;

   
   
   // addr decoder enable
   always @* begin
      //read
      yread_len_en = 1'b0;
      ywrite_read_len_en = 1'b0;
      //write
      ywrite_write_nBytesW_en = 1'b0;
      if(valid & wstrb)
	   case(addr)
	     //read
	     `DMA_XYOLO_READ_CONF_LEN : yread_len_en = 1'b1;
	     `DMA_XYOLO_WRITE_READ_CONF_LEN : ywrite_read_len_en = 1'b1;
	     //write
	     `DMA_XYOLO_WRITE_WRITE_CONF_NBYTESW : ywrite_write_nBytesW_en = 1'b1;
	     default : ;
	   endcase // case (addr)
   end // always @ *

   // addr decoder parameters
   always @(posedge clk, posedge clear, posedge rst)
     if(clear || rst) begin
	//read
	yread_len <= {`IO_ADDR_W/2{1'b0}};
	ywrite_read_len <= {`IO_ADDR_W/2{1'b0}};
	//write
	ywrite_write_nBytesW <= {`IO_ADDR_W{1'b0}};
     end else begin
	//read
	if(yread_len_en) yread_len <= wdata[`IO_ADDR_W/2-1:0];
	if(ywrite_read_len_en) ywrite_read_len <= wdata[`IO_ADDR_W/2-1:0];
	//write
	if(ywrite_write_nBytesW_en) ywrite_write_nBytesW <= wdata[`IO_ADDR_W-1:0];
     end // else: !if(clear || rst)

   // configurable parameters shadow register
   always @(posedge clk, posedge rst)
     if(rst) begin
	//read
	yread_len_shadow <= {`IO_ADDR_W/2{1'b0}};
	ywrite_read_len_shadow <= {`IO_ADDR_W/2{1'b0}};
	//write
	ywrite_write_nBytesW_shadow <= {`IO_ADDR_W{1'b0}};
	ywrite_write_nBytesW_pip0 <= {`IO_ADDR_W{1'b0}};
	ywrite_write_nBytesW_pip1 <= {`IO_ADDR_W{1'b0}};
     end else if(run) begin
	//read
	yread_len_shadow <= yread_len;
	ywrite_read_len_shadow <= ywrite_read_len;
	//write
	ywrite_write_nBytesW_pip0 <= ywrite_write_nBytesW;
	ywrite_write_nBytesW_pip1 <= ywrite_write_nBytesW_pip0;
	ywrite_write_nBytesW_shadow <= ywrite_write_nBytesW_pip1;
     end

   // update DMA length - READ
   //concatenate read length configurations
   assign read_len = {ywrite_read_len, yread_len};
   assign read_len_shadow = {ywrite_read_len_shadow, yread_len_shadow};
   
   genvar i;
   generate
      for(i=0;i<N_IFACES_R;i=i+1) begin : dma_r_len_ctr
	 //get len for a single burst
	 assign dma_read_lengths[i*`AXI_LEN_W +: `AXI_LEN_W] = |dma_cnt[(i+1)*`IO_ADDR_W/2-1 -:(`IO_ADDR_W/2 - `AXI_LEN_W)] ? {`AXI_LEN_W{1'b1}} : dma_cnt[i*`IO_ADDR_W/2 +: `AXI_LEN_W];

	 always @(posedge clk, posedge rst)
	   if(rst)
	     dma_cnt[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2] <= {`IO_ADDR_W/2{1'b0}};
	   else if(run)
	     dma_cnt[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2] <= read_len[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2];
	   else if(databus_ready[i]) begin
	      if(dma_cnt[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2] == {`IO_ADDR_W/2{1'b0}})
		dma_cnt[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2] <= read_len_shadow[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2];
	      else
		dma_cnt[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2] <= dma_cnt[i*`IO_ADDR_W/2 +: `IO_ADDR_W/2] - 1;
	   end
      end // block: dma_r_len_ctr
   endgenerate

   //choose read length with most priority
   assign dma_r_len = databus_valid[0] ? dma_read_lengths[0+:`AXI_LEN_W] : dma_read_lengths[1*`AXI_LEN_W +: `AXI_LEN_W];
   
   
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
		    .len      (dma_r_len),
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


   // Align WDATA
   wdata_aligner #(
		   .ADDR_W(`IO_ADDR_W),
		   .DATA_W(`MIG_BUS_W)
		   ) wdata_aligner_inst (
					 .clk(clk),
					 .rst(rst),
					 // control
					 .clear(clear),
					 .run(run),
					 .NBytesW(ywrite_write_nBytesW_shadow),
					 // Databus interface
					 .dbus_valid(databus_valid[N_IFACES_R]),
					 .dbus_addr(databus_addr[N_IFACES_R*`IO_ADDR_W+:`IO_ADDR_W]),
					 .dbus_wdata(databus_wdata[N_IFACES_R*`MIG_BUS_W+:`MIG_BUS_W]),
					 .dbus_wstrb(databus_wstrb[N_IFACES_R*(`MIG_BUS_W/8)+:(`MIG_BUS_W/8)]),
					 .dbus_ready(databus_ready[N_IFACES_R]),
					 //DMA_W interface
					 .dma_w_valid(dma_w_valid),
					 .dma_w_addr(dma_w_addr),
					 .dma_w_wdata(dma_w_wdata),
					 .dma_w_wstrb(dma_w_wstrb),
					 .dma_w_ready(dma_w_ready),
					 //DMA_W len
					 .dma_w_len(dma_w_len)
					 );
   

   
   // AXI_DMA WRITE
   axi_dma_w # (
      .USE_RAM(0)
   ) dma_w (
	    .clk(clk),
	    .rst(rst),
	    // Native interface
	    .valid    (dma_w_valid),
	    .addr     (dma_w_addr[0 +: `DDR_ADDR_W]),
	    .wdata    (dma_w_wdata),
	    .wstrb    (dma_w_wstrb),
	    .ready    (dma_w_ready),
	    // DMA configurations
	    .len        (dma_w_len), //TODO
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
