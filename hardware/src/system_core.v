`timescale 1 ns / 1 ps
`include "system.vh"
`include "interconnect.vh"

//do not remove line below
//PHEADER

`ifdef USE_NEW_VERSAT
`include "xversat.vh"
`endif   

module system 
  (
   //do not remove line below
   //PIO

`ifdef USE_DDR //AXI MASTER INTERFACE
`ifdef USE_NEW_VERSAT

   //address write
   output [2*1-1:0] 	     m_axi_awid, 
   output [2*`DDR_ADDR_W-1:0]  m_axi_awaddr,
   output [2*8-1:0] 	     m_axi_awlen,
   output [2*3-1:0] 	     m_axi_awsize,
   output [2*2-1:0] 	     m_axi_awburst,
   output [2*1-1:0] 	     m_axi_awlock,
   output [2*4-1:0] 	     m_axi_awcache,
   output [2*3-1:0] 	     m_axi_awprot,
   output [2*4-1:0] 	     m_axi_awqos,
   output [2*1-1:0]	     m_axi_awvalid,
   input [2*1-1:0]	     m_axi_awready,

   //write
   output [2*`MIG_BUS_W-1:0]   m_axi_wdata,
   output [2*`MIG_BUS_W/8-1:0] m_axi_wstrb,
   output [2*1-1:0]	     m_axi_wlast,
   output [2*1-1:0]	     m_axi_wvalid, 
   input [2*1-1:0]	     m_axi_wready,

   //write response
   // input [2*1-1:0]	     m_axi_bid,
   input [2*2-1:0]	     m_axi_bresp,
   input [2*1-1:0]	     m_axi_bvalid,
   output [2*1-1:0]	     m_axi_bready,
  
   //address read
   output [2*1-1:0] 	     m_axi_arid,
   output [2*`DDR_ADDR_W-1:0]  m_axi_araddr, 
   output [2*8-1:0] 	     m_axi_arlen,
   output [2*3-1:0] 	     m_axi_arsize,
   output [2*2-1:0] 	     m_axi_arburst,
   output [2*1-1:0] 	     m_axi_arlock,
   output [2*4-1:0] 	     m_axi_arcache,
   output [2*3-1:0] 	     m_axi_arprot,
   output [2*4-1:0] 	     m_axi_arqos,
   output [2*1-1:0]	     m_axi_arvalid, 
   input [2*1-1:0]	     m_axi_arready,

   //read
   // input [2*1-1:0]	     m_axi_rid,
   input [2*`MIG_BUS_W-1:0]  m_axi_rdata,
   input [2*2-1:0]	     m_axi_rresp,
   input [2*1-1:0]	     m_axi_rlast, 
   input [2*1-1:0]	     m_axi_rvalid, 
   output [2*1-1:0]	     m_axi_rready,
`else   
   //address write
   output [1-1:0] 	     m_axi_awid, 
   output [`DDR_ADDR_W-1:0]  m_axi_awaddr,
   output [8-1:0] 	     m_axi_awlen,
   output [3-1:0] 	     m_axi_awsize,
   output [2-1:0] 	     m_axi_awburst,
   output [1-1:0] 	     m_axi_awlock,
   output [4-1:0] 	     m_axi_awcache,
   output [3-1:0] 	     m_axi_awprot,
   output [4-1:0] 	     m_axi_awqos,
   output [1-1:0]	     m_axi_awvalid,
   input [1-1:0]	     m_axi_awready,

   //write
   output [`MIG_BUS_W-1:0]   m_axi_wdata,
   output [`MIG_BUS_W/8-1:0] m_axi_wstrb,
   output [1-1:0]	     m_axi_wlast,
   output [1-1:0]	     m_axi_wvalid, 
   input [1-1:0]	     m_axi_wready,

   //write response
   // input [1-1:0]	     m_axi_bid,
   input [2-1:0]	     m_axi_bresp,
   input [1-1:0]	     m_axi_bvalid,
   output [1-1:0]	     m_axi_bready,
  
   //address read
   output [1-1:0] 	     m_axi_arid,
   output [`DDR_ADDR_W-1:0]  m_axi_araddr, 
   output [8-1:0] 	     m_axi_arlen,
   output [3-1:0] 	     m_axi_arsize,
   output [2-1:0] 	     m_axi_arburst,
   output [1-1:0] 	     m_axi_arlock,
   output [4-1:0] 	     m_axi_arcache,
   output [3-1:0] 	     m_axi_arprot,
   output [4-1:0] 	     m_axi_arqos,
   output [1-1:0]	     m_axi_arvalid, 
   input [1-1:0]	     m_axi_arready,

   //read
   // input [1-1:0]	     m_axi_rid,
   input [`MIG_BUS_W-1:0]    m_axi_rdata,
   input [2-1:0]	     m_axi_rresp,
   input [1-1:0]	     m_axi_rlast, 
   input [1-1:0]	     m_axi_rvalid, 
   output [1-1:0]	     m_axi_rready,
`endif // ifdef USE_NEW_VERSAT
`endif //  `ifdef USE_DDR
   input 		     clk,
   input 		     reset,
   output 		     trap
   );

   localparam ADDR_W=32;
   localparam DATA_W=32;
   
   //
   // SYSTEM RESET
   //

   wire                      boot;
   wire                      boot_reset;   
   wire                      cpu_reset = reset | boot_reset;
   
   //
   //  CPU
   //

   // instruction bus
   wire [`REQ_W-1:0]         cpu_i_req;
   wire [`RESP_W-1:0]        cpu_i_resp;

   // data cat bus
   wire [`REQ_W-1:0]         cpu_d_req;
   wire [`RESP_W-1:0]        cpu_d_resp;
   
   //instantiate the cpu
   iob_picorv32 cpu
       (
        .clk     (clk),
        .rst     (cpu_reset),
        .boot    (boot),
        .trap    (trap),
        
        //instruction bus
        .ibus_req(cpu_i_req),
        .ibus_resp(cpu_i_resp),
        
        //data bus
        .dbus_req(cpu_d_req),
        .dbus_resp(cpu_d_resp)
        );


   //   
   // SPLIT INTERNAL AND EXTERNAL MEMORY BUSES
   //

   //internal memory instruction bus
   wire [`REQ_W-1:0]         int_mem_i_req;
   wire [`RESP_W-1:0]        int_mem_i_resp;
   //external memory instruction bus
`ifdef RUN_DDR_USE_SRAM
   wire [`REQ_W-1:0]         ext_mem_i_req;
   wire [`RESP_W-1:0]        ext_mem_i_resp;
`endif

   // INSTRUCTION BUS
   split #(
`ifdef RUN_DDR_USE_SRAM
           .N_SLAVES(2)
`else
           .N_SLAVES(1)
`endif
           )
   ibus_split
     (
      // master interface
      .m_req  (cpu_i_req),
      .m_resp (cpu_i_resp),
      
      // slaves interface
`ifdef RUN_DDR_USE_SRAM
      .s_req ({ext_mem_i_req, int_mem_i_req}),
      .s_resp ({ext_mem_i_resp, int_mem_i_resp})
`else
      .s_req (int_mem_i_req),
      .s_resp (int_mem_i_resp)
`endif
      );


   // DATA BUS

   //internal memory data bus
   wire [`REQ_W-1:0]         int_mem_d_req;
   wire [`RESP_W-1:0]        int_mem_d_resp;

`ifdef USE_DDR
   //external memory data bus
   wire [`REQ_W-1:0]         ext_mem_d_req;
   wire [`RESP_W-1:0]        ext_mem_d_resp;
`endif

   //peripheral bus
   wire [`REQ_W-1:0]         pbus_req;
   wire [`RESP_W-1:0]        pbus_resp;

   split 
     #(
`ifdef USE_DDR
       .N_SLAVES(3),
       .P_SLAVES(`E_BIT)
`else
       .N_SLAVES(2),
       .P_SLAVES(`P_BIT)
`endif
       )
   dbus_split    
     (
      // master interface
      .m_req  (cpu_d_req),
      .m_resp (cpu_d_resp),

      // slaves interface
`ifdef USE_DDR
      .s_req ({ext_mem_d_req, pbus_req, int_mem_d_req}),
      .s_resp({ext_mem_d_resp, pbus_resp, int_mem_d_resp})
`else
      .s_req ({pbus_req, int_mem_d_req}),
      .s_resp({pbus_resp, int_mem_d_resp})
`endif
      );
   

   //   
   // SPLIT PERIPHERAL BUS
   //

   //slaves bus
   wire [`N_SLAVES*`REQ_W-1:0] slaves_req;
   wire [`N_SLAVES*`RESP_W-1:0] slaves_resp;

   split 
     #(
       .N_SLAVES(`N_SLAVES),
       .P_SLAVES(`P_BIT-1)
       )
   pbus_split
     (
      // master interface
      .m_req(pbus_req),
      .m_resp(pbus_resp),
      
      // slaves interface
      .s_req(slaves_req),
      .s_resp(slaves_resp)
      );

   
   //
   // INTERNAL SRAM MEMORY
   //
   
   int_mem int_mem0 
     (
      .clk                  (clk ),
      .rst                  (reset),
      .boot                 (boot),
      .cpu_reset            (boot_reset),

      // instruction bus
      .i_req                (int_mem_i_req),
      .i_resp               (int_mem_i_resp),

      //data bus
      .d_req                (int_mem_d_req),
      .d_resp               (int_mem_d_resp)
      );

`ifdef USE_DDR

 `ifdef USE_NEW_VERSAT
   // Connect Versat to DMA
   wire [2:0]                   dbus_ready, dbus_valid;
   wire [3*`MIG_BUS_W-1:0]     	dbus_wdata, dbus_rdata;
   wire [3*`IO_ADDR_W-1:0]      dbus_addr;
   wire [3*`MIG_BUS_W/8-1:0]   	dbus_wstrb;
   wire [2*`AXI_LEN_W-1:0] 	dbus_len;
 `endif

   //
   // EXTERNAL DDR MEMORY
   //
   ext_mem 
     ext_mem0 
       (
        .clk                  (clk ),
        .rst                  (reset),

`ifdef RUN_DDR_USE_SRAM
        // instruction bus
        .i_req                (ext_mem_i_req),
        .i_resp               (ext_mem_i_resp),
`endif
        //data bus
        .d_req                (ext_mem_d_req),
        .d_resp               (ext_mem_d_resp),

`ifdef USE_NEW_VERSAT
	//Versat bus
	.databus_valid        (dbus_valid),
	.databus_addr         (dbus_addr),
	.databus_wdata        (dbus_wdata),
	.databus_wstrb        (dbus_wstrb),
	.databus_rdata        (dbus_rdata),
	.databus_ready        (dbus_ready),
	.dma_len	      (dbus_len),
`endif
	
        //AXI INTERFACE 
        //address write
        .axi_awid(m_axi_awid), 
        .axi_awaddr(m_axi_awaddr), 
        .axi_awlen(m_axi_awlen), 
        .axi_awsize(m_axi_awsize), 
        .axi_awburst(m_axi_awburst), 
        .axi_awlock(m_axi_awlock), 
        .axi_awcache(m_axi_awcache), 
        .axi_awprot(m_axi_awprot),
        .axi_awqos(m_axi_awqos), 
        .axi_awvalid(m_axi_awvalid), 
        .axi_awready(m_axi_awready), 
        //write
        .axi_wdata(m_axi_wdata), 
        .axi_wstrb(m_axi_wstrb), 
        .axi_wlast(m_axi_wlast), 
        .axi_wvalid(m_axi_wvalid), 
        .axi_wready(m_axi_wready), 
        //write response
        // .axi_bid(m_axi_bid), 
        .axi_bresp(m_axi_bresp), 
        .axi_bvalid(m_axi_bvalid), 
        .axi_bready(m_axi_bready), 
        //address read
        .axi_arid(m_axi_arid), 
        .axi_araddr(m_axi_araddr), 
        .axi_arlen(m_axi_arlen), 
        .axi_arsize(m_axi_arsize), 
        .axi_arburst(m_axi_arburst), 
        .axi_arlock(m_axi_arlock), 
        .axi_arcache(m_axi_arcache), 
        .axi_arprot(m_axi_arprot), 
        .axi_arqos(m_axi_arqos), 
        .axi_arvalid(m_axi_arvalid), 
        .axi_arready(m_axi_arready), 
        //read 
        // .axi_rid(m_axi_rid), 
        .axi_rdata(m_axi_rdata), 
        .axi_rresp(m_axi_rresp), 
        .axi_rlast(m_axi_rlast), 
        .axi_rvalid(m_axi_rvalid),  
        .axi_rready(m_axi_rready)
        );
`endif

   
endmodule
