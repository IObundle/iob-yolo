`timescale 1 ns / 1 ps
`include "system.vh"
`include "interconnect.vh"

//param. list - generated from system/core.mk TODO
`include "export.vh"

//do not remove line below
//PHEADER

`ifdef USE_NEW_VERSAT
`include "xversat.vh"
`endif   

module system 
  #(
    parameter CORE_PARAM1 = `PARAM1_VAL, //Core parameter 1 description
    parameter CORE_PARAM2 = `PARAM2_VAL //Core parameter 2 description
    )
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

//BLOCK Core CPU & RISC-V CPU core that executes the Tiny Yolo V3 program. The CPU controls the remaining modules. The CPU accesses main memory via cache for both data and instructions.    
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
           .N_SLAVES(2),
`else
           .N_SLAVES(1),
`endif
	   .P_SLAVES(`E_BIT)
           )
   ibus_split
     (
      .clk (clk),
      .rst (reset),
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
       .N_SLAVES(3), //E,P,I
`else
       .N_SLAVES(2),
`endif
       .P_SLAVES(`E_BIT)
       )
   dbus_split    
     (
      .clk (clk),
      .rst (reset),
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
      .clk (clk),
      .rst (reset),
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

   //
   // EXTERNAL DDR MEMORY
   //

//BLOCK Cache & Single level cache system controlled by the Core CPU. The cache provides connection between the Core CPU and main memory through an AXI4 interface. 
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
	
        //AXI INTERFACE 
        // Address write
        .axi_awid(m_axi_awid[1*1+:1]), 
        .axi_awaddr(m_axi_awaddr[1*`DDR_ADDR_W+:`DDR_ADDR_W]), 
        .axi_awlen(m_axi_awlen[1*8+:8]), 
        .axi_awsize(m_axi_awsize[1*3+:3]), 
        .axi_awburst(m_axi_awburst[1*2+:2]), 
        .axi_awlock(m_axi_awlock[1*1+:1]), 
        .axi_awcache(m_axi_awcache[1*4+:4]), 
        .axi_awprot(m_axi_awprot[1*3+:3]),
        .axi_awqos(m_axi_awqos[1*4+:4]), 
        .axi_awvalid(m_axi_awvalid[1*1+:1]), 
        .axi_awready(m_axi_awready[1*1+:1]),
        //write
        .axi_wdata(m_axi_wdata[1*`MIG_BUS_W+:`MIG_BUS_W]), 
        .axi_wstrb(m_axi_wstrb[1*`MIG_BUS_W/8+:`MIG_BUS_W/8]), 
        .axi_wlast(m_axi_wlast[1*1+:1]), 
        .axi_wvalid(m_axi_wvalid[1*1+:1]), 
        .axi_wready(m_axi_wready[1*1+:1]), 
        //write response
        // .axi_bid(axi_bid), 
        .axi_bresp(m_axi_bresp[1*2+:2]), 
        .axi_bvalid(m_axi_bvalid[1*1+:1]), 
        .axi_bready(m_axi_bready[1*1+:1]), 
        //address read
        .axi_arid(m_axi_arid[1*1+:1]), 
        .axi_araddr(m_axi_araddr[1*`DDR_ADDR_W+:`DDR_ADDR_W]), 
        .axi_arlen(m_axi_arlen[1*8+:8]), 
        .axi_arsize(m_axi_arsize[1*3+:3]), 
        .axi_arburst(m_axi_arburst[1*2+:2]), 
        .axi_arlock(m_axi_arlock[1*1+:1]), 
        .axi_arcache(m_axi_arcache[1*4+:4]), 
        .axi_arprot(m_axi_arprot[1*3+:3]), 
        .axi_arqos(m_axi_arqos[1*4+:4]), 
        .axi_arvalid(m_axi_arvalid[1*1+:1]), 
        .axi_arready(m_axi_arready[1*1+:1]), 
        //read 
        //.axi_rid(axi_rid), 
        .axi_rdata(m_axi_rdata[1*`MIG_BUS_W+:`MIG_BUS_W]), 
        .axi_rresp(m_axi_rresp[1*2+:2]), 
        .axi_rlast(m_axi_rlast[1*1+:1]), 
        .axi_rvalid(m_axi_rvalid[1*1+:1]),  
        .axi_rready(m_axi_rready[1*1+:1])
        );
`endif

   //
   // Other block descriptions
   //
//BLOCK UART & Serial communication module used to output Tiny Yolo V3 Core runtime messages.

//BLOCK VersatCNN & Runtime configurable accelerator. The VersatCNN is composed of a matrix of computational Functional Units (FUs), which are connected to arrays of input and output buffers. The data transfers between VersatCNN and main memory are mediated by a DMA. The VersatCNN is configured during runtime by the Core CPU. The configurations enable execution of CNN layer dataflows and other Tiny Yolo V3 routines.
   
endmodule
