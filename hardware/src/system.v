`timescale 1 ns / 1 ps
`include "system.vh"
`include "iob_uart.vh"
`include "interconnect.vh"
`include "iob_timer.vh"
`include "iob_eth_defs.vh"

module system 
  (
   input 		     clk,
   input 		     reset,
   output 		     trap,

`ifdef USE_DDR //AXI MASTER INTERFACE

   //address write
   output [0:0] 	     m_axi_awid, 
   output [`DDR_ADDR_W-1:0]  m_axi_awaddr,
   output [7:0] 	     m_axi_awlen,
   output [2:0] 	     m_axi_awsize,
   output [1:0] 	     m_axi_awburst,
   output [0:0] 	     m_axi_awlock,
   output [3:0] 	     m_axi_awcache,
   output [2:0] 	     m_axi_awprot,
   output [3:0] 	     m_axi_awqos,
   output 		     m_axi_awvalid,
   input 		     m_axi_awready,

   //write
   output [`MIG_BUS_W-1:0]   m_axi_wdata,
   output [`MIG_BUS_W/8-1:0] m_axi_wstrb,
   output 		     m_axi_wlast,
   output 		     m_axi_wvalid, 
   input 		     m_axi_wready,

   //write response
   input [0:0] 		     m_axi_bid,
   input [1:0] 		     m_axi_bresp,
   input 		     m_axi_bvalid,
   output 		     m_axi_bready,
  
   //address read
   output [0:0] 	     m_axi_arid,
   output [`DDR_ADDR_W-1:0]  m_axi_araddr, 
   output [7:0] 	     m_axi_arlen,
   output [2:0] 	     m_axi_arsize,
   output [1:0] 	     m_axi_arburst,
   output [0:0] 	     m_axi_arlock,
   output [3:0] 	     m_axi_arcache,
   output [2:0] 	     m_axi_arprot,
   output [3:0] 	     m_axi_arqos,
   output 		     m_axi_arvalid, 
   input 		     m_axi_arready,

   //read
   input [0:0] 		     m_axi_rid,
   input [`MIG_BUS_W-1:0]    m_axi_rdata,
   input [1:0] 		     m_axi_rresp,
   input 		     m_axi_rlast, 
   input 		     m_axi_rvalid, 
   output 		     m_axi_rready,
`endif //  `ifdef USE_DDR

   //UART
   output 		     uart_txd,
   input 		     uart_rxd,
   output 		     uart_rts,
   input 		     uart_cts,
   
   //ETHERNET
   output 		     ETH_PHY_RESETN,
   input 		     PLL_LOCKED,

   input 		     RX_CLK,
   input [3:0] 		     RX_DATA,
   input 		     RX_DV,

   input 		     TX_CLK,
   output 		     TX_EN,
   output [3:0] 	     TX_DATA
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
        .axi_bid(m_axi_bid), 
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
        .axi_rid(m_axi_rid), 
        .axi_rdata(m_axi_rdata), 
        .axi_rresp(m_axi_rresp), 
        .axi_rlast(m_axi_rlast), 
        .axi_rvalid(m_axi_rvalid),  
        .axi_rready(m_axi_rready)
        );
`endif

   //
   // UART
   //

   iob_uart uart
     (
      .clk       (clk),
      .rst       (reset),
      
      //cpu interface
      .valid(slaves_req[`valid(`UART)]),
      .address(slaves_req[`address(`UART,`UART_ADDR_W+2)-2]),
      .wdata(slaves_req[`wdata(`UART)]),
      .wstrb(|slaves_req[`wstrb(`UART)]),
      .rdata(slaves_resp[`rdata(`UART)]),
      .ready(slaves_resp[`ready(`UART)]),
      
      
      //RS232 interface
      .txd       (uart_txd),
      .rxd       (uart_rxd),
      .rts       (uart_rts),
      .cts       (uart_cts)
      );


   //
   // TIMER
   //
   iob_timer timer
     (
      .clk      (clk),
      .rst      (reset),

      //cpu interface
      .valid(slaves_req[`valid(`TIMER)]),
      .address(slaves_req[`address(`TIMER,`TIMER_ADDR_W+2)-2]),
      .wdata(slaves_req[`wdata(`TIMER)]),
      .rdata(slaves_resp[`rdata(`TIMER)]),
      .ready(slaves_resp[`ready(`TIMER)])
      );

   //
   // ETHERNET
   //
   iob_eth eth
     (
      .clk      (clk),
      .rst      (reset),

      //cpu interface
      .sel(slaves_req[`valid(`ETHERNET)]),
      .addr(slaves_req[`address(`ETHERNET, `ETH_ADDR_W+2)-2]),
      .data_in(slaves_req[`wdata(`ETHERNET)]),
      .we(|(slaves_req[`wstrb(`ETHERNET)])),
      .data_out(slaves_resp[`rdata(`ETHERNET)]),
      .ready(slaves_resp[`ready(`ETHERNET)]),

      // ethernet interface
      .ETH_PHY_RESETN(ETH_PHY_RESETN),
      .PLL_LOCKED(PLL_LOCKED),

      .RX_CLK(RX_CLK),
      .RX_DATA(RX_DATA),
      .RX_DV(RX_DV),

      .TX_CLK(TX_CLK),
      .TX_DATA(TX_DATA),
      .TX_EN(TX_EN)
      );
   
endmodule
