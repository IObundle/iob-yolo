`timescale 1ns / 1ps

`include "system.vh"
`include "iob_uart.vh"
`include "xversat.vh"

//constants
`define STRINGIFY(x) `"x`"
`define OFFSET (2**`FIRM_ADDR_W)
`define NTW_IN_C (3 + (`nYOLOmacs!=1) + (4*(`nYOLOmacs==8)))

`define NTW_IN_W 416
`define NTW_IN_KER_SIZE 3
`define NTW_IN_NUM_KER 16

`define WEIGHT_SIZE (`NTW_IN_NUM_KER*(1 + `NTW_IN_KER_SIZE*`NTW_IN_KER_SIZE*`NTW_IN_C + (5 + ((`nYOLOmacs!=1)*7)) -(4*(`nYOLOmacs==8)) ))
`define DATA_LAYER_1 ((`NTW_IN_W+2)*((`NTW_IN_W+2)*`NTW_IN_C+ (10 + (-2*(`nYOLOmacs!=1)))*(`nYOLOmacs!=8) )) 
`define DATA_LAYER_3 ((`NTW_IN_W/2+2)*(`NTW_IN_W/2+2)*`NTW_IN_NUM_KER)
`define FILE_SIZE ((`OFFSET + 2*(`WEIGHT_SIZE + `DATA_LAYER_1 + 2*`DATA_LAYER_3))/(`MIG_BUS_W/8))

module new_versat_test_tb;

   parameter clk_per = 8;

   //clock
   reg clk = 1;
   always #(clk_per/2) clk = ~clk;

   //reset 
   reg reset = 1;

   //ethernet clocks
   parameter pclk_per = 40;
   reg RX_CLK = 1;
   always #(pclk_per/2) RX_CLK = ~RX_CLK;
   wire TX_CLK;
   assign TX_CLK = RX_CLK;
   
   //received by getchar
   reg [7:0] cpu_char = 0;

   //tester uart
   reg       		  uart_valid;
   reg [`UART_ADDR_W-1:0] uart_addr;
   reg [`UART_WDATA_W-1:0] uart_wdata;
   reg                    uart_wstrb;
   reg [`DATA_W-1:0]      uart_rdata;
   wire                   uart_ready;

   // ethernet interface
   wire 		   tester_pll_locked;
   wire 		   tester_eth_phy_resetn;
   wire 		   tester_rx_clk;
   wire [3:0] 		   tester_rx_data;
   wire 		   tester_rx_dv;
   wire 		   tester_tx_clk;
   wire [3:0] 		   tester_tx_data;
   wire 		   tester_tx_en;
   wire 		   eth_phy_resetn;
   assign tester_pll_locked = 1'b1;
   assign tester_rx_clk = RX_CLK;
   assign tester_tx_clk = TX_CLK;
   
   //iterator
   integer                i;

   //define parameters
   parameter file_ddr = {"../../../../new_versat_", `STRINGIFY(`MIG_BUS_W), "_x", `STRINGIFY(`nYOLOmacs), ".hex"};
   parameter file_size = `FILE_SIZE;
   

   /////////////////////////////////////////////
   // TEST PROCEDURE
   //
   initial begin

`ifdef VCD
      $dumpfile("system.vcd");
      $dumpvars(0, new_versat_test_tb.uut.versat);
      $dumpvars(0, new_versat_test_tb.uut.ext_mem0);
      $dumpvars(0, new_versat_test_tb.interconnect_inst);
      
`endif

      //init cpu bus signals
      uart_valid = 0;
      uart_wstrb = 0;
      
      // deassert rst
      repeat (100) @(posedge clk);
      reset <= 0;

      //wait an arbitray (10) number of cycles 
      repeat (10) @(posedge clk) #1;

      // configure uart
      cpu_inituart();

      //connect with bootloader
      cpu_connect();

`ifdef LD_FW
      //send program
      cpu_sendfile();
      //uncomment for debug
      //cpu_receivefile();
`endif      
      //run firmware
      cpu_run();
      $finish;

   end

   
   //
   // INSTANTIATE COMPONENTS
   //

   //DDR AXI interface signals TODO add wires for SYSTEM - INTERCONNECT
`ifdef USE_DDR
 `ifdef USE_NEW_VERSAT

   //Write address
   wire [2*1-1:0] sys_awid;
   wire [2*`DDR_ADDR_W-1:0] sys_awaddr;
   wire [2*8-1:0] 	  sys_awlen;
   wire [2*3-1:0] 	  sys_awsize;
   wire [2*2-1:0] 	  sys_awburst;
   wire [2*1-1:0] 	  sys_awlock;
   wire [2*4-1:0] 	  sys_awcache;
   wire [2*3-1:0] 	  sys_awprot;
   wire [2*4-1:0] 	  sys_awqos;
   reg [2*1-1:0] 	  sys_awuser_reg = 2'b0; 	  
   wire [2*1-1:0] 	  sys_awvalid;
   wire [2*1-1:0] 	  sys_awready;
   //Write data
   wire [2*`MIG_BUS_W-1:0]   sys_wdata;
   wire [2*`MIG_BUS_W/8-1:0] sys_wstrb;
   wire [2*1-1:0] 	     sys_wlast;
   reg [2*1-1:0] 	     sys_wuser_reg = 2'b0;
   wire [2*1-1:0] 	     sys_wvalid;
   wire [2*1-1:0] 	     sys_wready;
   //Write response
   wire [2*1-1:0] 	     sys_bid;
   wire [2*2-1:0] 	     sys_bresp;
   wire [2*1-1:0] 	     sys_bvalid;
   wire [2*1-1:0] 	     sys_bready;
   //Read address
   wire [2*1-1:0] 	     sys_arid;
   wire [2*`DDR_ADDR_W-1:0]  sys_araddr;
   wire [2*8-1:0] 	     sys_arlen;
   wire [2*3-1:0] 	     sys_arsize;
   wire [2*2-1:0] 	     sys_arburst;
   wire [2*1-1:0] 	     sys_arlock;
   wire [2*4-1:0] 	     sys_arcache;
   wire [2*3-1:0] 	     sys_arprot;
   wire [2*4-1:0] 	     sys_arqos;
   reg [2*1-1:0] 	     sys_aruser_reg = 2'b0;
   wire [2*1-1:0] 	     sys_arvalid;
   wire [2*1-1:0] 	     sys_arready;
   //Read data
   wire [2*1-1:0] 	     sys_rid;
   wire [2*`MIG_BUS_W-1:0]   sys_rdata;
   wire [2*2-1:0] 	     sys_rresp;
   wire [2*1-1:0] 	     sys_rlast;
   wire [2*1-1:0] 	     sys_rvalid;
   wire [2*1-1:0] 	     sys_rready;

 `endif //ifdef USE_NEW_VERSAT

   //Write address
   wire [0:0] ddr_awid;
   wire [`DDR_ADDR_W-1:0] ddr_awaddr;
   wire [7:0]              ddr_awlen;
   wire [2:0]              ddr_awsize;
   wire [1:0]              ddr_awburst;
   wire                    ddr_awlock;
   wire [3:0]              ddr_awcache;
   wire [2:0]              ddr_awprot;
   wire [3:0]              ddr_awqos;
   wire                    ddr_awvalid;
   wire                    ddr_awready;
   //Write data
   wire [`MIG_BUS_W-1:0]   ddr_wdata;
   wire [`MIG_BUS_W/8-1:0] ddr_wstrb;
   wire                    ddr_wlast;
   wire                    ddr_wvalid;
   wire                    ddr_wready;
   //Write response
   wire [7:0]              ddr_bid;
   wire [1:0]              ddr_bresp;
   reg 			   ddr_buser_reg = 1'b0;
   wire                    ddr_bvalid;
   wire                    ddr_bready;
   //Read address
   wire [0:0]              ddr_arid;
   wire [`DDR_ADDR_W-1:0]  ddr_araddr;
   wire [7:0]              ddr_arlen;
   wire [2:0]              ddr_arsize;
   wire [1:0]              ddr_arburst;
   wire                    ddr_arlock;
   wire [3:0]              ddr_arcache;
   wire [2:0]              ddr_arprot;
   wire [3:0]              ddr_arqos;
   wire                    ddr_arvalid;
   wire                    ddr_arready;
   //Read data
   wire [7:0]              ddr_rid;
   wire [`MIG_BUS_W-1:0]             ddr_rdata;
   wire [1:0]              ddr_rresp;
   wire                    ddr_rlast;
   reg 			   ddr_ruser_reg = 1'b0;
   wire                    ddr_rvalid;
   wire                    ddr_rready;
`endif

   //test uart signals
   wire                    tester_txd, tester_rxd;       
   wire                    tester_rts, tester_cts;       

   //cpu trap signal
   wire                    trap;
   
   //
   // UNIT UNDER TEST
   //
   system uut (
	       .clk           (clk),
	       .reset         (reset),
	       .trap          (trap),
`ifdef USE_DDR
 `ifdef USE_NEW_VERSAT
               //DDR
               //address write
	       .m_axi_awid    (sys_awid),
	       .m_axi_awaddr  (sys_awaddr),
	       .m_axi_awlen   (sys_awlen),
	       .m_axi_awsize  (sys_awsize),
	       .m_axi_awburst (sys_awburst),
	       .m_axi_awlock  (sys_awlock),
	       .m_axi_awcache (sys_awcache),
	       .m_axi_awprot  (sys_awprot),
	       .m_axi_awqos   (sys_awqos),
	       .m_axi_awvalid (sys_awvalid),
	       .m_axi_awready (sys_awready),
             
	       //write  
	       .m_axi_wdata   (sys_wdata),
	       .m_axi_wstrb   (sys_wstrb),
	       .m_axi_wlast   (sys_wlast),
	       .m_axi_wvalid  (sys_wvalid),
	       .m_axi_wready  (sys_wready),
               
	       //write response
	       // .m_axi_bid     (sys_bid),
	       .m_axi_bresp   (sys_bresp),
	       .m_axi_bvalid  (sys_bvalid),
	       .m_axi_bready  (sys_bready),
               
	       //address read
	       .m_axi_arid    (sys_arid),
	       .m_axi_araddr  (sys_araddr),
	       .m_axi_arlen   (sys_arlen),
	       .m_axi_arsize  (sys_arsize),
	       .m_axi_arburst (sys_arburst),
	       .m_axi_arlock  (sys_arlock),
	       .m_axi_arcache (sys_arcache),
	       .m_axi_arprot  (sys_arprot),
	       .m_axi_arqos   (sys_arqos),
	       .m_axi_arvalid (sys_arvalid),
	       .m_axi_arready (sys_arready),
               
	       //read   
	       // .m_axi_rid     (sys_rid),
	       .m_axi_rdata   (sys_rdata),
	       .m_axi_rresp   (sys_rresp),
	       .m_axi_rlast   (sys_rlast),
	       .m_axi_rvalid  (sys_rvalid),
	       .m_axi_rready  (sys_rready),	

 `else
               //DDR
               //address write
	       .m_axi_awid    (ddr_awid),
	       .m_axi_awaddr  (ddr_awaddr),
	       .m_axi_awlen   (ddr_awlen),
	       .m_axi_awsize  (ddr_awsize),
	       .m_axi_awburst (ddr_awburst),
	       .m_axi_awlock  (ddr_awlock),
	       .m_axi_awcache (ddr_awcache),
	       .m_axi_awprot  (ddr_awprot),
	       .m_axi_awqos   (ddr_awqos),
	       .m_axi_awvalid (ddr_awvalid),
	       .m_axi_awready (ddr_awready),
               
	       //write  
	       .m_axi_wdata   (ddr_wdata),
	       .m_axi_wstrb   (ddr_wstrb),
	       .m_axi_wlast   (ddr_wlast),
	       .m_axi_wvalid  (ddr_wvalid),
	       .m_axi_wready  (ddr_wready),
               
	       //write response
	       .m_axi_bid     (ddr_bid[0]),
	       .m_axi_bresp   (ddr_bresp),
	       .m_axi_bvalid  (ddr_bvalid),
	       .m_axi_bready  (ddr_bready),
               
	       //address read
	       .m_axi_arid    (ddr_arid),
	       .m_axi_araddr  (ddr_araddr),
	       .m_axi_arlen   (ddr_arlen),
	       .m_axi_arsize  (ddr_arsize),
	       .m_axi_arburst (ddr_arburst),
	       .m_axi_arlock  (ddr_arlock),
	       .m_axi_arcache (ddr_arcache),
	       .m_axi_arprot  (ddr_arprot),
	       .m_axi_arqos   (ddr_arqos),
	       .m_axi_arvalid (ddr_arvalid),
	       .m_axi_arready (ddr_arready),
               
	       //read   
	       .m_axi_rid     (ddr_rid[0]),
	       .m_axi_rdata   (ddr_rdata),
	       .m_axi_rresp   (ddr_rresp),
	       .m_axi_rlast   (ddr_rlast),
	       .m_axi_rvalid  (ddr_rvalid),
	       .m_axi_rready  (ddr_rready),	
 `endif //ifdef USE_NEW_VERSAT
`endif
               
               //UART
	       .uart_txd      (tester_rxd),
	       .uart_rxd      (tester_txd),
	       .uart_rts      (tester_cts),
	       .uart_cts      (tester_rts),

	       //ETHERNET
	       .PLL_LOCKED    (tester_pll_locked),
	       .ETH_PHY_RESETN(eth_phy_resetn),
	       .RX_CLK        (tester_tx_clk),
	       .RX_DATA       (tester_tx_data),
	       .RX_DV         (tester_tx_en),
	       .TX_CLK        (tester_rx_clk),
	       .TX_DATA       (tester_rx_data),
	       .TX_EN         (tester_rx_dv)
	       );

   iob_uart test_uart (
		       .clk       (clk),
		       .rst       (reset),
      
		       .valid     (uart_valid),
		       .address   (uart_addr),
		       .wdata     (uart_wdata),
		       .wstrb     (uart_wstrb),
		       .rdata     (uart_rdata),
		       .ready     (uart_ready),
      
		       .txd       (tester_txd),
		       .rxd       (tester_rxd),
		       .rts       (tester_rts),
		       .cts       (tester_cts)
		       );

`ifdef USE_NEW_VERSAT
   //instantiate 2:1 axi_interconnect TODO - update bus names
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
   		      .rst(reset),
   		      // slave interface
   		      // address write
   		      .s_axi_awid(sys_awid),
   		      .s_axi_awaddr(sys_awaddr),
   		      .s_axi_awlen(sys_awlen),
   		      .s_axi_awsize(sys_awsize),
   		      .s_axi_awburst(sys_awburst),
   		      .s_axi_awlock(sys_awlock),
   		      .s_axi_awcache(sys_awcache),
   		      .s_axi_awprot(sys_awprot),
   		      .s_axi_awqos(sys_awqos),
   		      .s_axi_awuser(sys_awuser_reg), //input reg = 0
   		      .s_axi_awvalid(sys_awvalid),
   		      .s_axi_awready(sys_awready),
   		      // write
   		      .s_axi_wdata(sys_wdata),
   		      .s_axi_wstrb(sys_wstrb),
   		      .s_axi_wlast(sys_wlast),
   		      .s_axi_wuser(sys_wuser_reg), //input reg = 0
   		      .s_axi_wvalid(sys_wvalid),
   		      .s_axi_wready(sys_wready),
   		      .s_axi_bid(sys_bid),
   		      .s_axi_bresp(sys_bresp),
   		      .s_axi_bvalid(sys_bvalid),
   		      .s_axi_bready(sys_bready),
   		      // address read
   		      .s_axi_arid(sys_arid),
   		      .s_axi_araddr(sys_araddr),
   		      .s_axi_arlen(sys_arlen),
   		      .s_axi_arsize(sys_arsize),
   		      .s_axi_arburst(sys_arburst),
   		      .s_axi_arlock(sys_arlock),
   		      .s_axi_arcache(sys_arcache),
   		      .s_axi_arprot(sys_arprot),
   		      .s_axi_arqos(sys_arqos),
   		      .s_axi_aruser(sys_aruser_reg), //input reg = 0
   		      .s_axi_arvalid(sys_arvalid),
   		      .s_axi_arready(sys_arready),
   		      // read
   		      .s_axi_rid(sys_rid[1:0]),
   		      .s_axi_rdata(sys_rdata),
   		      .s_axi_rresp(sys_rresp),
   		      .s_axi_rlast(sys_rlast),
   		      // .s_axi_ruser(),
   		      .s_axi_rvalid(sys_rvalid),
   		      .s_axi_rready(sys_rready),
		      
   		      // master interface
   		      // address write
   		      .m_axi_awid(ddr_awid),
   		      .m_axi_awaddr(ddr_awaddr),
   		      .m_axi_awlen(ddr_awlen),
   		      .m_axi_awsize(ddr_awsize),
   		      .m_axi_awburst(ddr_awburst),
   		      .m_axi_awlock(ddr_awlock),
   		      .m_axi_awcache(ddr_awcache),
   		      .m_axi_awprot(ddr_awprot),
   		      .m_axi_awqos(ddr_awqos),
   		      .m_axi_awvalid(ddr_awvalid),
   		      .m_axi_awready(ddr_awready),
   		      // write
   		      .m_axi_wdata(ddr_wdata),
   		      .m_axi_wstrb(ddr_wstrb),
   		      .m_axi_wlast(ddr_wlast),
   		      .m_axi_wvalid(ddr_wvalid),
   		      .m_axi_wready(ddr_wready),
   		      .m_axi_bid(ddr_bid[0]),
   		      .m_axi_bresp(ddr_bresp),
   		      .m_axi_buser(ddr_buser_reg), //input reg = 0
   		      .m_axi_bvalid(ddr_bvalid),
   		      .m_axi_bready(ddr_bready),			 
   		      // address read
   		      .m_axi_arid(ddr_arid),
   		      .m_axi_araddr(ddr_araddr),
   		      .m_axi_arlen(ddr_arlen),
   		      .m_axi_arsize(ddr_arsize),
   		      .m_axi_arburst(ddr_arburst),
   		      .m_axi_arlock(ddr_arlock),
   		      .m_axi_arcache(ddr_arcache),
   		      .m_axi_arprot(ddr_arprot),
   		      .m_axi_arqos(ddr_arqos),
   		      .m_axi_aruser(),
   		      .m_axi_arvalid(ddr_arvalid),
   		      .m_axi_arready(ddr_arready),
   		      // read
   		      .m_axi_rid(ddr_rid[0]),
   		      .m_axi_rdata(ddr_rdata),
   		      .m_axi_rresp(ddr_rresp),
   		      .m_axi_rlast(ddr_rlast),
   		      .m_axi_ruser(ddr_ruser_reg), //input reg = 0
   		      .m_axi_rvalid(ddr_rvalid),
   		      .m_axi_rready(ddr_rready)
   		      );
`endif // ifdef USE_NEW_VERSAT

   
   //instantiate the axi memory
`ifdef USE_DDR
   axi_ram 
     #(
 `ifdef DDR_INIT
       .FILE("firmware.hex"),
 `else
       .FILE(file_ddr),
 `endif
       .FILE_SIZE(file_size),
       .DATA_WIDTH (`MIG_BUS_W),
       .ADDR_WIDTH (`DDR_ADDR_W-4) //Simulation with full sized DDR takes some time to start
       )
   ddr_model_mem(
                 //address write
                 .clk            (clk),
                 .rst            (reset),
		 .s_axi_awid     ({8{ddr_awid}}),
		 .s_axi_awaddr   (ddr_awaddr[`DDR_ADDR_W-4-1:0]),
                 .s_axi_awlen    (ddr_awlen),
                 .s_axi_awsize   (ddr_awsize),
                 .s_axi_awburst  (ddr_awburst),
                 .s_axi_awlock   (ddr_awlock),
		 .s_axi_awprot   (ddr_awprot),
		 .s_axi_awcache  (ddr_awcache),
     		 .s_axi_awvalid  (ddr_awvalid),
		 .s_axi_awready  (ddr_awready),
      
		 //write  
		 .s_axi_wvalid   (ddr_wvalid),
		 .s_axi_wready   (ddr_wready),
		 .s_axi_wdata    (ddr_wdata),
		 .s_axi_wstrb    (ddr_wstrb),
                 .s_axi_wlast    (ddr_wlast),
      
		 //write response
		 .s_axi_bready   (ddr_bready),
                 .s_axi_bid      (ddr_bid),
                 .s_axi_bresp    (ddr_bresp),
		 .s_axi_bvalid   (ddr_bvalid),
      
		 //address read
		 .s_axi_arid     ({8{ddr_arid}}),
		 .s_axi_araddr   (ddr_araddr[`DDR_ADDR_W-4-1:0]),
		 .s_axi_arlen    (ddr_arlen), 
		 .s_axi_arsize   (ddr_arsize),    
                 .s_axi_arburst  (ddr_arburst),
                 .s_axi_arlock   (ddr_arlock),
                 .s_axi_arcache  (ddr_arcache),
                 .s_axi_arprot   (ddr_arprot),
		 .s_axi_arvalid  (ddr_arvalid),
		 .s_axi_arready  (ddr_arready),
      
		 //read   
		 .s_axi_rready   (ddr_rready),
		 .s_axi_rid      (ddr_rid),
		 .s_axi_rdata    (ddr_rdata),
		 .s_axi_rresp    (ddr_rresp),
                 .s_axi_rlast    (ddr_rlast),
		 .s_axi_rvalid   (ddr_rvalid)
                 );   
`endif

`include "cpu_tasks.v"
   
   //finish simulation
   //always @(posedge trap) begin
   // #10 $display("Found CPU trap condition");
   //$finish;
   //end
   
endmodule
