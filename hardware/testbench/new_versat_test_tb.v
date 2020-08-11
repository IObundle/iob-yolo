`timescale 1ns / 1ps

`include "system.vh"
`include "iob_uart.vh"

//constants
`define STRINGIFY(x) `"x`"
`define OFFSET (2**`FIRM_ADDR_W)
`define NTW_IN_C 3
`define NTW_IN_W 416
`define NTW_IN_KER_SIZE 3
`define NTW_IN_NUM_KER 16
`define WEIGHT_SIZE (`NTW_IN_NUM_KER*(1 + `NTW_IN_KER_SIZE*`NTW_IN_KER_SIZE*`NTW_IN_C)) //16 bits
`define DATA_LAYER_1 ((`NTW_IN_W+2)*(`NTW_IN_W+2)*`NTW_IN_C + 4) //16 bits
`define DATA_LAYER_3 ((`NTW_IN_W/2+2)*(`NTW_IN_W/2+2)*`NTW_IN_NUM_KER)
`define FILE_SIZE ((`OFFSET + 2*(`WEIGHT_SIZE + `DATA_LAYER_1 + 2*`DATA_LAYER_3))/(`MIG_BUS_W/8))


module new_versat_test_tb;

   //clock
   reg clk = 1;
   always #5 clk = ~clk;

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
   reg       uart_valid;
   reg [`UART_ADDR_W-1:0] uart_addr;
   reg [`DATA_W-1:0]      uart_wdata;
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
   parameter file_ddr = {"../../../../new_versat_", `STRINGIFY(`MIG_BUS_W), ".hex"};
   parameter file_size = `FILE_SIZE;
   

   /////////////////////////////////////////////
   // TEST PROCEDURE
   //
   initial begin

`ifdef VCD
      $dumpfile("system.vcd");
      $dumpvars(0, new_versat_test_tb.uut.versat);
      $dumpvars(0, new_versat_test_tb.uut.ext_mem0);
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

   //DDR AXI interface signals
`ifdef USE_DDR
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
