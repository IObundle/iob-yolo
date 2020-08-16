`timescale 1ns / 1ps

`include "system.vh"
`include "iob_uart.vh"
`include "xversat.vh"

//constants
`define STRINGIFY(x) `"x`"

module dma_w_test_tb;

   //define parameters
   parameter file_name = {"../../../../dma_", `STRINGIFY(`MIG_BUS_W), ".hex"};
   parameter clk_per = 10;

   //clock
   reg clk = 1;
   always #(clk_per/2) clk = ~clk;

   //reset 
   reg reset = 1;
   reg cpu_rst = 1;

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
   wire                    tester_pll_locked;
   wire                    tester_eth_phy_resetn;
   wire                    tester_rx_clk;
   wire [3:0]              tester_rx_data;
   wire                    tester_rx_dv;
   wire                    tester_tx_clk;
   wire [3:0]              tester_tx_data;
   wire                    tester_tx_en;
   wire                    eth_phy_resetn;
   assign tester_pll_locked = 1'b1;
   assign tester_rx_clk = RX_CLK;
   assign tester_tx_clk = TX_CLK;
   
   //iterator
   integer                i;

   /////////////////////////////////////////////
   // RAM signals
   /////////////////////////////////////////////

   wire 				en, we;
   wire [`MEM_ADDR_W-1:0]		addr;
   wire [`MIG_BUS_W-1:0]		data_out;

   /////////////////////////////////////////////
   // ext_addrgen signals
   /////////////////////////////////////////////

   reg					run;
   wire					done;

   //configuration
   reg [`IO_ADDR_W-1:0]           	ext_addr;
   reg [`MEM_ADDR_W-1:0]           	int_addr;
   reg [`EXT_ADDR_W - 1:0]         	iterations;
   reg [`EXT_PERIOD_W - 1:0]       	period;
   reg [`EXT_ADDR_W - 1:0]         	start;
   reg signed [`EXT_ADDR_W - 1:0]  	shift;
   reg signed [`EXT_ADDR_W - 1:0]  	incr;

   //databus interface
   wire                           	databus_ready;
   wire                       		databus_valid;
   wire [`IO_ADDR_W-1:0]          	databus_addr;
   wire [`MIG_BUS_W-1:0]              	databus_wdata;
   wire [`MIG_BUS_W/8-1:0]        	databus_wstrb; 

   /////////////////////////////////////////////
   // TEST PROCEDURE
   /////////////////////////////////////////////
   
   initial begin

`ifdef VCD
      $dumpfile("system.vcd");
      $dumpvars();
`endif

      //init cpu bus signals
      uart_valid = 0;
      uart_wstrb = 0;

      // configurations
      run <= 1'b0;
      ext_addr <= `IO_ADDR_W'b0;
      int_addr <= `MEM_ADDR_W'b0;
      iterations <= `EXT_ADDR_W'b0;
      period <= `EXT_PERIOD_W'b0;
      start <= `EXT_ADDR_W'b0;
      shift <= `EXT_ADDR_W'b0;
      incr <= `EXT_ADDR_W'b0;
      
      // deassert rst
      repeat (100) @(posedge clk);
      reset <= 0;

      //wait an arbitray (10) number of cycles 
      repeat (10) @(posedge clk) #1;

      //configure ext_addrgen to read 16 lines
      ext_addr <= `IO_ADDR_W'h1000;
      iterations <= `EXT_ADDR_W'd1;
      period <= `EXT_PERIOD_W'd16;

      //run and wait for done
      run_conf();
      conf_done();

      //init cpu and check data was written to axi_ram
      cpu_rst <= 0;
      repeat (10) @(posedge clk) #1;
 
      // configure uart
      cpu_inituart();

      //connect with bootloader
      cpu_connect();

`ifdef LD_FW
      //send program
      cpu_sendfile();
`endif
      //run firmware
      cpu_run();
      $finish;
   end

   //
   // DDR AXI interface signals
   //

   //Write address
   wire [0:0] 		   ddr_awid;
   wire [`DDR_ADDR_W-1:0]  ddr_awaddr;
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
   wire [`MIG_BUS_W-1:0]   ddr_rdata;
   wire [1:0]              ddr_rresp;
   wire                    ddr_rlast;
   wire                    ddr_rvalid;
   wire                    ddr_rready;

   //test uart signals
   wire                    tester_txd, tester_rxd;
   wire                    tester_rts, tester_cts;

   //cpu trap signal
   wire                    trap;
 
   //
   // SINGLE PORT RAM
   //

   iob_sp_ram # (
      .FILE(file_name),
      .DATA_W(`MIG_BUS_W),
      .ADDR_W(`MEM_ADDR_W)
   ) ram (
      .clk(clk),
      .en(en),
      .we(we),
      .addr(addr),
      .data_in(`MIG_BUS_W'b0),
      .data_out(data_out)
   );

   //
   // EXT_ADDRGEN
   //

   ext_addrgen # (
      .DATA_W(`MIG_BUS_W),
      .EXT_ADDR_W(`EXT_ADDR_W),
      .EXT_PERIOD_W(`EXT_PERIOD_W),
      .MEM_ADDR_W(`MEM_ADDR_W)
   ) addrgen (
      .clk(clk),
      .rst(reset),
      .run(run),
      .int_cnt_en(1'b1),
      .done(done),
      //configurations
      .ext_addr(ext_addr),
      .int_addr(int_addr),
      .direction(2'b10), //INT2EXT
      .iterations(iterations),
      .period(period),
      .duty(period),
      .delay(`EXT_PERIOD_W'b0),
      .start(start),
      .shift(shift),
      .incr(incr),
      //databus interface
      .databus_ready(databus_ready),
      .databus_valid(databus_valid),
      .databus_addr(databus_addr),
      .databus_rdata(`MIG_BUS_W'b0),
      .databus_wdata(databus_wdata),
      .databus_wstrb(databus_wstrb),
      //mem interface
      .valid(en),
      .we(we),
      .addr(addr),
      .data_out(),
      .data_in(data_out)
   );

   //
   // UNIT UNDER TEST (axi-dma)
   //
   
   axi_dma_w # (
      .USE_RAM(1) //no need to 1-cycle delay on ready signal
   ) dma (
      .clk(clk),
      .rst(reset),
      //databus interface
      .ready(databus_ready),
      .valid(databus_valid),
      .addr(databus_addr[`DDR_ADDR_W-1:0]),
      .wdata(databus_wdata),
      .wstrb(databus_wstrb),
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
      .m_axi_bready  (ddr_bready)
   );

   //use CPU to confirm data written

   system uut (
	       .clk           (clk),
	       .reset         (cpu_rst),
	       .trap          (trap),
`ifdef USE_DDR
               //DDR
               //address write
	       .m_axi_awid    (),
	       .m_axi_awaddr  (),
	       .m_axi_awlen   (),
	       .m_axi_awsize  (),
	       .m_axi_awburst (),
	       .m_axi_awlock  (),
	       .m_axi_awcache (),
	       .m_axi_awprot  (),
	       .m_axi_awqos   (),
	       .m_axi_awvalid (),
	       .m_axi_awready (1'b0),
               
	       //write  
	       .m_axi_wdata   (),
	       .m_axi_wstrb   (),
	       .m_axi_wlast   (),
	       .m_axi_wvalid  (),
	       .m_axi_wready  (1'b0),
               
	       //write response
	       .m_axi_bid     (1'b0),
	       .m_axi_bresp   (2'b0),
	       .m_axi_bvalid  (1'b0),
	       .m_axi_bready  (),
               
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
               .rst       (cpu_rst),
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
   axi_ram 
     #(
       .DATA_WIDTH (`MIG_BUS_W),
       .ADDR_WIDTH (`DDR_ADDR_W-4)
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

    //
    // TASKS
    //

   `include "cpu_tasks.v"

   task run_conf;
      run = 1;
      #clk_per;
      run = 0;
      #clk_per;
   endtask

   task conf_done;
      do begin
	 #clk_per;
      end while(done == 0);
   endtask

endmodule
