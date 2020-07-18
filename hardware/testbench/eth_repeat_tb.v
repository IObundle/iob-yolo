`timescale 1ns / 1ps

`include "system.vh"
`include "iob_uart.vh"
`include "iob_eth_defs.vh"

`define ETH_NBYTES (256-18)

module eth_repeat_tb;

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

   reg 	     start_flag = 0;
   
   
   //tester uart
   reg       uart_valid;
   reg [`UART_ADDR_W-1:0] uart_addr;
   reg [`DATA_W-1:0]      uart_wdata;
   reg                    uart_wstrb;
   reg [`DATA_W-1:0]      uart_rdata;
   wire                   uart_ready;

   //test uart signals
   wire                    tester_txd, tester_rxd;       
   wire                    tester_rts, tester_cts;       

   //test ethernet signals
   // cpu interface
   reg 			   eth_sel, eth_we, eth_ready;
   reg [`ETH_ADDR_W-1:0]   eth_addr;
   reg [31:0] 		   eth_data_in, eth_data_out;

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
   
   
   //cpu trap signal
   wire                    trap;

   
   //iterator
   integer                i;

   /////////////////////////////////////////////
   // TEST PROCEDURE - SIMULATE CONSOLE
   //
   initial begin

`ifdef VCD
      $dumpfile("system.vcd");
      $dumpvars();
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
      start_flag = 1;
      cpu_run();
      repeat (10000) @(posedge clk)#1;
           
      $finish;

   end

   //
   // SIMULATE ETH_COMM
   //

   //ethernet aux signals
   reg [7:0] data[`ETH_NBYTES+30-1:0];
   reg [47:0] mac_addr = `ETH_RMAC_ADDR;
   reg [`ETH_NBYTES*8-1:0] data_tmp;
    
   
   initial begin

      //init signals
      eth_sel = 0;
      eth_we = 0;

      //wait for reset
      repeat (150)@(posedge clk) #1;

      //init ethernet module
      do
	repeat (200) @(posedge clk) #1;
      while(start_flag==0);
      cpu_ethinit();
      cpu_ethset_rx_payload_size();

      //send frame
      repeat (30000) @(posedge clk) #1;
      data_tmp = {`ETH_NBYTES*8{1'b0}};
      data_tmp[`ETH_NBYTES*8-1 -: 14*8] = "Hello from PC!";
      for(i=0; i<`ETH_NBYTES; i=i+1) data[i+30] = data_tmp[`ETH_NBYTES*8-i*8-1 -: 8];
      cpu_ethsend_frame();

      //wait to receive frame
      cpu_ethrcv_frame(1);
      data_tmp = {`ETH_NBYTES*8{1'b0}};
      for(i=0;i<`ETH_NBYTES; i = i+1) data_tmp[`ETH_NBYTES*8-i*8-1 -: 8] = data[i+30];
      $display("Data received: %s", data_tmp);

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

   //
   // TESTER ETHERNET
   //
   
   iob_eth #(
	     .ETH_MAC_ADDR(`ETH_RMAC_ADDR)
	    )
   test_eth (
	     .clk             (clk), //in
	     .rst             (reset), //in

	     //cpu interface
	     .sel             (eth_sel), //in
	     .addr            (eth_addr), //in
	     .data_in         (eth_data_in), //in
	     .we              (eth_we), //in
	     .data_out        (eth_data_out),
	     .ready           (eth_ready),

	     // ethernet interface
	     .ETH_PHY_RESETN  (tester_eth_phy_resetn),
	     .PLL_LOCKED      (tester_pll_locked), //in

	     .RX_CLK          (tester_rx_clk), //in
	     .RX_DATA         (tester_rx_data), //in
	     .RX_DV           (tester_rx_dv), //in

	     .TX_CLK          (tester_tx_clk), //in
	     .TX_DATA         (tester_tx_data),
	     .TX_EN           (tester_tx_en)
	     );
   
   
   //instantiate the axi memory
`ifdef USE_DDR
   axi_ram 
     #(
 `ifdef DDR_INIT
       .FILE("firmware.hex"),
 `endif
       .FILE_SIZE(2**(`FIRM_ADDR_W-2)),
       .DATA_WIDTH (`MIG_BUS_W),
       .ADDR_WIDTH (`DDR_ADDR_W-10) //Simulation with full sized DDR takes some time to start
       )
   ddr_model_mem(
                 //address write
                 .clk            (clk),
                 .rst            (reset),
		 .s_axi_awid     ({8{ddr_awid}}),
		 .s_axi_awaddr   (ddr_awaddr[`DDR_ADDR_W-10-1:0]),
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
		 .s_axi_araddr   (ddr_araddr[`DDR_ADDR_W-10-1:0]),
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

   //
   // ETHERNET TASKS
   //

   task cpu_ethwrite;
      input [`ETH_ADDR_W-1:0]  cpu_address;
      input [31:0]  cpu_data;

      #1 eth_addr = cpu_address;
      eth_sel = 1;
      eth_we = 1;
      eth_data_in = cpu_data;
      @ (posedge clk) #1 eth_we = 0;
      eth_sel = 0;
   endtask

   task cpu_ethread;
      input [`ETH_ADDR_W-1:0]   cpu_address;
      output [31:0] read_reg;

      #1 eth_addr = cpu_address;
      eth_sel = 1;
      @ (posedge clk) #1 read_reg = eth_data_out;
      @ (posedge clk) #1 eth_sel = 0;
   endtask

   task cpu_ethinit;
      integer i;
      reg [31:0] cpu_reg;

      //preamble
      for(i=0; i < 15; i= i+1)
         data[i] = 8'h55;

      //sfd
      data[15] = 8'hD5;

      //dest mac address
      mac_addr = `ETH_MAC_ADDR;
      for(i=0; i < 6; i= i+1) begin
         data[i+16] = mac_addr[47:40];
         mac_addr = mac_addr<<8;
      end
      //source mac address
      mac_addr = `ETH_RMAC_ADDR;
      for(i=0; i < 6; i= i+1) begin
         data[i+22] = mac_addr[47:40];
         mac_addr = mac_addr<<8;
      end

      //eth type
      data[28] = 8'h08;
      data[29] = 8'h00;

      //reset core
      cpu_ethwrite(`ETH_SOFTRST, 1);
      cpu_ethwrite(`ETH_SOFTRST, 0);

      //wait for PHY to produce rx clock
      cpu_ethread(`ETH_STATUS, cpu_reg);
      while(!cpu_reg[3])
        cpu_ethread(`ETH_STATUS, cpu_reg);
      $display("TEST: Ethernet RX clock detected");

      //wait for PLL to lock and produce tx clock
      cpu_ethread(`ETH_STATUS, cpu_reg);
      while(!cpu_reg[15])
        cpu_ethread(`ETH_STATUS, cpu_reg);
      $display("TEST: Ethernet TX PLL locked");

      //set initial payload size to Ethernet minimum excluding FCS
      cpu_ethwrite(`ETH_TX_NBYTES, 46);
      cpu_ethwrite(`ETH_RX_NBYTES, 46);

      //check processor interface
      //write dummy register
      cpu_ethwrite(`ETH_DUMMY, 32'hDEADBEEF);

      //read and check result
      cpu_ethread(`ETH_DUMMY, cpu_reg);
      if (cpu_reg != 32'hDEADBEEF)
	$display("TEST: Ethernet Init failed");
      else
	$display("TEST: Ethernet Core Initialized");
   endtask

   //set frame size
   task cpu_ethset_rx_payload_size;
      cpu_ethwrite(`ETH_RX_NBYTES, `ETH_NBYTES);
   endtask

   task cpu_ethrcv_frame;
      input integer wait_flag;

      reg [31:0] cpu_reg;
      integer timeout;
      integer 	 i;
      timeout = 5000;

      //wait until data received
      do begin
	 cpu_ethread(`ETH_STATUS, cpu_reg);
         if(wait_flag == 0)  @(posedge clk) #1 timeout = timeout-1;
      end while( (!cpu_reg[1]) && (timeout != 0) );

      //check if timeout passed
      if(timeout == 0)
	$display("TEST: ETH_NO_DATA\n");
      else begin

         //read received data
	 for(i=14; i< `ETH_NBYTES+18 ; i = i+1) begin
	    cpu_ethread(`ETH_DATA +i, cpu_reg);
	    data[i+30-14] = cpu_reg;
	 end

         //confirm data was received
	 cpu_ethwrite(`ETH_RCVACK, 1);
      end
   endtask

   task cpu_ethsend_frame;
      integer i;
      reg [31:0] cpu_reg;

      //wait for ready
      do begin
	 cpu_ethread(`ETH_STATUS, cpu_reg);
      end while( !cpu_reg[0] );

      //set frame size
      cpu_ethwrite(`ETH_TX_NBYTES, `ETH_NBYTES);

      //write data to send
      for (i = 0; i < 30 + `ETH_NBYTES; i = i+1) begin
         cpu_ethwrite(`ETH_DATA + i, data[i]);
      end

      //start sending
      cpu_ethwrite(`ETH_SEND, `ETH_SEND);

   endtask

   
`include "cpu_tasks.v"
   
   //finish simulation
   //always @(posedge trap) begin
   // #10 $display("Found CPU trap condition");
   //$finish;
   //end
   
endmodule
