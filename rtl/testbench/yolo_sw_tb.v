`timescale 1ns / 1ps

`include "system.vh"
`include "iob-uart.vh"
`include "iob_eth_defs.vh"

`define ETH_NBYTES (1024-18)
`define INPUT_FILE_SIZE (418*418*3*2) //16 bits per input
`define OUTPUT_FILE_SIZE ((13*13*255+26*26*255)*2) //16 bits per point
`define NUM_OUTPUT_FRAMES (`OUTPUT_FILE_SIZE/`ETH_NBYTES)
`define WEIGTHS_BASE_ADDRESS 32768 //0x00008000
`define DATA_BASE_ADDRESS 21004288 //0x01408000
`define WEIGHT_FILE_SIZE (`DATA_BASE_ADDRESS+`INPUT_FILE_SIZE)
`define STRINGIFY(x) `"x`"

`define DATA_LAYER_17 (13*13*255*2)
`define DATA_LAYER_24 (26*26*255*2)
`define NUM_FRAMES_LAYER_17 (`DATA_LAYER_17/`ETH_NBYTES)
`define NUM_FRAMES_LAYER_24 (`DATA_LAYER_24/`ETH_NBYTES)

module yolo_sw_tb;

   //clock
   reg clk = 1;
   always #5 clk = ~clk;

   //reset
   reg reset = 1;

   //ethernet clocks
   parameter pclk_per = 40;
   reg RX_CLK = 1;
   always #(pclk_per/2) RX_CLK = ~RX_CLK;
   assign TX_CLK = RX_CLK;

   // program memory
   reg [31:0] progmem[`PROG_SIZE/4-1:0];

   //uart signals
   reg [7:0] 	rxread_reg = 8'b0;
   reg [2:0]    uart_addr;
   reg          uart_sel;
   reg          uart_wr;
   reg          uart_rd;
   reg [31:0]   uart_di;
   reg [31:0]   uart_do;

   //cpu to receive getchar
   reg [7:0]    cpu_char = 0;

   integer      i;
   reg          end_flag1, end_flag2 = 0;
`ifdef USE_BOOT
   reg          start_flag = 0;
`else
   reg          start_flag = 1;
`endif

   //
   // TEST PROCEDURE
   //
   initial begin

`ifdef VCD
      $dumpfile("system.vcd");
      $dumpvars(2, yolo_sw_tb.uut.eth, yolo_sw_tb.uut.picorv32_core);
      $dumpvars(0, test_eth);
`endif

      //init cpu bus signals
      uart_sel = 0;
      uart_wr = 0;
      uart_rd = 0;

      // deassert rst
      repeat (100) @(posedge clk);
      reset <= 0;

      //sync up with reset
      repeat (100) @(posedge clk) #1;

      //
      // CONFIGURE UART
      //
      cpu_inituart();

      do begin
         cpu_getchar(cpu_char);

         if (cpu_char == 2) begin // Send file
            cpu_sendFile(`PROG_SIZE);
         end else if (cpu_char == 3) begin // Receive file
            $write("Please, insert a name for a file:");
            $write("out.bin\n");
            cpu_receiveFile();
            start_flag = 1;
         end else if (cpu_char == 4) begin // Finish
            $write("Bye, bye!\n");
            end_flag1 = 1;
         end else begin
            $write("%c", cpu_char);
         end
      end while (cpu_char != 4);

   end // test procedure

   //
   // INSTANTIATE COMPONENTS
   //

   wire       tester_txd, tester_rxd;
   wire       tester_rts, tester_cts;
   wire       trap;

   // TESTER ETHERNET SIGNALS
   reg 	      eth_sel, eth_we;
   reg [`ETH_ADDR_W-1:0] eth_addr;
   reg [31:0] 	       eth_data_in, eth_data_out;

   wire 	       tester_pll_locked;
   wire 	       tester_eth_phy_resetn;
   wire 	       tester_tx_clk;
   wire [3:0] 	       tester_tx_data;
`ifdef LOOPBACK
   reg                 tester_tx_en;
`else
   wire 	       tester_tx_en;
`endif
   wire 	       tester_rx_clk;
   wire [3:0] 	       tester_rx_data;
   wire 	       tester_rx_dv;

   wire 	       eth_phy_resetn;

   assign tester_pll_locked = 1'b1;
   assign tester_rx_clk = RX_CLK;
   assign tester_tx_clk = TX_CLK;

   reg [7:0] data[`ETH_NBYTES+30-1:0];
   reg [47:0] mac_addr = `ETH_RMAC_ADDR;
   reg [7:0] output_fixed_data[`OUTPUT_FILE_SIZE-1:0];
   integer j, count_bytes = 0, bytes_to_receive, count_errors = 0, offset;

   parameter output_fixed_file = {`STRINGIFY(`FILES_DIR), "output_fixed.hex"};
   parameter file_ddr = {`STRINGIFY(`FILES_DIR), "weights_input_ntw_fixed.hex"};
   parameter file_size = (`WEIGHT_FILE_SIZE)/4;
   parameter file_addr_w = `ADDR_W-`N_SLAVES_W;

`ifndef LOOPBACK
   // TEST ETHERNET
   initial begin

      //init signals
      eth_sel = 0;
      eth_we = 0;

      //wait for reset
      repeat (200) @(posedge clk) #1;

      //init ethernet module
      do
        repeat (200) @(posedge clk) #1;
      while(start_flag == 0);
      cpu_ethinit();
      cpu_ethset_rx_payload_size();

      //read output.network
      $readmemh(output_fixed_file, output_fixed_data, 0, `OUTPUT_FILE_SIZE-1);

      //wait a bit for RISC-V to be ready to receive data
      repeat (100000) @(posedge clk) #1;

      //for loop to receive output of first yolo layer
      for(j = 0; j < `NUM_FRAMES_LAYER_17+1; j = j+1) begin

         //check if it is last packet (has less data that full payload size)
         if(j == `NUM_FRAMES_LAYER_17) begin
            bytes_to_receive = `DATA_LAYER_17 - count_bytes;
            //set remaining bytes to zero
            for(i = bytes_to_receive; i < `ETH_NBYTES; i=i+1) begin
                data[i+30] = 8'b0;
            end
         end else bytes_to_receive = `ETH_NBYTES;

         //wait to receive frame back
         cpu_ethrcv_frame(1);
         count_bytes = count_bytes + bytes_to_receive;

         //check that sent and receive packages are the same
         for(i = 0; i < bytes_to_receive; i=i+1) begin
            if(data[i+30] != output_fixed_data[j*`ETH_NBYTES+i]) count_errors = count_errors + 1;
         end

         //print progress each 100 cycles
         if( (j%100) == 0) $display("iter = %d, total_err = %d", j, count_errors);
         repeat (200) @(posedge clk) #1;

      end

      //reset byte counter
      offset = count_bytes;
      count_bytes = 0;

      //for loop to receive output of second yolo layer
      for(j = 0; j < `NUM_FRAMES_LAYER_24+1; j = j+1) begin

         //check if it is last packet (has less data that full payload size)
         if(j == `NUM_FRAMES_LAYER_24) begin
            bytes_to_receive = `DATA_LAYER_24 - count_bytes;
            //set remaining bytes to zero
            for(i = bytes_to_receive; i < `ETH_NBYTES; i=i+1) begin
                data[i+30] = 8'b0;
            end
         end else bytes_to_receive = `ETH_NBYTES;

         //wait to receive frame back
         cpu_ethrcv_frame(1);
         count_bytes = count_bytes + bytes_to_receive;

         //check that sent and receive packages are the same
         for(i = 0; i < bytes_to_receive; i=i+1) begin
            if(data[i+30] != output_fixed_data[j*`ETH_NBYTES+i+offset]) count_errors = count_errors + 1;
         end

         //print progress each 100 cycles
         if( (j%100) == 0) $display("iter = %d, total_err = %d", j, count_errors);
         repeat (200) @(posedge clk) #1;

      end

      //end of procedure
      repeat (50000) @(posedge clk) #1;
      $display("output.network received with %d errors", count_errors);
      end_flag2 = 1;

   end
`endif

`ifdef USE_DDR
   //Write address
   wire [0:0] 			ddr_awid;
   wire [31:0] 			ddr_awaddr;
   wire [7:0] 			ddr_awlen;
   wire [2:0] 			ddr_awsize;
   wire [1:0] 			ddr_awburst;
   wire                 ddr_awlock;
   wire [3:0] 			ddr_awcache;
   wire [2:0] 			ddr_awprot;
   wire [3:0] 			ddr_awqos;
   wire                 ddr_awvalid;
   wire                 ddr_awready;
   //Write data
   wire [31:0] 			ddr_wdata;
   wire [3:0] 			ddr_wstrb;
   wire                 ddr_wlast;
   wire                 ddr_wvalid;
   wire                 ddr_wready;
   //Write response
   wire [7:0]           ddr_bid;
   wire [1:0]           ddr_bresp;
   wire                 ddr_bvalid;
   wire                 ddr_bready;
   //Read address
   wire [0:0] 			ddr_arid;
   wire [31:0] 			ddr_araddr;
   wire [7:0] 			ddr_arlen;
   wire [2:0] 			ddr_arsize;
   wire [1:0] 			ddr_arburst;
   wire                 ddr_arlock;
   wire [3:0] 			ddr_arcache;
   wire [2:0] 			ddr_arprot;
   wire [3:0] 			ddr_arqos;
   wire                 ddr_arvalid;
   wire                 ddr_arready;
   //Read data
   wire [7:0]			ddr_rid;
   wire [31:0] 			ddr_rdata;
   wire [1:0] 			ddr_rresp;
   wire                 ddr_rlast;
   wire                 ddr_rvalid;
   wire                 ddr_rready;
`endif

   //
   // UNIT UNDER TEST
   //
   system uut (
	           .clk                 (clk),
	           .reset               (reset),
	           .trap                (trap),

`ifdef USE_DDR

                   //DDR
                   //address write
	           .m_axi_awid          (ddr_awid),
	           .m_axi_awaddr        (ddr_awaddr),
	           .m_axi_awlen         (ddr_awlen),
	           .m_axi_awsize        (ddr_awsize),
	           .m_axi_awburst       (ddr_awburst),
	           .m_axi_awlock        (ddr_awlock),
	           .m_axi_awcache       (ddr_awcache),
	           .m_axi_awprot        (ddr_awprot),
	           .m_axi_awqos         (ddr_awqos),
	           .m_axi_awvalid       (ddr_awvalid),
	           .m_axi_awready       (ddr_awready),

	           //write
	           .m_axi_wdata         (ddr_wdata),
	           .m_axi_wstrb         (ddr_wstrb),
	           .m_axi_wlast         (ddr_wlast),
	           .m_axi_wvalid        (ddr_wvalid),
	           .m_axi_wready        (ddr_wready),

	           //write response
	           .m_axi_bid           (ddr_bid[0]),
	           .m_axi_bresp         (ddr_bresp),
	           .m_axi_bvalid        (ddr_bvalid),
	           .m_axi_bready        (ddr_bready),

	           //address read
	           .m_axi_arid          (ddr_arid),
	           .m_axi_araddr        (ddr_araddr),
	           .m_axi_arlen         (ddr_arlen),
	           .m_axi_arsize        (ddr_arsize),
	           .m_axi_arburst       (ddr_arburst),
	           .m_axi_arlock        (ddr_arlock),
	           .m_axi_arcache       (ddr_arcache),
	           .m_axi_arprot        (ddr_arprot),
	           .m_axi_arqos         (ddr_arqos),
	           .m_axi_arvalid       (ddr_arvalid),
	           .m_axi_arready       (ddr_arready),

	           //read
	           .m_axi_rid           (ddr_rid[0]),
	           .m_axi_rdata         (ddr_rdata),
	           .m_axi_rresp         (ddr_rresp),
	           .m_axi_rlast         (ddr_rlast),
	           .m_axi_rvalid        (ddr_rvalid),
	           .m_axi_rready        (ddr_rready),
`endif

                   //UART
	           .uart_txd            (tester_rxd),
	           .uart_rxd            (tester_txd),
	           .uart_rts            (tester_cts),
	           .uart_cts            (tester_rts),

                   //ETHERNET
                   .PLL_LOCKED          (tester_pll_locked),
                   .ETH_PHY_RESETN      (eth_phy_resetn),
                   .RX_CLK              (tester_tx_clk),
                   .RX_DATA             (tester_tx_data),
                   .RX_DV               (tester_tx_en),
                   .TX_CLK              (tester_rx_clk),
                   .TX_EN               (tester_rx_dv),
                   .TX_DATA             (tester_rx_data)
	           );

   //Receive back everything being sent
`ifdef LOOPBACK
   assign tester_tx_en =  tester_rx_dv;
   assign tester_tx_data = tester_rx_data;
`endif


   //TESTER UART
   iob_uart test_uart (
		               .clk       (clk),
		               .rst       (reset),

		               .sel       (uart_sel),
		               .address   (uart_addr),
		               .write     (uart_wr),
		               .read      (uart_rd),
		               .data_in   (uart_di),
		               .data_out  (uart_do),

		               .txd       (tester_txd),
		               .rxd       (tester_rxd),
		               .rts       (tester_rts),
		               .cts       (tester_cts)
		               );


   //
   // TESTER ETHERNET
   //

`ifndef LOOPBACK
   iob_eth  #(
             .ETH_MAC_ADDR(`ETH_RMAC_ADDR)
             )

               test_eth (

               //cpu interface
               .clk                  (clk),
               .rst                  (reset),

               //cpu i/f
               .sel                  (eth_sel),
               .we                   (eth_we),
               .addr                 (eth_addr),
               .data_out             (eth_data_out),
               .data_in              (eth_data_in),

               //ethernet i/f
               .PLL_LOCKED           (tester_pll_locked),
               .ETH_PHY_RESETN       (tester_eth_phy_resetn),
               .TX_CLK               (tester_tx_clk),
               .TX_DATA              (tester_tx_data),
               .TX_EN                (tester_tx_en),
               .RX_CLK               (tester_rx_clk),
               .RX_DATA              (tester_rx_data),
               .RX_DV                (tester_rx_dv)
               );
`endif


`ifdef USE_DDR
   axi_ram #(
            .ADDR_WIDTH(file_addr_w),
            .FILE(file_ddr),
            .FILE_SIZE(file_size)
            )
   ddr_model_mem(
                 //address write
                   .clk            (clk),
                   .rst            (reset),
		           .s_axi_awid     ({8{ddr_awid}}),
		           .s_axi_awaddr   (ddr_awaddr[file_addr_w-1:0]),
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
		           .s_axi_araddr   (ddr_araddr[file_addr_w-1:0]),
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
   // CPU TASKS
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

   task eth_printstatus();
      reg [31:0] cpu_reg;
      cpu_ethread(`ETH_STATUS, cpu_reg);
      $display("tx_ready = %x", cpu_reg[0]);
      $display("rx_ready = %x", cpu_reg[1]);
      $display("phy_dv_detected = %x", cpu_reg[2]);
      $display("phy_clk_detected = %x", cpu_reg[3]);
      $display("rx_wr_addr = %x", cpu_reg[19:4]);
      cpu_ethread(`ETH_CRC, cpu_reg);
      $display("CRC = %x", cpu_reg);

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

   // 1-cycle write
   task cpu_uartwrite;
      input [3:0]  cpu_address;
      input [31:0] cpu_data;

      # 1 uart_addr = cpu_address;
      uart_sel = 1;
      uart_wr = 1;
      uart_di = cpu_data;
      @ (posedge clk) #1 uart_wr = 0;
      uart_sel = 0;
   endtask //cpu_uartwrite

   // 2-cycle read
   task cpu_uartread;
      input [3:0]   cpu_address;
      output [31:0] read_reg;

      # 1 uart_addr = cpu_address;
      uart_sel = 1;
      uart_rd = 1;
      @ (posedge clk) #1 read_reg = uart_do;
      @ (posedge clk) #1 uart_rd = 0;
      uart_sel = 0;
   endtask //cpu_uartread

   task cpu_sendFile;
      input [`DATA_W-1:0] file_size;
      integer             i, j, k;

      $readmemh("progmem.hex", progmem, 0, file_size/4-1);

      $write("Starting File 'progmem.hex' Transfer...\n");
      $write("file_size = %d\n", file_size);

      // Send file size
      cpu_putchar(file_size[7:0]);
      cpu_putchar(file_size[15:8]);
      cpu_putchar(file_size[23:16]);
      cpu_putchar(file_size[31:24]);

      k = 0;
      for(i = 0; i < file_size/4; i++) begin
	     for(j=1; j<=4; j=j+1) begin
	        cpu_putchar(progmem[i][j*8-1 -: 8]);
	     end

         if(i == (file_size/4*k/100)) begin
            $write("%d%%\n", k);
            k=k+10;
         end
      end
      $write("%d%%\n", 100);
      $write("UART transfer complete.\n");
   endtask

   task cpu_receiveFile;
      reg [`DATA_W-1:0] file_size;
      integer           fp;
      integer           i, j, k;

      fp = $fopen("out.bin","wb");

      $write("Starting File 'out.bin' Transfer...\n");

      // Send file size
      cpu_getchar(file_size[7:0]);
      cpu_getchar(file_size[15:8]);
      cpu_getchar(file_size[23:16]);
      cpu_getchar(file_size[31:24]);
      $write("file_size = %d\n", file_size);

      k = 0;
      for(i = 0; i < file_size; i++) begin
	     //for(j=1; j<=4; j=j+1) begin
	     cpu_getchar(cpu_char);
         $fwrite(fp, "%c", cpu_char);
	     //end

         if(i/4 == (file_size/4*k/100)) begin
            $write("%d%%\n", k);
            k=k+10;
         end
      end
      $write("%d%%\n", 100);
      $write("UART transfer complete.\n");
   endtask


   task cpu_inituart;
      //pulse reset uart
      cpu_uartwrite(`UART_SOFT_RESET, 1);
      cpu_uartwrite(`UART_SOFT_RESET, 0);
      //config uart div factor
      cpu_uartwrite(`UART_DIV, `UART_CLK_FREQ/`UART_BAUD_RATE);
      //enable uart for receiving
      cpu_uartwrite(`UART_RXEN, 1);
   endtask

   task cpu_getchar;
      output [7:0] rcv_char;

      //wait until something is received
      do
	    cpu_uartread(`UART_READ_VALID, rxread_reg);
      while(!rxread_reg);

      //read the data
      cpu_uartread(`UART_DATA, rxread_reg);

      rcv_char = rxread_reg[7:0];
   endtask


   task cpu_putchar;
      input [7:0] send_char;
      //wait until tx ready
      do
	    cpu_uartread(`UART_WRITE_WAIT, rxread_reg);
      while(rxread_reg);
      //write the data
      cpu_uartwrite(`UART_DATA, send_char);

   endtask

   task cpu_getline;
      do begin
         cpu_getchar(cpu_char);
         $write("%c", cpu_char);
      end while (cpu_char != "\n");
   endtask


   // finish simulation
   always @(posedge clk)
     begin
        if (end_flag1 == 1)
           if (end_flag2 == 1)
              $finish;
     end

endmodule
