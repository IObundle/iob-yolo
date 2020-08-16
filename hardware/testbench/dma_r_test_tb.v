`timescale 1ns / 1ps

`include "system.vh"
`include "iob_uart.vh"
`include "xversat.vh"

//constants
`define STRINGIFY(x) `"x`"
`define FILE_SIZE ((16*32)/(`MIG_BUS_W/8))

module dma_r_test_tb;

   //define parameters
   parameter file_name = {"../../../../dma_", `STRINGIFY(`MIG_BUS_W), ".hex"};
   parameter clk_per = 10;
   parameter file_size = `FILE_SIZE;

   //clock
   reg clk = 1;
   always #(clk_per/2) clk = ~clk;

   //reset 
   reg reset = 1;

   //iterator
   integer                i;

   /////////////////////////////////////////////
   // RAM signals
   /////////////////////////////////////////////

   wire 				en, we;
   wire [`MEM_ADDR_W-1:0]		addr;
   wire [`MIG_BUS_W-1:0]		data_in;

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
   wire [`MIG_BUS_W-1:0]              	databus_rdata;

   /////////////////////////////////////////////
   // TEST PROCEDURE
   /////////////////////////////////////////////
   
   initial begin

`ifdef VCD
      $dumpfile("system.vcd");
      $dumpvars();
`endif

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

      //configure ext_addrgen to write 16 lines to local mem
      iterations <= `EXT_ADDR_W'd1;
      period <= `EXT_PERIOD_W'd16;

      //run and wait for done
      run_conf();
      conf_done();
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

   //
   // SINGLE PORT RAM
   //

   iob_sp_ram # (
      .DATA_W(`MIG_BUS_W),
      .ADDR_W(`MEM_ADDR_W)
   ) ram (
      .clk(clk),
      .en(en),
      .we(we),
      .addr(addr),
      .data_in(data_in),
      .data_out()
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
      .direction(2'b01), //EXT2INT
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
      .databus_rdata(databus_rdata),
      .databus_wdata(),
      .databus_wstrb(),
      //mem interface
      .valid(en),
      .we(we),
      .addr(addr),
      .data_out(data_in),
      .data_in(`MIG_BUS_W'b0)
   );

   //
   // UNIT UNDER TEST (axi-dma)
   //
   
   axi_dma_r dma (
      .clk(clk),
      .rst(reset),
      //databus interface
      .ready(databus_ready),
      .valid(databus_valid),
      .addr(databus_addr[`DDR_ADDR_W-1:0]),
      .rdata(databus_rdata),
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
      //data read
      .m_axi_rid     (ddr_rid[0]),
      .m_axi_rdata   (ddr_rdata),
      .m_axi_rresp   (ddr_rresp),
      .m_axi_rlast   (ddr_rlast),
      .m_axi_rvalid  (ddr_rvalid),
      .m_axi_rready  (ddr_rready)
   );

   //instantiate the axi memory
   axi_ram 
     #(
       .FILE(file_name),
       .FILE_SIZE(file_size),
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
