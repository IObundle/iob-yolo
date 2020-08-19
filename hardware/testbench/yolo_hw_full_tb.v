`timescale 1ns / 1ps

`include "system.vh"
`include "iob_uart.vh"

//DDR initial constants
`define NEW_W (416)
`define NEW_H (312)
`define ix_size (`NEW_W*4)
`define iy_size (`NEW_H)
`define dx_size (`NEW_W*2)
`define dy_size (`NEW_H*2)
`define DDR_INITIAL (2**`FIRM_ADDR_W + (`ix_size+`iy_size+`dx_size+`dy_size)*2)

//FM constants
`define IMAGE_INPUT (768*576*3)
`define NETWORK_INPUT_AUX (416*2*312*3)
`define NETWORK_INPUT (418*418*3)
`define LAYER_1 (210*210*16)
`define LAYER_3 (106*106*32)
`define LAYER_5 (54*54*64)
`define LAYER_7 (28*28*128)
`define LAYER_9 (28*28*256)
`define LAYER_10 (15*15*256)
`define LAYER_11 (14*14*512)
`define LAYER_12 (15*15*512)
`define LAYER_13 (13*13*1024)
`define LAYER_14 (15*15*256)
`define LAYER_15 (13*13*512)
`define LAYER_16 (13*13*255)
`define LAYER_19 (13*13*128)
`define LAYER_20 (28*28*128)
`define LAYER_22 (26*26*256)
`define LAYER_23 (26*26*255)
`define NBOXES 8
`define YOLO (`NBOXES*84)
`define TOTAL_FM ((`IMAGE_INPUT+`NETWORK_INPUT_AUX+`NETWORK_INPUT+`LAYER_1+`LAYER_3+`LAYER_5+`LAYER_7+`LAYER_9+`LAYER_10+`LAYER_11+`LAYER_12+`LAYER_13+`LAYER_14+`LAYER_15+`LAYER_16+`LAYER_19+`LAYER_20+`LAYER_22+`LAYER_23+`YOLO)*2)

//Weight constants
`define WEIGHTS_LAYER_1 (16 + 16*3*3*3)
`define WEIGHTS_LAYER_3 (32 + 32*3*3*16)
`define WEIGHTS_LAYER_5 (64 + 64*3*3*32)
`define WEIGHTS_LAYER_7 (128 + 128*3*3*64)
`define WEIGHTS_LAYER_9 (256 + 256*3*3*128)
`define WEIGHTS_LAYER_11 (512 + 512*3*3*256)
`define WEIGHTS_LAYER_13 (1024 + 1024*3*3*512)
`define WEIGHTS_LAYER_14 (256 + 256*1*1*1024)
`define WEIGHTS_LAYER_15 (512 + 512*3*3*256)
`define WEIGHTS_LAYER_16 (255 + 255*1*1*512)
`define WEIGHTS_LAYER_19 (128 + 128*1*1*256)
`define WEIGHTS_LAYER_22 (256 + 256*3*3*384)
`define WEIGHTS_LAYER_23 (255 + 255*1*1*256)
`define TOTAL_WEIGHTS ((`WEIGHTS_LAYER_1+`WEIGHTS_LAYER_3+`WEIGHTS_LAYER_5+`WEIGHTS_LAYER_7+`WEIGHTS_LAYER_9+`WEIGHTS_LAYER_11+`WEIGHTS_LAYER_13+`WEIGHTS_LAYER_14+`WEIGHTS_LAYER_15+`WEIGHTS_LAYER_16+`WEIGHTS_LAYER_19+`WEIGHTS_LAYER_22+`WEIGHTS_LAYER_23)*2)

//Total constants
`define FILE_SIZE ((`DDR_INITIAL + `TOTAL_WEIGHTS + `TOTAL_FM)/4)

module yolo_hw_full_tb;

   //clock
   reg clk = 1;
   always #5 clk = ~clk;

   //reset 
   reg reset = 1;

   //received by getchar
   reg [7:0] cpu_char = 0;


   //tester uart
   reg       uart_valid;
   reg [`UART_ADDR_W-1:0] uart_addr;
   reg [`DATA_W-1:0]      uart_wdata;
   reg                    uart_wstrb;
   reg [`DATA_W-1:0]      uart_rdata;
   wire                   uart_ready;

   //iterator
   integer                i;

   //define parameters
   parameter file_ddr = "../../../yolo.hex";
   parameter file_size = `FILE_SIZE;

   /////////////////////////////////////////////
   // TEST PROCEDURE
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
	       // .m_axi_bid     (ddr_bid[0]),
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
	       // .m_axi_rid     (ddr_rid[0]),
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
	       .uart_cts      (tester_rts)
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
