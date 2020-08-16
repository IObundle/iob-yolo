`timescale 1ns / 1ps

`include "xversat.vh"
`include "xyolo_read.vh"
`include "axi_dma.vh"

module xyolo_read #(
    	parameter                      	DATAPATH_W = 32,
	parameter			DATABUS_W = 256
   ) (
    	input                           clk,
    	input                           rst,

	// control
	input				clear,
    	input                           run,
    	output                          done,

	// cpu interface (only request)
	input				valid,
	input [`XYOLO_READ_ADDR_W-1:0]	addr,
	input [`IO_ADDR_W-1:0]		wdata,
	input 			    	wstrb,

    	// databus interface
    	input               		databus_ready,
    	output 			        databus_valid,
    	output [`IO_ADDR_W-1:0]  	databus_addr,
    	input  [DATABUS_W-1:0]      	databus_rdata,
    	output [DATABUS_W-1:0]      	databus_wdata,
    	output [DATABUS_W/8-1:0]    	databus_wstrb,

    	// output data
    	output [`nYOLOvect*DATAPATH_W-1:0] flow_out_bias,
    	output [`nYOLOvect*DATAPATH_W-1:0] flow_out_weight,

	// DMA - number of tranfers per burst
	output [`AXI_LEN_W-1:0]		dma_len
   );

   // local parameter for merge
   localparam                           ADDR_W = `IO_ADDR_W;
   localparam				DATA_W = DATABUS_W;

   // configuration enables
   reg  		                ext_addr_en;
   reg  		                offset_en;
   reg					len_en;
   reg 					int_addr_en;
   reg 					iterA_en;
   reg 					perA_en;
   reg 					shiftA_en;
   reg 					incrA_en;
   reg 					iterB_en;
   reg 					perB_en;
   reg 					startB_en;
   reg 					shiftB_en;
   reg 					incrB_en;

   // configuration parameters
   reg [`IO_ADDR_W-1:0] 	        ext_addr;
   reg [`nYOLOvect*`IO_ADDR_W-1:0]      ext_addr_shadow;
   reg [`IO_ADDR_W/2-1:0] 	        offset;
   reg [`AXI_LEN_W-1:0] 		len, len_shadow;
   reg [`W_ADDR_W-1:0] 			int_addr, int_addr_shadow;
   reg [`MEM_ADDR_W-1:0] 		iterA, iterA_shadow;
   reg [`PERIOD_W-1:0] 			perA, perA_shadow;
   reg [`MEM_ADDR_W-1:0] 		shiftA, shiftA_shadow;
   reg [`MEM_ADDR_W-1:0] 		incrA, incrA_shadow;
   reg [`MEM_ADDR_W-1:0] 		iterB, iterB_pip, iterB_shadow;
   reg [`PERIOD_W-1:0] 			perB, perB_pip, perB_shadow;
   reg [`MEM_ADDR_W-1:0] 		startB, startB_pip, startB_shadow;
   reg [`MEM_ADDR_W-1:0] 		shiftB, shiftB_pip, shiftB_shadow;
   reg [`MEM_ADDR_W-1:0] 		incrB, incrB_pip, incrB_shadow;

   //external address bus
   wire [`nYOLOvect*`IO_ADDR_W-1:0]     ext_addr_bus;

   // mem enables output by addr gen
   wire [`nYOLOvect-1:0]                enA, we;
   reg [`nYOLOvect-1:0]                 enA_reg, we_reg;
   wire                                 enB;
   reg                                  enB_reg;

   // port addresses and enables
   wire [`nYOLOvect*`W_ADDR_W-1:0]      addrA;
   reg [`nYOLOvect*`W_ADDR_W-1:0]       addrA_reg;
   wire [`MEM_ADDR_W-1:0]               addrB;
   reg [`MEM_ADDR_W-1:0]                addrB_reg;

   // data inputs
   wire [`nYOLOvect*DATABUS_W-1:0]      inA;
   reg [`nYOLOvect*DATABUS_W-1:0]       inA_reg;

   // data output
   wire [`nYOLOvect*DATAPATH_W-1:0]     weights;
   reg [`nYOLOvect*DATAPATH_W-1:0]      weights_reg;

   // done output
   wire [`nYOLOvect-1:0]                vread_done;
   wire                                 doneB;
   assign                               done = &{vread_done, doneB};

   // define number of transactions (DMA)
   assign				dma_len = len_shadow;

   // bias and run registers
   reg					run_reg;
   reg [`nYOLOvect*DATAPATH_W-1:0]	bias0, bias1;
   reg [`nYOLOvect*DATAPATH_W-1:0]      bias0_reg, bias1_reg;
   assign 				flow_out_bias = addrB[`MEM_ADDR_W-1] ? bias1_reg : bias0_reg;

   // merge master interface
   wire [`nYOLOvect*`REQ_W-1:0]         m_req;
   wire [`nYOLOvect*`RESP_W-1:0]        m_resp;

   //merge slave interface
   wire [`REQ_W-1:0]                    s_req;
   wire [`RESP_W-1:0]                   s_resp;

   // register run and bias
   always @ (posedge clk, posedge rst)
      if(rst) begin
	 run_reg <= 1'b0;
	 bias0_reg <= {`nYOLOvect*DATAPATH_W{1'b0}};
	 bias1_reg <= {`nYOLOvect*DATAPATH_W{1'b0}};
      end else begin
	 run_reg <= run;
	 bias0_reg <= bias0;
	 bias1_reg <= bias1;
      end

   // addr decoder enable
   always @* begin
      ext_addr_en = 1'b0;
      offset_en = 1'b0;
      len_en = 1'b0;
      int_addr_en = 1'b0;
      iterA_en = 1'b0;
      perA_en = 1'b0;
      shiftA_en = 1'b0;
      incrA_en = 1'b0;
      iterB_en = 1'b0;
      perB_en = 1'b0;
      startB_en = 1'b0;
      shiftB_en = 1'b0;
      incrB_en = 1'b0;
      if(valid & wstrb)
         case(addr)
            `XYOLO_READ_CONF_EXT_ADDR : ext_addr_en = 1'b1;
            `XYOLO_READ_CONF_OFFSET : offset_en = 1'b1;
            `XYOLO_READ_CONF_LEN : len_en = 1'b1;
            `XYOLO_READ_CONF_INT_ADDR : int_addr_en = 1'b1;
            `XYOLO_READ_CONF_ITER_A : iterA_en = 1'b1;
	    `XYOLO_READ_CONF_PER_A: perA_en = 1'b1;
	    `XYOLO_READ_CONF_SHIFT_A : shiftA_en = 1'b1;
	    `XYOLO_READ_CONF_INCR_A : incrA_en = 1'b1;
	    `XYOLO_READ_CONF_ITER_B : iterB_en = 1'b1;
	    `XYOLO_READ_CONF_PER_B : perB_en = 1'b1;
	    `XYOLO_READ_CONF_START_B : startB_en = 1'b1;
	    `XYOLO_READ_CONF_SHIFT_B : shiftB_en = 1'b1;
	    `XYOLO_READ_CONF_INCR_B : incrB_en = 1'b1;
	    default : ;
	 endcase
   end

   // addr decoder parameters
   always @(posedge clk, posedge clear, posedge rst)
      if(clear || rst) begin
	 ext_addr <= `IO_ADDR_W'b0;
         offset <= {`IO_ADDR_W/2{1'b0}};
         len <= `AXI_LEN_W'b0;
	 int_addr <= {`W_ADDR_W{1'b0}};
	 iterA <= `MEM_ADDR_W'b0;
	 perA <= `PERIOD_W'b0;
	 shiftA <= `MEM_ADDR_W'b0;
	 incrA <= `MEM_ADDR_W'b0;
	 iterB <= `MEM_ADDR_W'b0;
	 perB <= `PERIOD_W'b0;
	 startB <= `MEM_ADDR_W'b0;
	 shiftB <= `MEM_ADDR_W'b0;
	 incrB <= `MEM_ADDR_W'b0;
      end else begin
         if(ext_addr_en) ext_addr <= wdata[`IO_ADDR_W-1:0];
         if(offset_en) offset <= wdata[`IO_ADDR_W/2-1:0];
         if(len_en) len <= wdata[`AXI_LEN_W-1:0];
         if(int_addr_en) int_addr <= wdata[`W_ADDR_W-1:0];
         if(iterA_en) iterA <= wdata[`MEM_ADDR_W-1:0];
         if(perA_en) perA <= wdata[`PERIOD_W-1:0];
         if(shiftA_en) shiftA <= wdata[`MEM_ADDR_W-1:0];
         if(incrA_en) incrA <= wdata[`MEM_ADDR_W-1:0];
         if(iterB_en) iterB <= wdata[`MEM_ADDR_W-1:0];
         if(perB_en) perB <= wdata[`PERIOD_W-1:0];
         if(startB_en) startB <= wdata[`MEM_ADDR_W-1:0];
         if(shiftB_en) shiftB <= wdata[`MEM_ADDR_W-1:0];
         if(incrB_en) incrB <= wdata[`MEM_ADDR_W-1:0];
      end

   // configurable parameters shadow register
   always @(posedge clk, posedge rst)
      if(rst) begin
	 ext_addr_shadow <= `nYOLOvect*`IO_ADDR_W'b0;
	 len_shadow <= `AXI_LEN_W'b0;
	 int_addr_shadow <= {`W_ADDR_W{1'b0}};
	 iterA_shadow <= `MEM_ADDR_W'b0;
	 perA_shadow <= `PERIOD_W'b0;
	 shiftA_shadow <= `MEM_ADDR_W'b0;
	 incrA_shadow <= `MEM_ADDR_W'b0;
	 iterB_shadow <= `MEM_ADDR_W'b0;
	 iterB_pip <= `MEM_ADDR_W'b0;
	 perB_shadow <= `PERIOD_W'b0;
	 perB_pip <= `PERIOD_W'b0;
	 startB_shadow <= `MEM_ADDR_W'b0;
	 startB_pip <= `MEM_ADDR_W'b0;
	 shiftB_shadow <= `MEM_ADDR_W'b0;
	 shiftB_pip <= `MEM_ADDR_W'b0;
	 incrB_shadow <= `MEM_ADDR_W'b0;
	 incrB_pip <= `MEM_ADDR_W'b0;
      end else if(run) begin
         ext_addr_shadow <= ext_addr_bus;
	 len_shadow <= len;
	 //XOR ensures ping-pong happens when acessing external mem
	 int_addr_shadow <= {int_addr_shadow[`W_ADDR_W-1] ^ |iterA, int_addr[`W_ADDR_W-2:0]};
	 iterA_shadow <= iterA;
	 perA_shadow <= perA;
	 shiftA_shadow <= shiftA;
	 incrA_shadow <= incrA;
	 iterB_pip <= iterB;
	 iterB_shadow <= iterB_pip;
	 perB_pip <= perB;
	 perB_shadow <= perB_pip;
	 startB_pip <= {startB_pip[`MEM_ADDR_W-1] ^ |iterA, startB[`MEM_ADDR_W-2:0]};
	 startB_shadow <= startB_pip;
	 shiftB_pip <= shiftB;
	 shiftB_shadow <= shiftB_pip;
	 incrB_pip <= incrB;
	 incrB_shadow <= incrB_pip;
      end

   // external address calculation based on base and offset values
   genvar i;
   assign ext_addr_bus[`nYOLOvect*`IO_ADDR_W-1 -: `IO_ADDR_W] = ext_addr; //base value
   generate
      for (i=1; i < `nYOLOvect; i=i+1) begin : addrgen_calc

         //instantiate 4-stage multiplier
         mul_4stage # (
            .DATA_W(`IO_ADDR_W/2)
         ) mul (
            //control
            .clk(clk),
            .ld_acc(1'b1),
            //data
            .inA(i[`IO_ADDR_W/2-1:0]),
            .inB(offset),
            .inC(ext_addr),
            .out(ext_addr_bus[`nYOLOvect*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W])
         );

      end
   endgenerate

   // store bias
   always @ * begin
      integer j;
      bias0 = bias0_reg;
      bias1 = bias1_reg;
      for(j = 0; j < `nYOLOvect; j++)
	 if(enA[j] & we[j] & (addrA[`nYOLOvect*`W_ADDR_W-`W_ADDR_W*j-2 -: `W_ADDR_W-1] == {`W_ADDR_W-1{1'b0}}))
	    if(addrA[`nYOLOvect*`W_ADDR_W-`W_ADDR_W*j-1])
	       bias1[`nYOLOvect*DATAPATH_W-DATAPATH_W*j-1 -: DATAPATH_W] = inA[`nYOLOvect*DATABUS_W-DATABUS_W*j-(DATABUS_W/DATAPATH_W-1)*DATAPATH_W-1 -: DATAPATH_W];
	    else
	       bias0[`nYOLOvect*DATAPATH_W-DATAPATH_W*j-1 -: DATAPATH_W] = inA[`nYOLOvect*DATABUS_W-DATABUS_W*j-(DATABUS_W/DATAPATH_W-1)*DATAPATH_W-1 -: DATAPATH_W];
   end

   // register mem inputs
   always @ (posedge clk) begin
      // read port
      enA_reg <= enA;
      we_reg <= we;
      addrA_reg <= addrA;
      inA_reg <= inA;
      // write port
      enB_reg <= enB;
      addrB_reg <= addrB;
   end

   // instantiate external address generators and internal memories
   generate
      for (i=0; i < `nYOLOvect; i=i+1) begin : vread_array

         // external address generator
         ext_addrgen #(
            .DATA_W(DATABUS_W),
	    .MEM_ADDR_W(`W_ADDR_W)
         ) addrgenA (
	    .clk(clk),
	    .rst(rst),

            // Control
	    .run(run_reg),
	    .int_cnt_en(1'b1),
            .done(vread_done[i]),

            // Configuration
	    .ext_addr(ext_addr_shadow[`nYOLOvect*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W]),
            .int_addr(int_addr_shadow),
	    .direction(2'b01),
	    .iterations(iterA_shadow),
	    .period(perA_shadow),
	    .duty(perA_shadow),
	    .start(`MEM_ADDR_W'd0),
	    .shift(shiftA_shadow),
	    .incr(incrA_shadow),
	    .delay(`PERIOD_W'd0),

            // Databus interface
 	    .databus_ready(m_resp[`ready((`nYOLOvect-i-1))]),
 	    .databus_valid(m_req[`valid((`nYOLOvect-i-1))]),
	    .databus_addr(m_req[`address((`nYOLOvect-i-1), `IO_ADDR_W)]),
	    .databus_rdata(m_resp[`rdata((`nYOLOvect-i-1))]),
	    .databus_wdata(m_req[`wdata((`nYOLOvect-i-1))]),
	    .databus_wstrb(m_req[`wstrb((`nYOLOvect-i-1))]),

            // internal memory interface
            .valid(enA[i]),
            .we(we[i]),
            .addr(addrA[`nYOLOvect*`W_ADDR_W-`W_ADDR_W*i-1 -: `W_ADDR_W]),
            .data_out(inA[`nYOLOvect*DATABUS_W-DATABUS_W*i-1 -: DATABUS_W]),
            .data_in({DATABUS_W{1'b0}})
	    );

         //internal memory
         iob_2p_assim_mem_w_big #(
	    .W_DATA_W(DATABUS_W),
      	    .W_ADDR_W(`W_ADDR_W),
      	    .R_DATA_W(DATAPATH_W),
      	    .R_ADDR_W(`MEM_ADDR_W),
            .USE_RAM(0)
         ) mem (
           .clk(clk),

           // Writting port
           .w_en(enA_reg[i] & we_reg[i]),
           .w_addr(addrA_reg[`nYOLOvect*`W_ADDR_W-`W_ADDR_W*i-1 -: `W_ADDR_W]),
           .data_in(inA_reg[`nYOLOvect*DATABUS_W-DATABUS_W*i-1 -: DATABUS_W]),

           // Reading port
           .r_en(enB_reg),
           .r_addr(addrB_reg),
           .data_out(weights[`nYOLOvect*DATAPATH_W-DATAPATH_W*i-1 -: DATAPATH_W])
           );

      end
   endgenerate

   // register mem outputs
   always @ (posedge clk)
      weights_reg <= weights;
   assign flow_out_weight = weights_reg;

   // common internal addrgen
   xaddrgen addrgenB (
      .clk(clk),
      .rst(rst),
      .init(run_reg),
      .run(run_reg & |iterB_shadow),
      .pause(1'b0),
      .iterations(iterB_shadow),
      .period(perB_shadow),
      .duty(perB_shadow),
      .start(startB_shadow),
      .shift(shiftB_shadow),
      .incr(incrB_shadow),
      .delay(`PERIOD_W'd0),
      .addr(addrB),
      .mem_en(enB),
      .done(doneB)
   );

   //
   // Merge
   //

   //instantiate merge
   merge # (
      .N_MASTERS(`nYOLOvect),
      .DATA_W(DATA_W),
      .ADDR_W(ADDR_W)
   ) xyolo_read_merge (
      //masters interface
      .m_req(m_req),
      .m_resp(m_resp),
      //slave interface
      .s_req(s_req),
      .s_resp(s_resp)
   );

   //unconcatenate merge slave interface back to native interface
   assign databus_addr = s_req[`address(0, `IO_ADDR_W)];
   assign databus_wdata = s_req[`wdata(0)];
   assign databus_wstrb = s_req[`wstrb(0)];
   assign databus_valid = s_req[`valid(0)];
   assign s_resp[`rdata(0)] = databus_rdata;
   assign s_resp[`ready(0)] = databus_ready;

endmodule
