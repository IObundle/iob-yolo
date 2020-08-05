`timescale 1ns / 1ps

`include "xversat.vh"
`include "xyolo_write.vh"
`include "interconnect.vh"

module xyolo_write_stage #(
    	parameter                       DATA_W = 32
    ) (
    	input                           clk,
    	input                           rst,

	// control
    	input                           global_run,
    	output                          done,

        // internal addrgen
        input                           vread_enB,
        input                           vwrite_enB,
        input [`MEM_ADDR_W-1:0]         vread_addrB,
        input [`VWRITE_ADDR_W-1:0]      vwrite_addrB,

        // load control
        input                           ld_acc,
        input                           ld_mp,
        input                           ld_res,

        // vread config params
        input [`IO_ADDR_W-1:0]          vread_ext_addr,
        input [`MEM_ADDR_W-1:0]         vread_int_addr,
        input [`EXT_ADDR_W-1:0]         vread_iterA,
        input [`EXT_PERIOD_W-1:0]       vread_perA,
        input [`EXT_ADDR_W-1:0]         vread_shiftA,
        input [`EXT_ADDR_W-1:0]         vread_incrA,

        // vwrite config params
        input [`IO_ADDR_W-1:0]          vwrite_ext_addr,
        input [`VWRITE_ADDR_W-1:0]      vwrite_int_addr,
        input [`MEM_ADDR_W-1:0]         vwrite_iterA,
        input [`PERIOD_W-1:0]           vwrite_perA,
        input [`MEM_ADDR_W-1:0]         vwrite_shiftA,
        input [`MEM_ADDR_W-1:0]         vwrite_incrA,
        input                           vwrite_bypass,

        // xyolo config params
	input                           xyolo_bias,
	input                           xyolo_leaky,
	input                           xyolo_maxpool,
	input                           xyolo_bypass,
	input [`SHIFT_W-1:0]            xyolo_shift,

    	// Databus interface
    	input                           databus_ready,
    	output                      	databus_valid,
    	output [`IO_ADDR_W-1:0]       	databus_addr,
    	input [DATA_W-1:0]              databus_rdata,
    	output [DATA_W-1:0]             databus_wdata,
    	output [DATA_W/8-1:0]           databus_wstrb,

    	// input data
    	input [`nYOLOvect*DATA_W-1:0]   flow_in_bias,
    	input [`nYOLOvect*DATA_W-1:0]	flow_in_weight
    );

   // local parameter for merge
   localparam				ADDR_W = `IO_ADDR_W;

   // external addrgen wires and regs
   wire					vread_enA, vread_we, vwrite_enA;
   reg					vread_enA_reg, vread_we_reg;
   wire [`MEM_ADDR_W-1:0]		vread_addrA;
   reg [`MEM_ADDR_W-1:0]		vread_addrA_reg;
   wire [`VWRITE_ADDR_W-1:0]            vwrite_addrA;
   wire [DATA_W-1:0]      		vread_inA;
   reg [DATA_W-1:0]      		vread_inA_reg;
   wire [`nYOLOvect*DATA_W-1:0]		vwrite_inA, vwrite_inB;

   // done output
   wire					vread_doneA, vwrite_doneA;
   assign                               done = &{vread_doneA, vwrite_doneA};

   // vread output
   wire	[DATA_W-1:0]			pixel, vread_out;
   reg	[DATA_W-1:0]			vread_out_reg;

   // vwrite counter and mux
   reg [$clog2(`nYOLOvect)-1:0]         vwrite_cnt;
   reg                                  vwrite_cnt_en;
   reg [DATA_W-1:0]                     vwrite_mux;

   // merge master interface
   wire [2*`REQ_W-1:0]   	        m_req;
   wire [2*`RESP_W-1:0]                 m_resp;

   //merge slave interface
   wire [`REQ_W-1:0]                    s_req;
   wire [`RESP_W-1:0]                   s_resp;

   //
   // global vread
   //

   //external address generator
   ext_addrgen #(
   	.DATA_W(DATA_W),
	.EXT_ADDR_W(`EXT_ADDR_W),
	.EXT_PERIOD_W(`EXT_PERIOD_W)
   ) vread_addrgenA (
      .clk(clk),
      .rst(rst),
      // Control
      .run(global_run),
      .int_cnt_en(1'b1),
      .done(vread_doneA),
      // Configuration
      .ext_addr(vread_ext_addr),
      .int_addr(vread_int_addr),
      .direction(2'b01),
      .iterations(vread_iterA),
      .period(vread_perA),
      .duty(vread_perA),
      .start(`EXT_ADDR_W'd0),
      .shift(vread_shiftA),
      .incr(vread_incrA),
      .delay(`EXT_PERIOD_W'd0),
      // Databus interface
      .databus_ready(m_resp[`ready(0)]),
      .databus_valid(m_req[`valid(0)]),
      .databus_addr(m_req[`address(0, `IO_ADDR_W)]),
      .databus_rdata(m_resp[`rdata(0)]),
      .databus_wdata(m_req[`wdata(0)]),
      .databus_wstrb(m_req[`wstrb(0)]),
      // internal memory interface
      .valid(vread_enA),
      .we(vread_we),
      .addr(vread_addrA),
      .data_out(vread_inA),
      .data_in({DATA_W{1'b0}})
   );

   //register vread mem write inputs
   always @ (posedge clk) begin
      vread_enA_reg <= vread_enA;
      vread_we_reg <= vread_we;
      vread_addrA_reg <= vread_addrA;
      vread_inA_reg <= vread_inA;
   end

   //internal memory
   iob_2p_mem #(
      .DATA_W(DATA_W),
      .ADDR_W(`MEM_ADDR_W),
      .USE_RAM(0)
   ) vread_mem (
       .clk(clk),
       // Writting port
       .w_en(vread_enA_reg & vread_we_reg),
       .w_addr(vread_addrA_reg),
       .data_in(vread_inA_reg),
       // Reading port
       .r_en(vread_enB),
       .r_addr(vread_addrB),
       .data_out(vread_out)
   );

   //register vread mem read output
   always @ (posedge clk)
      vread_out_reg <= vread_out;
   assign pixel = vread_out_reg;

   //
   // vwrite/xyolo vector
   //

   //vwrite counter - to address each vwrite mem sequentially
   always @ (posedge clk, posedge global_run)
      if (global_run) begin
         vwrite_cnt <= {$clog2(`nYOLOvect){1'b0}};
         vwrite_cnt_en <= 1'b0;
      end else if(vwrite_bypass)
         vwrite_cnt_en <= 1'b1;
      else if(m_resp[`ready(1)]) begin
         if(vwrite_cnt == `nYOLOvect-1) begin
            vwrite_cnt <= {$clog2(`nYOLOvect){1'b0}};
            vwrite_cnt_en <= 1'b0;
         end else begin
            vwrite_cnt <= vwrite_cnt + 1;
            if(vwrite_cnt == `nYOLOvect-2)
               vwrite_cnt_en <= 1'b1;
            else
               vwrite_cnt_en <= 1'b0;
         end
      end

   //vwrite mux - select which memory to address
   always @ * begin
      integer j;
      vwrite_mux = {DATA_W{1'b0}};
      for(j = 0; j < `nYOLOvect; j++)
         if(vwrite_cnt == j)
            vwrite_mux = vwrite_inA[`nYOLOvect*DATA_W-DATA_W*j-1 -: DATA_W];
   end

   //external address generator
   ext_addrgen #(
      .DATA_W(DATA_W),
      .MEM_ADDR_W(`VWRITE_ADDR_W)
   ) vwrite_addrgenA (
      .clk(clk),
      .rst(rst),
      // Control
      .run(global_run),
      .int_cnt_en(vwrite_cnt_en),
      .done(vwrite_doneA),
      // Configuration
      .ext_addr(vwrite_ext_addr),
      .int_addr(vwrite_int_addr),
      .direction(2'b10),
      .iterations(vwrite_iterA),
      .period(vwrite_perA),
      .duty(vwrite_perA),
      .start(`MEM_ADDR_W'd0),
      .shift(vwrite_shiftA),
      .incr(vwrite_incrA),
      .delay(`PERIOD_W'd0),
      // Databus interface
      .databus_ready(m_resp[`ready(1)]),
      .databus_valid(m_req[`valid(1)]),
      .databus_addr(m_req[`address(1, `IO_ADDR_W)]),
      .databus_rdata(m_resp[`rdata(1)]),
      .databus_wdata(m_req[`wdata(1)]),
      .databus_wstrb(m_req[`wstrb(1)]),
      // internal memory interface
      .valid(vwrite_enA),
      .we(),
      .addr(vwrite_addrA),
      .data_out(),
      .data_in(vwrite_mux)
   );

   // instantiate vwrite internal memories and xyolo units
   genvar i;
   generate
      for (i=0; i < `nYOLOvect; i=i+1) begin : vwrite_array

	 //internal memory
         iob_2p_mem #(
            .DATA_W(DATA_W),
            .ADDR_W(`VWRITE_ADDR_W),
            .USE_RAM(0)
         ) vwrite_mem (
            .clk(clk),
            // Reading port
            .r_en(vwrite_enA),
            .r_addr(vwrite_addrA),
            .data_out(vwrite_inA[`nYOLOvect*DATA_W-DATA_W*i-1 -: DATA_W]),
            // Writting port
            .w_en(vwrite_enB),
            .w_addr(vwrite_addrB),
            .data_in(vwrite_inB[`nYOLOvect*DATA_W-DATA_W*i-1 -: DATA_W])
         );

	 //xyolo
	 xyolo #(
	    .DATA_W(DATA_W)
	 ) xyolo (
	    .clk(clk),
	    .rst(global_run),
	    //load control
	    .ld_acc(ld_acc),
	    .ld_mp(ld_mp),
	    .ld_res(ld_res),
	    //configuration
	    .bias(xyolo_bias),
	    .leaky(xyolo_leaky),
	    .maxpool(xyolo_maxpool),
	    .bypass(xyolo_bypass),
	    .shift(xyolo_shift),
	    //data interface
	    .flow_in_pixel(pixel),
	    .flow_in_weight(flow_in_weight[`nYOLOvect*DATA_W-DATA_W*i-1 -: DATA_W]),
	    .flow_in_bias(flow_in_bias[`nYOLOvect*DATA_W-DATA_W*i-1 -: DATA_W]),
	    .flow_out(vwrite_inB[`nYOLOvect*DATA_W-DATA_W*i-1 -: DATA_W])
	 );

      end
   endgenerate

   //
   // merge
   //

   //instantiate merge
   merge #(
      .N_MASTERS(2),
      .DATA_W(DATA_W),
      .ADDR_W(ADDR_W)
   ) xyolo_write_merge (
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
