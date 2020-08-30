`timescale 1ns / 1ps

`include "xversat.vh"
`include "xyolo_write.vh"
`include "interconnect.vh"

module xyolo_write_stage #(
    	parameter                       DATAPATH_W = 32,
        parameter			DATABUS_W = 256
    ) (
    	input 					     clk,
    	input 					     rst,

	// control
    	input 					     global_run,
    	output 					     done,

        // internal addrgen
        input 					     vread_enB,
        input [`nYOLOvect-1:0] 			     vwrite_enB,
        input [`PIXEL_INT_ADDR_W-1:0] 		     vread_addrB,
        input [`VWRITE_ADDR_W-1:0] 		     vwrite_addrB,

        // load control
        input 					     ld_acc,
        input 					     ld_mp,
        input 					     ld_res,

        // vread config params
        input [`IO_ADDR_W-1:0] 			     vread_ext_addr,
        input [`PIXEL_W_ADDR_W-1:0] 		     vread_int_addr,
        input [`EXT_ADDR_W-1:0] 		     vread_iterA,
        input [`EXT_ADDR_W-1:0] 		     vread_perA,
        input [`EXT_ADDR_W-1:0] 		     vread_shiftA,
        input [`EXT_ADDR_W-1:0] 		     vread_incrA,

        // vwrite config params
        input [`IO_ADDR_W-1:0] 			     vwrite_ext_addr,
        input [`VWRITE_ADDR_W-1:0] 		     vwrite_int_addr,
        input [`PIXEL_ADDR_W-1:0] 		     vwrite_iterA,
        input [`PIXEL_ADDR_W-1:0] 		     vwrite_perA,
        input [`PIXEL_ADDR_W-1:0] 		     vwrite_shiftA,
        input [`PIXEL_ADDR_W-1:0] 		     vwrite_incrA,

        // xyolo config params
	input 					     xyolo_bias,
	input 					     xyolo_leaky,
	input 					     xyolo_maxpool,
	input 					     xyolo_bypass,
	input [`SHIFT_W-1:0] 			     xyolo_shift,

    	// databus interface (1 vread + 1 vwrite)
    	input [1:0] 				     databus_ready,
    	output [1:0] 				     databus_valid,
    	output [2*`IO_ADDR_W-1:0] 		     databus_addr,
    	input [2*DATABUS_W-1:0] 		     databus_rdata,
    	output [2*DATABUS_W-1:0] 		     databus_wdata,
    	output [2*DATABUS_W/8-1:0] 		     databus_wstrb,

    	// input data
    	input [`nYOLOvect*DATAPATH_W-1:0] 	     flow_in_bias,
	input [`nYOLOvect*`nYOLOmacs*DATAPATH_W-1:0] flow_in_weight
    );

   // external addrgen wires and regs
   wire					vread_enA, vread_we, vwrite_enA;
   reg					vread_enA_reg, vread_we_reg;
   wire [`PIXEL_W_ADDR_W-1:0]		vread_addrA;
   reg [`PIXEL_W_ADDR_W-1:0]		vread_addrA_reg;
   wire [`VWRITE_ADDR_W-1:0]            vwrite_addrA;
   wire [DATABUS_W-1:0]      		vread_inA;
   reg [DATABUS_W-1:0]      		vread_inA_reg;
   wire [`nYOLOvect*DATAPATH_W-1:0]	vwrite_inA, vwrite_inB;
   reg [`nYOLOvect*DATAPATH_W-1:0]	vwrite_inB_reg;

   // done output
   wire					vread_doneA, vwrite_doneA;
   assign                               done = &{vread_doneA, vwrite_doneA};

   // vread output
   wire [`nYOLOmacs*DATAPATH_W-1:0] 	pixel, vread_out;
   reg [`nYOLOmacs*DATAPATH_W-1:0] 	vread_out_reg;

   //
   // global vread
   //

   //external address generator
   ext_addrgen #(
   	.DATA_W(DATABUS_W),
	.EXT_ADDR_W(`EXT_ADDR_W),
	.EXT_PERIOD_W(`EXT_ADDR_W),
	.MEM_ADDR_W(`PIXEL_W_ADDR_W)
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
      .delay(`EXT_ADDR_W'd0),
      // Databus interface
      .databus_ready(databus_ready[0]),
      .databus_valid(databus_valid[0]),
      .databus_addr(databus_addr[`IO_ADDR_W-1:0]),
      .databus_rdata(databus_rdata[DATABUS_W-1:0]),
      .databus_wdata(databus_wdata[DATABUS_W-1:0]),
      .databus_wstrb(databus_wstrb[DATABUS_W/8-1:0]),
      // internal memory interface
      .valid(vread_enA),
      .we(vread_we),
      .addr(vread_addrA),
      .data_out(vread_inA),
      .data_in({DATABUS_W{1'b0}})
   );

   //register vread mem write inputs
   always @ (posedge clk) begin
      vread_enA_reg <= vread_enA;
      vread_we_reg <= vread_we;
      vread_addrA_reg <= vread_addrA;
      vread_inA_reg <= vread_inA;
   end

   //internal memory
   iob_2p_assim_mem_w_big #(
      .W_DATA_W(DATABUS_W),
      .W_ADDR_W(`PIXEL_W_ADDR_W),
      .R_DATA_W(`nYOLOmacs*DATAPATH_W),
      .R_ADDR_W(`PIXEL_INT_ADDR_W),
      .USE_RAM(1)
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

   //external address generator
   ext_addrgen #(
      .DATA_W(DATABUS_W),
      .EXT_ADDR_W(`PIXEL_ADDR_W),
      .EXT_PERIOD_W(`PIXEL_ADDR_W),
      .MEM_ADDR_W(`VWRITE_ADDR_W)
   ) vwrite_addrgenA (
      .clk(clk),
      .rst(rst),
      // Control
      .run(global_run),
      .int_cnt_en(1'b1),
      .done(vwrite_doneA),
      // Configuration
      .ext_addr(vwrite_ext_addr),
      .int_addr(vwrite_int_addr),
      .direction(2'b10),
      .iterations(vwrite_iterA),
      .period(vwrite_perA),
      .duty(vwrite_perA),
      .start({`PIXEL_ADDR_W{1'd0}}),
      .shift(vwrite_shiftA),
      .incr(vwrite_incrA),
      .delay({`PIXEL_ADDR_W{1'd0}}),
      // Databus interface
      .databus_ready(databus_ready[1]),
      .databus_valid(databus_valid[1]),
      .databus_addr(databus_addr[2*`IO_ADDR_W-1:`IO_ADDR_W]),
      .databus_rdata(databus_rdata[2*DATABUS_W-1:DATABUS_W]),
      .databus_wdata(databus_wdata[2*DATABUS_W-1:DATABUS_W]),
      .databus_wstrb(databus_wstrb[2*DATABUS_W/8-1:DATABUS_W/8]),
      // internal memory interface
      .valid(vwrite_enA),
      .we(),
      .addr(vwrite_addrA),
      .data_out(),
      .data_in(vwrite_inA)
   );

   // register data between xyolos and vwrites
   always @ (posedge clk, posedge rst)
      if(rst)
         vwrite_inB_reg <= {`nYOLOvect*DATAPATH_W{1'b0}};
      else
	 vwrite_inB_reg <= vwrite_inB;

   // instantiate vwrite internal memories and xyolo units
   genvar i;
   generate
      for (i=0; i < `nYOLOvect; i=i+1) begin : vwrite_array

	 //internal memory
         iob_2p_mem #(
            .DATA_W(DATAPATH_W),
            .ADDR_W(`VWRITE_ADDR_W),
            .USE_RAM(0)
         ) vwrite_mem (
            .clk(clk),
            // Reading port
            .r_en(vwrite_enA),
            .r_addr(vwrite_addrA),
            .data_out(vwrite_inA[DATAPATH_W*i +: DATAPATH_W]),
            // Writting port
            .w_en(vwrite_enB[i]),
            .w_addr(vwrite_addrB),
            .data_in(vwrite_inB_reg[`nYOLOvect*DATAPATH_W-DATAPATH_W*i-1 -: DATAPATH_W])
         );

	 //xyolo
	 xyolo #(
	    .DATAPATH_W(DATAPATH_W),
	    .N_MACS(`nYOLOmacs)
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
	    .flow_in_weight(flow_in_weight[`nYOLOvect*`nYOLOmacs*DATAPATH_W-`nYOLOmacs*DATAPATH_W*i-1 -: `nYOLOmacs*DATAPATH_W]),
	    .flow_in_bias(flow_in_bias[`nYOLOvect*DATAPATH_W-DATAPATH_W*i-1 -: DATAPATH_W]),
	    .flow_out(vwrite_inB[`nYOLOvect*DATAPATH_W-DATAPATH_W*i-1 -: DATAPATH_W])
	 );

      end
   endgenerate

endmodule
