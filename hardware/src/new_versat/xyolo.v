`timescale 1ns/1ps

`include "xyolo_write.vh"

module xyolo # (
		parameter		DATAPATH_W = 32,
		parameter               N_MACS = 1,
		parameter               N_MACS_W = $clog2(N_MACS)+($clog2(N_MACS)==0)
	) (
                input 			      clk,
                input 			      rst,

		//load control
		input 			      ld_acc,
		input 			      ld_mp,
		input 			      ld_res,
		input [N_MACS_W-1:0] 	      ld_nmac,
	   
		//configuration
		input 			      bias,
                input 			      leaky,
                input 			      maxpool,
                input 			      bypass,
		input [`SHIFT_W-1:0] 	      shift,

                //data interface
                input [N_MACS*DATAPATH_W-1:0] flow_in_pixel, //op_a
		input [N_MACS*DATAPATH_W-1:0] flow_in_weight, //op_b
                input [DATAPATH_W-1:0] 	      flow_in_bias, //op_c
                output [DATAPATH_W-1:0]       flow_out
                );

   //data interface wires and regs
   wire [2*DATAPATH_W-1:0]              shifted, adder;
   wire signed [DATAPATH_W-1:0]         act_fnc, shifted_half, mp_w, bypass_w;
   reg signed [DATAPATH_W-1:0]          bias_reg, result;
   reg signed [DATAPATH_W-1:0] 		op_a_bypass;
   wire [DATAPATH_W-1:0] 		result_w;
   reg [DATAPATH_W-1:0] 		op_a_bypass_nmac;

   //multiplier wires and regs
   wire signed [N_MACS*2*DATAPATH_W-1:0] dsp_out, adder_w;
   reg signed [2*DATAPATH_W-1:0] conv_res;

   // Mux to select pixel for bypass
   integer 			 k;
   always @* begin
      op_a_bypass_nmac = flow_in_pixel[0 +: DATAPATH_W];
      for (k=0;k<N_MACS;k=k+1)
	if (k == ld_nmac)
	  op_a_bypass_nmac = flow_in_pixel[k*DATAPATH_W +: DATAPATH_W];
   end
      
   //update registers
   always @ (posedge clk, posedge rst)
     if (rst) begin
       bias_reg <= {DATAPATH_W{1'b0}};
       op_a_bypass <= {N_MACS*DATAPATH_W{1'b0}};
       result <= {DATAPATH_W{1'b0}};
     end else begin
	bias_reg <= flow_in_bias;
	op_a_bypass <= op_a_bypass_nmac;
	if(ld_res) result <= result_w;
     end

   //double-precision bias
   assign adder = {{DATAPATH_W{1'b0}}, bias_reg};

   genvar i;
   generate
      for(i=0;i<N_MACS;i=i+1) begin : macs

	 if (i==0) begin : bias_blk
	    //select accumulation initial value - add only bias to first mac
	    assign adder_w[i*2*DATAPATH_W +: 2*DATAPATH_W] = bias ? adder << shift : {2*DATAPATH_W{1'b0}};
	 end else begin : bias_blk
	    //accumulation initial value - always 0
	    assign adder_w[i*2*DATAPATH_W +: 2*DATAPATH_W] = {2*DATAPATH_W{1'b0}};
	 end
	 //4-stage multiplier (DSP48E2 template)
	 mul_4stage # (
		       .DATA_W(DATAPATH_W)
		       ) mul (
			      //control
			      .clk(clk),
			      .ld_acc(ld_acc),
			      //data
			      .inA(flow_in_pixel[i*DATAPATH_W +: DATAPATH_W]),
			      .inB(flow_in_weight[i*DATAPATH_W +: DATAPATH_W]),
			      .inC(adder_w[i*2*DATAPATH_W +: 2*DATAPATH_W]),
			      .out(dsp_out[i*2*DATAPATH_W +: 2*DATAPATH_W])
			      );	 
      end
   endgenerate
   
   // Adder tree
   generate
      case(N_MACS)
   	1 : begin
   	   wire signed [2*DATAPATH_W-1:0] part_sum;
   	   // nothing to add
   	   assign part_sum = dsp_out[0 +: 2*DATAPATH_W];
   	   // register result
   	   always @ (posedge clk, posedge rst)
   	     if (rst) begin
   		conv_res <= {2*DATAPATH_W{1'b0}};
   	     end else begin
   		conv_res <= part_sum[2*DATAPATH_W-1 -: 2*DATAPATH_W];
   	     end
   	end
   	2 : begin
   	   wire signed [2*DATAPATH_W-1:0] part_sum;
	   
   	   // level 0
   	   assign part_sum[2*DATAPATH_W-1 -: 2*DATAPATH_W] = dsp_out[0*2*DATAPATH_W +: 2*DATAPATH_W] + dsp_out[1*2*DATAPATH_W +: 2*DATAPATH_W];
   	   // register result
   	   always @ (posedge clk, posedge rst)
   	     if (rst) begin
   		conv_res <= {2*DATAPATH_W{1'b0}};
   	     end else begin
   		conv_res <= part_sum[2*DATAPATH_W-1 -: 2*DATAPATH_W];
   	     end
   	end
   	4: begin
   	   wire signed [2*DATAPATH_W-1:0] part_sum;

   	   assign part_sum[2*DATAPATH_W-1 -: 2*DATAPATH_W] = dsp_out[0*2*DATAPATH_W +: 2*DATAPATH_W] + dsp_out[1*2*DATAPATH_W +: 2*DATAPATH_W] + dsp_out[2*2*DATAPATH_W +: 2*DATAPATH_W] + dsp_out[3*2*DATAPATH_W +: 2*DATAPATH_W];

   	   // register result
   	   always @ (posedge clk, posedge rst)
   	     if (rst) begin
   		conv_res <= {2*DATAPATH_W{1'b0}};
   	     end else begin
   		conv_res <= part_sum[2*DATAPATH_W-1 -: 2*DATAPATH_W];
   	     end
   	end
   	default : begin
   	   wire signed [2*DATAPATH_W-1:0] part_sum;

   	   // nothing to add
   	   assign part_sum = dsp_out[0 +: 2*DATAPATH_W];
   	   // register result
   	   always @ (posedge clk, posedge rst)
   	     if (rst) begin
   		conv_res <= {2*DATAPATH_W{1'b0}};
   	     end else begin
   		conv_res <= part_sum[2*DATAPATH_W-1 -: 2*DATAPATH_W];
   	     end
   	end
      endcase
   endgenerate

   //activation function
   assign shifted = conv_res >> shift;
   assign shifted_half = shifted[DATAPATH_W-1:0];
   assign act_fnc = (leaky & shifted_half[DATAPATH_W-1]) ? shifted_half >>> 3 : shifted_half;

   //maxpooling
   assign bypass_w = bypass ? op_a_bypass : act_fnc;
   assign mp_w = ld_mp & result > bypass_w ? result : bypass_w;

   //result
   assign result_w = maxpool ? mp_w : bypass_w;
   assign flow_out = result;

endmodule
