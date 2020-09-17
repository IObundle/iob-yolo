`timescale 1ns/1ps

`include "xyolo_write.vh"

module xyolo # (
		parameter		      DATAPATH_W = 32,
		parameter                     N_MACS = 1,
		parameter		      INDEX = 0,
   		parameter                     N_MACS_W = $clog2(N_MACS)+($clog2(N_MACS)==0)
	) (
                input 			      clk,
                input 			      rst,

		//load control
		input 			      ld_acc,
		input 			      ld_mp,
		input 			      ld_res,
		input [N_MACS_W-1:0] 	      ld_nmac,
	   
		//configuration
		input                         bias,
                input                         leaky,
                input                         sigmoid,
                input                         maxpool,
                input                         bypass,
                input                         bypass_adder,
		input [`SHIFT_W-1:0]	      shift,
		input [`SHIFT_W-1:0]	      b_shift,
		input [`SHIFT_W-1:0]	      sig_shift,

                //data interface
                input [N_MACS*DATAPATH_W-1:0] flow_in_pixel, //op_a
		input [N_MACS*DATAPATH_W-1:0] flow_in_weight, //op_b
                input [DATAPATH_W-1:0]        flow_in_bias, //op_c
                output [DATAPATH_W-1:0]       flow_out
                );

   //wires for sigmoid linear approximation
   wire [2*DATAPATH_W-1:0]		fp5 = 'h500 << sig_shift;
   wire [2*DATAPATH_W-1:0]		fp2375 = 'h260 << sig_shift;
   wire [2*DATAPATH_W-1:0]		fp1 = 'h100 << sig_shift;
   wire [2*DATAPATH_W-1:0]		fp084375 = 'hD8 << sig_shift;
   wire [2*DATAPATH_W-1:0]		fp0625 = 'hA0 << sig_shift;
   wire [2*DATAPATH_W-1:0]		fp05 = 'h80 << sig_shift;

   //data interface wires and regs
   wire signed [2*DATAPATH_W-1:0]       shifted, bias_shifted;
   wire signed [DATAPATH_W-1:0]         shifted_half, mp_w, bypass_w;
   reg signed [DATAPATH_W-1:0]          result, op_a_bypass;
   reg signed [2*DATAPATH_W-1:0] 	bias_reg;
   wire [DATAPATH_W-1:0] 		result_w;
   reg [DATAPATH_W-1:0] 		op_a_bypass_nmac;

   //activation function wires
   wire signed [2*DATAPATH_W-1:0]	sig_in, sig_out, sig_adder;
   reg signed [2*DATAPATH_W-1:0]	sig_t1, sig_t1_r, sig_t2, sig_t2_r, sig_out_r;
   wire signed [2*DATAPATH_W-1:0]	leaky_in, leaky_out, act_fnc;
   reg signed [2*DATAPATH_W-1:0]	conv_r, conv_r2, leaky_out_r;

   //multiplier wires and regs
   wire signed [N_MACS*2*DATAPATH_W-1:0] dsp_out, adder_w;
   wire signed [2*DATAPATH_W-1:0] 	 conv_res;

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
       bias_reg <= {2*DATAPATH_W{1'b0}};
       op_a_bypass <= {DATAPATH_W{1'b0}};
       result <= {DATAPATH_W{1'b0}};
       leaky_out_r <= {2*DATAPATH_W{1'b0}};
       sig_out_r <= {2*DATAPATH_W{1'b0}};
       conv_r <= {2*DATAPATH_W{1'b0}};
       conv_r2 <= {2*DATAPATH_W{1'b0}};
       sig_t1_r <= {2*DATAPATH_W{1'b0}};
       sig_t2_r <= {2*DATAPATH_W{1'b0}};
     end else begin
       bias_reg <= {flow_in_bias, `DATAPATH_W'b0};
       op_a_bypass <= op_a_bypass_nmac;
       if(ld_res) result <= bypass_adder ? dsp_out[INDEX*2*DATAPATH_W +: 2*DATAPATH_W] >> shift : result_w;
       leaky_out_r <= leaky_out;
       sig_out_r <= sig_out;
       conv_r <= conv_res;
       conv_r2 <= conv_r;
       sig_t1_r <= sig_t1;
       sig_t2_r <= sig_t2;
     end

   //double-precision bias
   assign bias_shifted = bias_reg >>> b_shift;

   genvar i;
   generate
      for(i=0;i<N_MACS;i=i+1) begin : macs

	 if (i==0) begin : bias_blk
	    //select accumulation initial value - add only bias to first mac
	    assign adder_w[i*2*DATAPATH_W +: 2*DATAPATH_W] = bias ? bias_shifted : {2*DATAPATH_W{1'b0}};
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

   // add dsp outputs
   adder_N # (
	      .DATA_W(2*DATAPATH_W),
	      .N_INPUTS(N_MACS)
	      ) dsp_adder (
			   .clk(clk),
			   .rst(rst),
			   // data
			   .data_in(dsp_out),
			   .data_out(conv_res)
			   );

   //leaky activation function
   assign leaky_in = (conv_res >>> 4) + (conv_res >>> 5) + (conv_res >>> 7);
   assign leaky_out = conv_res[2*DATAPATH_W-1] ? leaky_in : conv_res;

   //sigmoid activation function
   assign sig_in = conv_res[2*DATAPATH_W-1] ? ~conv_res + 1'b1 : conv_res;
   always @ * begin
      if(sig_in >= fp5) begin
         sig_t1 = fp1;
         sig_t2 = 0;
      end else if(sig_in >= fp2375) begin
         sig_t1 = fp084375;
         sig_t2 = sig_in >> 5;
      end else if(sig_in >= fp1) begin
         sig_t1 = fp0625;
         sig_t2 = sig_in >> 3;
      end else begin
         sig_t1 = fp05;
         sig_t2 = sig_in >> 2;
      end
   end
   assign sig_adder = sig_t1_r + sig_t2_r;
   assign sig_out = conv_r[2*DATAPATH_W-1] ? fp1 - sig_adder : sig_adder;

   //choose activation function
   assign act_fnc = leaky ? leaky_out_r : sigmoid ? sig_out_r : conv_r2;
   assign shifted = act_fnc >>> shift;
   assign shifted_half = shifted[DATAPATH_W-1:0];

   //maxpooling
   assign bypass_w = bypass ? op_a_bypass : shifted_half;
   assign mp_w = ld_mp & result > bypass_w ? result : bypass_w;

   //result
   assign result_w = maxpool ? mp_w : bypass_w;
   assign flow_out = result;

endmodule
