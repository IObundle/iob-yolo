`timescale 1ns/1ps

`include "xyolo_write.vh"

module xyolo # (
		parameter		DATAPATH_W = 32
	) (
                input                   clk,
                input                   rst,

		//load control
		input 			ld_acc,
		input 			ld_mp,
		input			ld_res,

		//configuration
		input                   bias,
                input                   leaky,
                input                   sigmoid,
                input                   maxpool,
                input                   bypass,
		input [`SHIFT_W-1:0]	shift,

                //data interface
                input [DATAPATH_W-1:0]	flow_in_pixel,  //op_a
                input [DATAPATH_W-1:0]	flow_in_weight, //op_b
                input [DATAPATH_W-1:0]	flow_in_bias,   //op_c
                output [DATAPATH_W-1:0]	flow_out
                );

   //local parameters for sigmoid linear approximation
   localparam [DATAPATH_W-1:0]		fp5 = 'h500;
   localparam [DATAPATH_W-1:0]		fp2375 = 'h260;
   localparam [DATAPATH_W-1:0]		fp1 = 'h100;
   localparam [DATAPATH_W-1:0]		fp084375 = 'hD8;
   localparam [DATAPATH_W-1:0]		fp0625 = 'hA0;
   localparam [DATAPATH_W-1:0]		fp05 = 'h80;

   //data interface wires and regs
   wire [2*DATAPATH_W-1:0]              shifted, adder;
   wire signed [DATAPATH_W-1:0]         act_fnc, shifted_half, mp_w, bypass_w;
   reg signed [DATAPATH_W-1:0]          bias_reg, result, op_a_bypass;
   wire [DATAPATH_W-1:0]                result_w;

   //activation function wires
   wire signed [DATAPATH_W-1:0]		sig_in, sig_out, sig_adder;
   reg signed [DATAPATH_W-1:0]		sig_t1, sig_t2, sig_out_r, shift_r;
   wire signed [DATAPATH_W-1:0]		leaky_out;

   //multiplier wires and regs
   wire signed [2*DATAPATH_W-1:0] 	dsp_out, adder_w;

   //update registers
   always @ (posedge clk, posedge rst)
     if (rst) begin
       bias_reg <= {DATAPATH_W{1'b0}};
       op_a_bypass <= {DATAPATH_W{1'b0}};
       result <= {DATAPATH_W{1'b0}};
       sig_out_r <= {DATAPATH_W{1'b0}};
       shift_r <= {DATAPATH_W{1'b0}};
     end else begin
       bias_reg <= flow_in_bias;
       op_a_bypass <= flow_in_pixel;
       if(ld_res) result <= result_w;
       sig_out_r <= sig_out;
       shift_r <= shifted_half;
     end

   //double-precision bias
   assign adder = {{DATAPATH_W{1'b0}}, bias_reg};

   //4-stage multiplier (DSP48E2 template)
   mul_4stage # (
     .DATA_W(DATAPATH_W)
   ) mul (
     //control
     .clk(clk),
     .ld_acc(ld_acc),
     //data
     .inA(flow_in_pixel),
     .inB(flow_in_weight),
     .inC(adder_w),
     .out(dsp_out)
   );

   //select accumulation initial value
   assign adder_w = bias ? adder << shift : {2*DATAPATH_W{1'b0}};

   //apply shift to half precision
   assign shifted = dsp_out >> shift;
   assign shifted_half = shifted[DATAPATH_W-1:0];

   //leaky activation function
   assign leaky_out = shifted_half[DATAPATH_W-1] ? shifted_half >>> 3 : shifted_half;
   
   //sigmoid activation function
   assign sig_in = shifted_half[DATAPATH_W-1] ? ~shifted_half + 1'b1 : shifted_half;
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
   assign sig_adder = sig_t1 + sig_t2;
   assign sig_out = shifted_half[DATAPATH_W-1] ? fp1 - sig_adder : sig_adder;

   //choose activation function
   assign act_fnc = leaky ? leaky_out : sigmoid ? sig_out_r : shift_r;

   //maxpooling
   assign bypass_w = bypass ? op_a_bypass : act_fnc;
   assign mp_w = ld_mp & result > bypass_w ? result : bypass_w;

   //result
   assign result_w = maxpool ? mp_w : bypass_w;
   assign flow_out = result;

endmodule
