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
                input                   maxpool,
                input                   bypass,
		input [`SHIFT_W-1:0]	shift,

                //data interface
                input [DATAPATH_W-1:0]	flow_in_pixel,  //op_a
                input [DATAPATH_W-1:0]	flow_in_weight, //op_b
                input [DATAPATH_W-1:0]	flow_in_bias,   //op_c
                output [DATAPATH_W-1:0]	flow_out
                );

   //data interface wires and regs
   wire [2*DATAPATH_W-1:0]              shifted, adder;
   wire signed [DATAPATH_W-1:0]         act_fnc, shifted_half, mp_w, bypass_w;
   reg signed [DATAPATH_W-1:0]          bias_reg, result, op_a_bypass;
   wire [DATAPATH_W-1:0]                result_w;

   //multiplier wires and regs
   wire signed [2*DATAPATH_W-1:0] 	dsp_out, adder_w;

   //update registers
   always @ (posedge clk, posedge rst)
     if (rst) begin
       bias_reg <= {DATAPATH_W{1'b0}};
       op_a_bypass <= {DATAPATH_W{1'b0}};
       result <= {DATAPATH_W{1'b0}};
     end else begin
       bias_reg <= flow_in_bias;
       op_a_bypass <= flow_in_pixel;
       if(ld_res) result <= result_w;
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

   //activation function
   assign shifted = dsp_out >> shift;
   assign shifted_half = shifted[DATAPATH_W-1:0];
   assign act_fnc = (leaky & shifted_half[DATAPATH_W-1]) ? shifted_half >>> 3 : shifted_half;

   //maxpooling
   assign bypass_w = bypass ? op_a_bypass : act_fnc;
   assign mp_w = ld_mp & result > bypass_w ? result : bypass_w;

   //result
   assign result_w = maxpool ? mp_w : bypass_w;
   assign flow_out = result;

endmodule
