`timescale 1ns/1ps

module mul_4stage # (
		parameter		DATA_W = 32,
		parameter		SIGNED = 1
	) (
                //control
                input                   clk,
                input                   ld_acc,

                //data
                input [DATA_W-1:0]	inA,
                input [DATA_W-1:0]	inB,
                input [2*DATA_W-1:0]	inC,
                output [2*DATA_W-1:0]	out
                );

 generate
 if(SIGNED) begin

   //multiplier wires and regs
   reg signed [DATA_W-1:0] 		op_a, op_b;
   reg signed [2*DATA_W-1:0] 		acc, dsp_out, op_c, result_mult;
   wire signed [2*DATA_W-1:0] 		acc_w;

   //4-stage multiplier (DSP48E2 template)
   always @ (posedge clk) begin
     op_a <= inA;
     op_b <= inB;
     result_mult <= op_a * op_b;
     op_c <= inC;
     acc <= acc_w + result_mult;
     dsp_out <= acc;
   end
   assign acc_w = ld_acc ? op_c : acc;
   assign out = dsp_out;

 end else begin

   //multiplier wires and regs
   reg [DATA_W-1:0] 			op_a, op_b;
   reg [2*DATA_W-1:0] 			acc, dsp_out, op_c, result_mult;
   wire [2*DATA_W-1:0] 			acc_w;

   //4-stage multiplier (DSP48E2 template)
   always @ (posedge clk) begin
     op_a <= inA;
     op_b <= inB;
     result_mult <= op_a * op_b;
     op_c <= inC;
     acc <= acc_w + result_mult;
     dsp_out <= acc;
   end
   assign acc_w = ld_acc ? op_c : acc;
   assign out = dsp_out;

 end
 endgenerate

endmodule
