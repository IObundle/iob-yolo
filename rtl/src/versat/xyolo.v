`timescale 1ns/1ps
`include "xversat.vh"

module xyolo # (
		parameter		      DATA_W = 32
	) (
                input                         rst,
                input                         clk,
                input			      addrgen_rst,

                //flow interface
                input [2*`DATABUS_W-1:0]      flow_in,
                output [DATA_W-1:0] 	      flow_out,

                // config interface
                input [`YOLO_CONF_BITS-1:0]   configdata
                );

   //data
   reg signed [2*DATA_W-1:0]                  int_acc;
   wire [2*DATA_W-1:0]                        shifted, adder;
   wire signed [DATA_W-1:0]                   op_a, op_b, op_c, act_fnc, shifted_half;
   wire [`MEM_ADDR_W-1:0]                     op_o;
   reg signed [DATA_W-1:0]                    bias_reg, result, op_a_bypass;
   wire [DATA_W-1:0]                          result_w;

   //config data
   wire [`N_W-1:0]                            sela, selb, selc;
   wire [`MEM_ADDR_W-1:0]		      iterations;
   wire [`PERIOD_W-1:0]                       period, delay;
   wire [`SHIFT_W-1:0]			      shift;
   wire                                       bias, leaky, maxpool, bypass, accumulator;

   //accumulator load signal
   wire                                       ld_acc;
   reg                                        ld_acc0, ld_acc1, ld_acc2, ld_acc3;

   //maxpool data
   reg [1:0]                                  mp_cnt;
   wire signed [DATA_W-1:0]                   mp_w, bypass_w;

   //unpack config bits
   assign sela = configdata[`YOLO_CONF_BITS-1 -: `N_W];
   assign selb = configdata[`YOLO_CONF_BITS-1-`N_W -: `N_W];
   assign selc = configdata[`YOLO_CONF_BITS-1-2*`N_W -: `N_W];
   assign iterations = configdata[`YOLO_CONF_BITS-1-3*`N_W -: `MEM_ADDR_W];
   assign period = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W -: `PERIOD_W];
   assign delay = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W-`PERIOD_W -: `PERIOD_W];
   assign shift = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W-2*`PERIOD_W -: `SHIFT_W];
   assign bias = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W-2*`PERIOD_W-`SHIFT_W -: 1];
   assign leaky = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W-2*`PERIOD_W-`SHIFT_W-1 -: 1];
   assign maxpool = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W-2*`PERIOD_W-`SHIFT_W-2 -: 1];
   assign bypass = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W-2*`PERIOD_W-`SHIFT_W-3 -: 1];
   assign accumulator = configdata[`YOLO_CONF_BITS-1-3*`N_W-`MEM_ADDR_W-2*`PERIOD_W-`SHIFT_W-4 -: 1];

   //input selection
   xinmux # (
	.DATA_W(DATA_W)
   ) muxa (
        .sel(sela),
        .data_in(flow_in),
        .data_out(op_a)
 	);

   xinmux # (
	.DATA_W(DATA_W)
   ) muxb (
        .sel(selb),
        .data_in(flow_in),
        .data_out(op_b)
	);

   xinmux # (
	.DATA_W(DATA_W)
   ) muxc (
        .sel(selc),
        .data_in(flow_in),
        .data_out(op_c)
	);

   //addr_gen to control macc
   wire ready = |iterations;
   wire mem_en, done;
   xaddrgen addrgen (
	.clk(clk),
	.rst(addrgen_rst),
	.init(rst),
	.run(rst & ready),
	.pause(1'b0),
	.iterations(iterations),
	.period(period),
	.duty(period),
	.start(`MEM_ADDR_W'b0),
	.shift(-period[`MEM_ADDR_W-1:0]),
	.incr(`MEM_ADDR_W'b1),
	.delay(delay),
	.addr(op_o),
	.mem_en(mem_en),
	.done(done)
	);

   //update registers
   always @ (posedge clk, posedge rst) begin
     if (rst) begin
       ld_acc0 <= 1'b0;
       ld_acc1 <= 1'b0;
       ld_acc2 <= 1'b0;
       ld_acc3 <= 1'b0;
       bias_reg <= {DATA_W{1'b0}};
       op_a_bypass <= {DATA_W{1'b0}};
     end else begin
       ld_acc0 <= ld_acc;
       ld_acc1 <= ld_acc0;
       ld_acc2 <= ld_acc1;
       ld_acc3 <= ld_acc2;
       bias_reg <= op_c;
       op_a_bypass <= op_a;
     end
   end

   //double-precision bias
   assign adder = {{DATA_W{1'b0}}, bias_reg};

   //compute accumulator load signal
   assign ld_acc = (op_o=={`MEM_ADDR_W{1'd0}});

   //multiplier signals and regs
   reg signed [DATA_W-1:0] op_a_reg, op_b_reg;
   reg signed [2*DATA_W-1:0] acc, dsp_out, op_c_reg, result_mult;
   wire signed [2*DATA_W-1:0] acc_w, adder_w;

   //4-stage multiplier (DSP48E2 template)
   always @ (posedge clk) begin
     op_a_reg <= op_a;
     op_b_reg <= op_b;
     result_mult <= op_a_reg * op_b_reg;
     op_c_reg <= adder_w;
     acc <= acc_w + result_mult;
     dsp_out <= acc;
   end
   assign acc_w = ld_acc1 ? op_c_reg : acc;

   //select accumulation initial value
   assign adder_w = bias ? adder << shift : accumulator ? int_acc : {2*DATA_W{1'b0}};

   //activation function
   assign shifted = dsp_out >> shift;
   assign shifted_half = shifted[DATA_W-1:0];
   assign act_fnc = (leaky & shifted_half[DATA_W-1]) ? shifted_half >>> 3 : shifted_half;

   //maxpooling
   assign bypass_w = bypass ? op_a_bypass : act_fnc;
   always @ (posedge clk, posedge rst)
     if(rst) begin
       mp_cnt <= 2'd2;
       result <= {DATA_W{1'b0}};
     end else if(ld_acc2 || bypass) begin
       mp_cnt <= mp_cnt + 1;
       result <= result_w;
     end
   assign mp_w = |mp_cnt & result>bypass_w ? result : bypass_w;

   //internal accumulator
   always @ (posedge clk, posedge addrgen_rst)
     if(addrgen_rst)
       int_acc <= {2*DATA_W{1'b0}};
     else if(!ld_acc3 && mp_cnt==0)
       int_acc <= result_w;

   //result
   assign result_w = maxpool ? mp_w : bypass_w;
   assign flow_out = result;

endmodule
