`timescale 1ns / 1ps
`include "xversat.vh"

/* 6-LOOP ADDRESS GENERATOR
  
    loops 5/6: iter3, per3, incr3, shift3
      loops 3/4: iter2, per2, incr2, shift2
        loops 1/2: iter, per, incr, shift
 
*/

module xaddrgen3 # (
		 parameter				MEM_ADDR_W = 10,
		 parameter				PERIOD_W = 10
		) (
		 input                                  clk,
		 input                                  rst,
		 input                                  run,

		 //global configurations
		 input [PERIOD_W - 1:0]                 duty,
		 input [PERIOD_W - 1:0]                 delay,
		 input [MEM_ADDR_W - 1:0]              	start,

		 //loops 1-2
		 input [PERIOD_W - 1:0]              	iterations,
		 input [PERIOD_W - 1:0]                	period,
		 input signed [MEM_ADDR_W - 1:0]       	shift,
		 input signed [MEM_ADDR_W - 1:0]       	incr,

		 //loops 3-4 
		 input [PERIOD_W - 1:0]              	iterations2,
		 input [PERIOD_W - 1:0]                	period2,
		 input signed [MEM_ADDR_W - 1:0]       	shift2,
		 input signed [MEM_ADDR_W - 1:0]       	incr2,

		 //loops 5-6
		 input [PERIOD_W - 1:0]              	iterations3,
		 input [PERIOD_W - 1:0]                	period3,
		 input signed [MEM_ADDR_W - 1:0]       	shift3,
		 input signed [MEM_ADDR_W - 1:0]       	incr3,

		 //outputs
		 output reg [MEM_ADDR_W - 1:0]         	addr,
		 output reg                             mem_en,
		 output reg                             done
		 );

   //connection wires
   wire [MEM_ADDR_W - 1:0]                             	addrB, addrC;
   wire                                                 mem_enB, mem_enC;
   wire                                                 doneA, doneB, doneC;
   wire                                                 mem_en_reg_w;

   //only run if iterations > 0
   wire                                                 readyA = |iterations;
   wire                                                 readyB = |iterations2;
   wire                                                 readyC = |iterations3;

   //keep running addrgenB/A while addrgenC/B is enabled
   wire                                                 runA = run | mem_enB;
   wire                                                 runB = run | mem_enC;

   //done if all addrgen are done
   assign                                               done = doneA & (doneB | ~readyB) & (doneC | ~readyC);

   //update addrgenC/B after addrgenB/A is done
   wire                                                 pauseB = mem_en_reg_w & ~doneA;
   wire                                                 pauseC = ~doneB;

   //after first run, addrgenB/A start comes from addrgenC/B addr
   wire	[MEM_ADDR_W - 1:0]				startA = run ? start : addrB;
   wire	[MEM_ADDR_W - 1:0]				startB = run ? start : addrC;

   //instantiate address generators
   xaddrgen # ( 
	.MEM_ADDR_W(MEM_ADDR_W),
	.PERIOD_W(PERIOD_W)
   ) addrgenA (
        .clk(clk),
        .rst(rst),
        .init(runA),
        .run(runA & readyA),
        .pause(1'b0),
        .iterations(iterations),
        .period(period),
        .duty(duty),
        .delay(delay),
        .start(startA),
        .shift(shift),
        .incr(incr),
        .addr(addr),
        .mem_en(mem_en),
        .done(doneA)
        );

   xaddrgen # ( 
	.MEM_ADDR_W(MEM_ADDR_W),
	.PERIOD_W(PERIOD_W)
   ) addrgenB (
        .clk(clk),
        .rst(rst),
        .init(runB),
        .run(runB & readyB),
        .pause(pauseB),
        .iterations(iterations2),
        .period(period2),
        .duty(period2),
        .delay(delay),
        .start(startB),
        .shift(shift2),
        .incr(incr2),
        .addr(addrB),
        .mem_en(mem_enB),
        .done(doneB)
        );

   xaddrgen # (
	.MEM_ADDR_W(MEM_ADDR_W),
	.PERIOD_W(PERIOD_W)
   ) addrgenC (
        .clk(clk),
        .rst(rst),
        .init(run),
        .run(run & readyC),
        .pause(pauseC),
        .iterations(iterations3),
        .period(period3),
        .duty(period3),
        .delay(delay),
        .start(start),
        .shift(shift3),
        .incr(incr3),
        .addr(addrC),
        .mem_en(mem_enC),
        .done(doneC)
        );

   //register mem_en of addrgenA
   reg mem_en_reg;
   always @ (posedge clk, posedge rst)
     if(rst)
       mem_en_reg <= 1'b0;
     else
       mem_en_reg <= mem_en;
   assign mem_en_reg_w = mem_en_reg;

endmodule
