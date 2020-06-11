`timescale 1ns / 1ps
`include "xversat.vh"
`include "xmemdefs.vh"

module xmem #(
              parameter MEM_INIT_FILE="none",
	      parameter DATA_W = 32
              )
    (
    //control
    input                         clk,
    input                         rst,
    input                         run,
    output                        done,

    //mem interface
    input                         we,
    input [`MEM_ADDR_W-1:0]       addr,
    input [DATA_W-1:0]            rdata,
    input                         valid,

    //input / output data
    input [2*`DATABUS_W-1:0]      flow_in,
    output [DATA_W-1:0]           flow_out,

    //configurations
    input [`MEMP_CONF_BITS-1:0]   config_bits
    );

   //output databus
   wire [DATA_W-1:0]              out;
   reg  [DATA_W-1:0]              out_reg;
   assign flow_out = out_reg;

   //unpack configuration bits
   wire [`MEM_ADDR_W-1:0] iter    = config_bits[`MEMP_CONF_BITS-1 -: `MEM_ADDR_W];
   wire [`PERIOD_W-1:0]   per     = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-1 -: `PERIOD_W];
   wire [`PERIOD_W-1:0]   duty    = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-`PERIOD_W-1 -: `PERIOD_W];
   wire [`N_W-1:0]        sel     = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-1 -: `N_W];
   wire [`MEM_ADDR_W-1:0] start   = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-1 -: `MEM_ADDR_W];
   wire [`MEM_ADDR_W-1:0] shift   = config_bits[`MEMP_CONF_BITS-2*`MEM_ADDR_W-2*`PERIOD_W-`N_W-1 -: `MEM_ADDR_W];
   wire [`MEM_ADDR_W-1:0] incr    = config_bits[`MEMP_CONF_BITS-3*`MEM_ADDR_W-2*`PERIOD_W-`N_W-1 -: `MEM_ADDR_W];
   wire [`PERIOD_W-1:0]   delay   = config_bits[`MEMP_CONF_BITS-4*`MEM_ADDR_W-2*`PERIOD_W-`N_W-1 -: `PERIOD_W];
   wire                   ext     = config_bits[`MEMP_CONF_BITS-4*`MEM_ADDR_W-3*`PERIOD_W-`N_W-1 -: 1];
   wire                   in_wr   = config_bits[`MEMP_CONF_BITS-4*`MEM_ADDR_W-3*`PERIOD_W-`N_W-1-1 -: 1];
   wire [`MEM_ADDR_W-1:0] iter2   = config_bits[`MEMP_CONF_BITS-4*`MEM_ADDR_W-3*`PERIOD_W-`N_W-2-1 -: `MEM_ADDR_W];
   wire [`PERIOD_W-1:0]   per2    = config_bits[`MEMP_CONF_BITS-5*`MEM_ADDR_W-3*`PERIOD_W-`N_W-2-1 -: `PERIOD_W];
   wire [`MEM_ADDR_W-1:0] shift2  = config_bits[`MEMP_CONF_BITS-5*`MEM_ADDR_W-4*`PERIOD_W-`N_W-2-1 -: `MEM_ADDR_W];
   wire [`MEM_ADDR_W-1:0] incr2   = config_bits[`MEMP_CONF_BITS-6*`MEM_ADDR_W-4*`PERIOD_W-`N_W-2-1 -: `MEM_ADDR_W];
   wire [`MEM_ADDR_W-1:0] iter3   = config_bits[`MEMP_CONF_BITS-7*`MEM_ADDR_W-4*`PERIOD_W-`N_W-2-1 -: `MEM_ADDR_W];
   wire [`PERIOD_W-1:0]   per3    = config_bits[`MEMP_CONF_BITS-8*`MEM_ADDR_W-4*`PERIOD_W-`N_W-2-1 -: `PERIOD_W];
   wire [`MEM_ADDR_W-1:0] shift3  = config_bits[`MEMP_CONF_BITS-8*`MEM_ADDR_W-5*`PERIOD_W-`N_W-2-1 -: `MEM_ADDR_W];
   wire [`MEM_ADDR_W-1:0] incr3   = config_bits[`MEMP_CONF_BITS-9*`MEM_ADDR_W-5*`PERIOD_W-`N_W-2-1 -: `MEM_ADDR_W];

   //mem enable output by addr gen
   wire en_int;
   wire en = en_int | valid;

   //write enable
   wire wr = valid ? we : (en_int & in_wr & ~ext); //addrgen on & input on & input isn't address

   //port addresses
   wire [`MEM_ADDR_W-1:0] addr_w, addr_int;

   //data inputs
   wire [DATA_W-1:0]     in;

   //input selection
   xinmux # (
	.DATA_W(DATA_W)
   ) mux (
	.sel(sel),
	.data_in(flow_in),
	.data_out(in)
	);
   wire [DATA_W-1:0]     data_to_wr = valid ? rdata : in;

   //address generators
   xaddrgen3 addrgen3 (
		      .clk(clk),
		      .rst(rst),
		      .run(run),
		      .iterations(iter),
		      .period(per),
		      .duty(duty),
		      .start(start),
		      .shift(shift),
		      .incr(incr),
		      .delay(delay),
		      .iterations2(iter2),
                      .period2(per2),
                      .shift2(shift2),
                      .incr2(incr2),
		      .iterations3(iter3),
                      .period3(per3),
                      .shift3(shift3),
                      .incr3(incr3),
		      .addr(addr_int),
		      .mem_en(en_int),
		      .done(done)
		      );

   //define addresses based on ext and rvrs
   assign addr_w = valid ? addr[`MEM_ADDR_W-1:0] : ext ? in : addr_int[`MEM_ADDR_W-1:0];

   //register mem inputs
   reg [DATA_W-1:0] data_reg;
   reg [`MEM_ADDR_W-1:0] addr_reg;
   reg en_reg, we_reg;
   always @ (posedge clk) begin
      data_reg <= data_to_wr;
      addr_reg <= addr_w;
      en_reg <= en;
      we_reg <= wr;
   end

   iob_1p_mem #(
                   .FILE(MEM_INIT_FILE),
		   .DATA_W(DATA_W),
		   .ADDR_W(`MEM_ADDR_W))
   mem
     (
      .data_in(data_reg),
      .addr(addr_reg),
      .en(en_reg),
      .we(we_reg),
      .data_out(out),
      .clk(clk)
      );

   //register mem output
   always @ (posedge clk)
      out_reg <= out;

endmodule
