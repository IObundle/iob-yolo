`timescale 1ns / 1ps

`include "xversat.vh"

// Log2 number of states
`define STATES_W 2

// FSM States
`define IDLE       `STATES_W'h0
`define EXT2INT    `STATES_W'h1
`define INT2EXT    `STATES_W'h2

module ext_addrgen #(
    parameter 			     DATA_W = 32,
    parameter			     EXT_ADDR_W = 10,
    parameter			     EXT_PERIOD_W = 10,
    parameter                        MEM_ADDR_W = 10
    ) (
    input 			    clk,
    input 			    rst,

    // Control I/F
    input 			    run,
    output reg 			    done,

    // Configuration
    input [`IO_ADDR_W-1:0] 	    ext_addr,
    input [MEM_ADDR_W-1:0] 	    int_addr,
    input [1:0] 		    direction,
    input [EXT_PERIOD_W - 1:0] 	    iterations,
    input [EXT_PERIOD_W - 1:0] 	    period,
    input [EXT_PERIOD_W - 1:0] 	    duty,
    input [EXT_PERIOD_W - 1:0] 	    delay,
    input [EXT_ADDR_W - 1:0] 	    start,
    input signed [EXT_ADDR_W - 1:0] shift,
    input signed [EXT_ADDR_W - 1:0] incr,

    // Databus interface
    input 			    databus_ready,
    output reg 			    databus_valid,
    output [`IO_ADDR_W-1:0] 	    databus_addr,
    input [DATA_W-1:0] 		    databus_rdata,
    output [DATA_W-1:0] 	    databus_wdata,
    output reg [DATA_W/8-1:0] 	    databus_wstrb,

    // internal memory interface
    output reg 			    valid,
    output reg 			    we,
    output [MEM_ADDR_W-1:0] 	    addr,
    output [DATA_W-1:0] 	    data_out,
    input [DATA_W-1:0] 		    data_in
   );

   // addr gen wires
   wire [EXT_ADDR_W-1:0]             addr_gen;
   wire                              ready = |iterations;
   wire                              mem_en;

   // int mem regs
   reg [MEM_ADDR_W-1:0]              counter_int, counter_int_nxt;

   // link int mem and ext databus
   assign data_out = databus_rdata;
   assign databus_wdata = data_in;
   assign databus_addr = ext_addr + (addr_gen<<1);
   assign addr = int_addr + counter_int;

   // instantiate addrgen for ext mem access
   xaddrgen # (
      .MEM_ADDR_W(EXT_ADDR_W),
      .PERIOD_W(EXT_PERIOD_W)
   ) addrgen (
      .clk(clk),
      .rst(rst),
      .init(run),
      .run(run & ready),
      .pause(~databus_ready & databus_valid),
      .iterations(iterations),
      .period(period),
      .duty(duty),
      .delay(delay),
      .start(start),
      .shift(shift),
      .incr(incr),
      .addr(addr_gen),
      .mem_en(mem_en),
      .done()
   );

   // State and counter registers
   reg [`STATES_W-1:0] state, state_nxt;
   always @ (posedge rst, posedge clk)
     if (rst) begin
	state <= `IDLE;
        counter_int <= {MEM_ADDR_W{1'b0}};
     end else begin
        state <= state_nxt;
        counter_int <= counter_int_nxt;
     end

   // State machine
   always @ * begin
      state_nxt = state;
      done = 1'b1;
      valid = 1'b0;
      we = 1'b0;
      databus_valid = 1'b0;
      databus_wstrb = {(DATA_W/8){1'b0}};
      counter_int_nxt = counter_int;
      case (state)
	    `IDLE: begin
               counter_int_nxt = {MEM_ADDR_W{1'b0}};
	       if (run) begin
                  if (direction == 2'b01)
		     state_nxt = `EXT2INT;
	          else if (direction == 2'b10)
		     state_nxt = `INT2EXT;
	       end
	    end
	    `EXT2INT: begin
	       if (mem_en == 1'b0)
	          state_nxt = `IDLE;
	       else begin
                  databus_valid = 1'b1;
	          done = 1'b0;
	          if (databus_ready) begin
                     valid = 1'b1;
	             we = 1'b1;
                     counter_int_nxt += 1'b1;
	          end
	       end
	    end
	    `INT2EXT: begin
	       if (mem_en == 1'b0)
	          state_nxt = `IDLE;
	       else begin
                  valid = 1'b1;
                  databus_valid = 1'b1;
                  databus_wstrb = {(DATA_W/8){1'b1}};
	          done = 1'b0;
                  if (databus_ready)
                     counter_int_nxt += 1'b1;
	       end
	    end
        default: begin
           state_nxt = `IDLE;
        end
      endcase
   end

endmodule
