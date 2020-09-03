`timescale 1ns/1ps

// Module to be used to add all dsp_outputs
// Only the output is registered
// Output has same size as inputs - carefull with overflow

module adder_N # (
		  parameter DATA_W = 32,
		  parameter N_INPUTS = 2,
		  parameter N_LEVELS = $clog2(N_INPUTS)
		  ) (
		     input 			 clk,
		     input 			 rst,
		     // data
		     input [N_INPUTS*DATA_W-1:0] data_in,
		     output reg [DATA_W-1:0] 	 data_out
		     );

   // aux wire
   wire signed [(2*N_INPUTS-1)*DATA_W-1:0] 	 part_sum;
   
   //assign input
   assign part_sum[0 +: N_INPUTS*DATA_W] = data_in; 
   
   // generate all tree levels
   genvar 					 i;
   generate
      if (N_LEVELS == 0) begin : no_tree
	 assign part_sum[(2*N_INPUTS-1)*DATA_W-1 -: DATA_W] = data_in; 
      end
      else begin
	 for(i=0;i<N_LEVELS;i=i+1) begin : adder_tree
	    localparam LINE_ADDERS = N_INPUTS/(2**(i+1)); // Number of adders in i-th line
	    localparam ACC_SUMS = 2*N_INPUTS-4*LINE_ADDERS; // Number of partial sums done before i-th line
	    	    
	    adder_line #(
			 .DATA_W(DATA_W),
			 .N_ADDERS(LINE_ADDERS)
			 ) adder_tree_line (
					    .data_in(part_sum[ACC_SUMS*DATA_W +: LINE_ADDERS*2*DATA_W]),
					    .data_out(part_sum[(ACC_SUMS+2*LINE_ADDERS)*DATA_W +: LINE_ADDERS*DATA_W])
					    );
	 end
      end
   endgenerate

   // register output
   always @(posedge clk, rst)
     if(rst)
       data_out <= {DATA_W{1'b0}};
     else
       data_out <= part_sum[(2*N_INPUTS-1)*DATA_W-1 -: DATA_W];
   
endmodule // adder_N

module adder_line # (
		     parameter DATA_W = 32,
		     parameter N_ADDERS = 2
		     ) (
			//data
			input [2*N_ADDERS*DATA_W-1:0] data_in,
			output [N_ADDERS*DATA_W-1:0]     data_out
			);

   // generate an adder line
   genvar 						 i;
   generate
      for(i=0;i<N_ADDERS;i=i+1) begin : adder_l
      adder2_1 #(
		 .DATA_W(DATA_W)
		 ) adder2_1_inst (
				  .data_in(data_in[i*2*DATA_W +: 2*DATA_W]),
				  .data_out(data_out[i*DATA_W +: DATA_W])
				  );
      end
   endgenerate   

endmodule // adder_line

module adder2_1 #(
		  parameter DATA_W = 32
		  ) (
		     //data
		     input [2*DATA_W-1:0] data_in,
		     output [DATA_W-1:0]  data_out
		     );

   assign data_out = data_in[0*DATA_W +: DATA_W] + data_in[1*DATA_W +: DATA_W];
   
endmodule // adder2_1   
