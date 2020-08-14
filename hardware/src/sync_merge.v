`timescale 1ns / 1ps
`include "interconnect.vh"

module sync_merge
  #(
    parameter N_MASTERS = 2,
    parameter DATA_W = 32,
    parameter ADDR_W = 32
    )
   (

    //inputs
    input			       clk,
    input			       rst,

    //masters interface
    input [N_MASTERS*`REQ_W-1:0]       m_req,
    output reg [N_MASTERS*`RESP_W-1:0] m_resp,

    //slave interface
    output reg [`REQ_W-1:0]            s_req,
    input [`RESP_W-1:0]                s_resp
    );

   integer                             i, j;
   reg [$clog2(N_MASTERS)-1:0]         ptr;

   //store current prioritized master
   always @ (posedge clk, posedge rst)
      if(rst)
         ptr <= {$clog2(N_MASTERS){1'b0}};
      else
         ptr <= j[$clog2(N_MASTERS)-1:0];

   //priority encoder: most significant bus has priority   
   always @* begin
      s_req = {`REQ_W{1'b0}};
      m_resp = {N_MASTERS*`RESP_W{1'b0}};
      j = 0;
      for (i = 0; i < N_MASTERS; i++)
        if(m_req[`valid(i)] && (!m_req[`valid(ptr)] || i == ptr)) begin //test valid bit 
           s_req = m_req[`req(i)];
           m_resp = {N_MASTERS*`RESP_W{1'b0}};
           m_resp[`resp(i)] = s_resp;
	   j = i;
        end
   end

endmodule
