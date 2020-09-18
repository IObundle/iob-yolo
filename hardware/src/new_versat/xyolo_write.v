`timescale 1ns / 1ps

`include "xversat.vh"
`include "xyolo_write.vh"
`include "xyolo_read.vh"

module xyolo_write #(
    	parameter                       DATAPATH_W = 32,
        parameter			DATABUS_W = 256,
	parameter                       N_MACS = `nYOLOmacs
    ) (
    	input                           clk,
    	input                           rst,

	// control
	input                           clear,
    	input                           run,
    	output                          done,

	// cpu interface (only request)
	input                           valid,
        input [`XYOLO_WRITE_ADDR_W-1:0] addr,
        input [`IO_ADDR_W-1:0]          wdata,
        input                           wstrb,

        // databus interface (1 vread merge + 1 vwrite merge)
        input [1:0]                     databus_ready,
        output [1:0]                    databus_valid,
        output [2*`IO_ADDR_W-1:0]       databus_addr,
        input [2*DATABUS_W-1:0]         databus_rdata,
        output [2*DATABUS_W-1:0]        databus_wdata,
        output [2*DATABUS_W/8-1:0]      databus_wstrb,

    	// input data
    	input [`nYOLOvect*DATAPATH_W-1:0] flow_in_bias,
    	input [`nYOLOvect*N_MACS*DATAPATH_W-1:0] flow_in_weight

    );

   // size of nYOLOmacs counter
   localparam                           N_MACS_W = $clog2(N_MACS)+($clog2(N_MACS)==0);
   
   // vread latency
   localparam [`PIXEL_ADDR_W-1:0]       vread_lat = `XYOLO_READ_LAT;

   // local parameter for merge
   localparam                           ADDR_W = `IO_ADDR_W;
   localparam				DATA_W = DATABUS_W;

   // vwrite configuration enables
   reg                                  vwrite_ext_addr_en;
   reg                                  vwrite_offset_en;
   reg                                  vwrite_int_addr_en;
   reg                                  vwrite_iterA_en;
   reg                                  vwrite_perA_en;
   reg                                  vwrite_shiftA_en;
   reg                                  vwrite_incrA_en;
   reg                                  vwrite_startB_en;
   reg                                  vwrite_dutyB_en;
   reg                                  vwrite_delayB_en;
   reg                                  vwrite_iterB_en;
   reg                                  vwrite_perB_en;
   reg                                  vwrite_shiftB_en;
   reg                                  vwrite_incrB_en;

   // vread configuration enables
   reg					vread_ext_addr_en;
   reg					vread_offset_en;
   reg					vread_pp_en;
   reg					vread_int_addr_en;
   reg					vread_iterA_en;
   reg                                  vread_perA_en;
   reg                                  vread_shiftA_en;
   reg                                  vread_incrA_en;
   reg                                  vread_startB_en;
   reg                                  vread_iterB_en;
   reg                                  vread_perB_en;
   reg                                  vread_shiftB_en;
   reg                                  vread_incrB_en;
   reg					vread_iter2B_en;
   reg                                  vread_per2B_en;
   reg                                  vread_shift2B_en;
   reg                                  vread_incr2B_en;
   reg					vread_iter3B_en;
   reg                                  vread_per3B_en;
   reg                                  vread_shift3B_en;
   reg                                  vread_incr3B_en;
   //ix
   reg					vread_ext_en;
   reg 					ix_ext_addr_en;
   reg 					ix_iterA_en;
   reg 					ix_perA_en;
   reg 					ix_incrA_en;

   // xyolo configuration enables
   reg					xyolo_iter_en;
   reg					xyolo_per_en;
   reg					xyolo_shift_en;
   reg					xyolo_b_shift_en;
   reg					xyolo_bias_en;
   reg					xyolo_leaky_en;
   reg					xyolo_sigmoid_en;
   reg					xyolo_sig_mask_en;
   reg					xyolo_maxpool_en;
   reg					xyolo_bypass_en;
   reg					xyolo_bypass_adder_en;

   // vwrite configuration parameters
   reg [`IO_ADDR_W-1:0]                 vwrite_ext_addr;
   reg [`nSTAGES*`IO_ADDR_W-1:0]        vwrite_ext_addr_shadow, vwrite_ext_addr_pip0, vwrite_ext_addr_pip1;
   reg [`IO_ADDR_W/2-1:0]               vwrite_offset;
   reg [`VWRITE_ADDR_W-1:0]             vwrite_int_addr, vwrite_int_addr_shadow, vwrite_int_addr_pip0, vwrite_int_addr_pip1;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_iterA, vwrite_iterA_shadow, vwrite_iterA_pip0, vwrite_iterA_pip1;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_perA, vwrite_perA_shadow, vwrite_perA_pip0, vwrite_perA_pip1;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_shiftA, vwrite_shiftA_shadow, vwrite_shiftA_pip0, vwrite_shiftA_pip1;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_incrA, vwrite_incrA_shadow, vwrite_incrA_pip0, vwrite_incrA_pip1;
   reg [`VWRITE_ADDR_W-1:0]             vwrite_startB, vwrite_startB_pip, vwrite_startB_shadow;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_dutyB, vwrite_dutyB_pip, vwrite_dutyB_shadow;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_delayB, vwrite_delayB_pip, vwrite_delayB_shadow;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_iterB, vwrite_iterB_pip, vwrite_iterB_shadow;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_perB, vwrite_perB_pip, vwrite_perB_shadow;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_shiftB, vwrite_shiftB_pip, vwrite_shiftB_shadow;
   reg [`PIXEL_ADDR_W-1:0]              vwrite_incrB, vwrite_incrB_pip, vwrite_incrB_shadow;
   reg					mask_shadow, mask_pip0, mask_pip1;

   // vread configuration parameters
   reg [`IO_ADDR_W-1:0]			vread_ext_addr;
   reg [`nSTAGES*`IO_ADDR_W-1:0]	vread_ext_addr_shadow;
   reg [`IO_ADDR_W/2-1:0]		vread_offset;
   reg 					vread_pp;
   reg [`PIXEL_W_ADDR_W-1:0]		vread_int_addr, vread_int_addr_shadow;
   reg [`EXT_ADDR_W-1:0]		vread_iterA, vread_iterA_shadow;
   reg [`EXT_ADDR_W-1:0]              	vread_perA, vread_perA_shadow;
   reg [`EXT_ADDR_W-1:0]                vread_shiftA, vread_shiftA_shadow;
   reg [`EXT_ADDR_W-1:0]                vread_incrA, vread_incrA_shadow;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_startB, vread_startB_pip, vread_startB_shadow;
   reg [`PIXEL_ADDR_W-1:0] 		vread_iterB, vread_iterB_pip, vread_iterB_shadow;
   reg [`PIXEL_ADDR_W-1:0] 		vread_perB, vread_perB_pip, vread_perB_shadow;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_shiftB, vread_shiftB_pip, vread_shiftB_shadow;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_incrB, vread_incrB_pip, vread_incrB_shadow;
   reg [`PIXEL_ADDR_W-1:0] 		vread_iter2B, vread_iter2B_pip, vread_iter2B_shadow;
   reg [`PIXEL_ADDR_W-1:0] 		vread_per2B, vread_per2B_pip, vread_per2B_shadow;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_shift2B, vread_shift2B_pip, vread_shift2B_shadow;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_incr2B, vread_incr2B_pip, vread_incr2B_shadow;
   reg [`PIXEL_ADDR_W-1:0]		vread_iter3B, vread_iter3B_pip, vread_iter3B_shadow;
   reg [`PIXEL_ADDR_W-1:0] 		vread_per3B, vread_per3B_pip, vread_per3B_shadow;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_shift3B, vread_shift3B_pip, vread_shift3B_shadow;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_incr3B, vread_incr3B_pip, vread_incr3B_shadow;
   //ix
   reg					vread_ext, vread_ext_pip, vread_ext_shadow;
   reg [`IO_ADDR_W-1:0]			ix_ext_addr, ix_ext_addr_shadow;
   reg [`IX_ADDR_W-1:0]			ix_iterA, ix_iterA_shadow;
   reg [`IX_ADDR_W-1:0]			ix_perA, ix_perA_shadow;
   reg [`IX_ADDR_W-1:0]			ix_incrA, ix_incrA_shadow;
   
   // xyolo configuration parameters
   reg [`PIXEL_ADDR_W-1:0]		xyolo_iter, xyolo_iter_pip, xyolo_iter_shadow;
   reg [`PIXEL_ADDR_W-1:0]		xyolo_per, xyolo_per_pip, xyolo_per_shadow;
   reg [`SHIFT_W-1:0]			xyolo_shift, xyolo_shift_pip, xyolo_shift_shadow;
   reg [`SHIFT_W-1:0]			xyolo_b_shift, xyolo_b_shift_pip, xyolo_b_shift_shadow;
   reg 					xyolo_bias, xyolo_bias_pip, xyolo_bias_shadow;
   reg					xyolo_leaky, xyolo_leaky_pip, xyolo_leaky_shadow;
   reg					xyolo_sigmoid, xyolo_sigmoid_pip, xyolo_sigmoid_shadow;
   reg [`nYOLOvect-1:0]			xyolo_sig_mask, xyolo_sig_mask_pip, xyolo_sig_mask_shadow;
   reg					xyolo_maxpool, xyolo_maxpool_pip, xyolo_maxpool_shadow;
   reg					xyolo_bypass, xyolo_bypass_pip, xyolo_bypass_shadow;
   reg					xyolo_bypass_adder, xyolo_bypass_adder_pip, xyolo_bypass_adder_shadow;
   reg [`PIXEL_ADDR_W-1:0]		xyolo_delay;

   // external address buses
   wire [`nSTAGES*`IO_ADDR_W-1:0]       vwrite_ext_addr_bus, vread_ext_addr_bus;

   // ix ext_addr gen wires
   wire					ix_enA, ix_we;
   wire [`IX_W_ADDR_W-1:0]		ix_addrA;
   wire [DATABUS_W-1:0]			ix_inA;
   wire [DATAPATH_W-1:0]		ix_out;

   // internal addrgen wires and regs
   wire                                 vread_enB, vwrite_enB;
   reg                                  vread_enB_reg, vwrite_enB_reg;
   wire [`PIXEL_ADDR_W-1:0]             vwrite_addrB, vwrite_addrB_mux;
   wire [`PIXEL_INT_ADDR_W-1:0] 	vread_addrB;
   reg [`PIXEL_INT_ADDR_W-1:0] 		vread_addrB_reg; 		
   reg [`PIXEL_ADDR_W-1:0]              vwrite_addrB_r0, vwrite_addrB_r1, vwrite_addrB_stage;
   wire                                 vread_doneB, vwrite_doneB;
   reg [$clog2(`nYOLOvect)+1:0] 	vwrite_enB_cnt; //+1 as maxpool is 2x2
   reg [`nYOLOvect-1:0]			vwrite_enB_stage, vwrite_enB_stage_reg;
   assign				vwrite_addrB_mux = xyolo_bypass_adder_shadow ? vwrite_addrB_r1 :  xyolo_bypass_shadow ? vwrite_addrB_r0 : vwrite_addrB;

   // done output
   wire [`nSTAGES-1:0]                  stages_done;
   wire					ix_done;
   assign                               done = &{vread_doneB, vwrite_doneB, stages_done, ix_done};

   // run signals
   reg                                  run_reg;

   // xyolo wires and regs
   wire [`PIXEL_ADDR_W-1:0]		xyolo_addr;
   wire                                 ld_acc, ld_mp, ld_res;
   reg                                  ld_acc0, ld_acc1, ld_acc2, ld_acc3, ld_acc4;
   reg [1:0]                            mp_cnt;
   reg [N_MACS_W-1:0] 			nmac_cnt; 			

   // merge master interface
   wire [(`nSTAGES+1)*`REQ_W-1:0]       vread_m_req;
   wire [`nSTAGES*`REQ_W-1:0]       	vwrite_m_req;
   wire [(`nSTAGES+1)*`RESP_W-1:0]      vread_m_resp;
   wire [`nSTAGES*`RESP_W-1:0]      	vwrite_m_resp;

   // merge slave interface
   wire [`REQ_W-1:0]                    vread_s_req, vwrite_s_req;
   wire [`RESP_W-1:0]                   vread_s_resp, vwrite_s_resp;

   // register run
   always @ (posedge clk, posedge rst)
      if(rst)
         run_reg <= 1'b0;
      else
         run_reg <= run;

   // addr decoder enable
   always @* begin
      //vwrite
      vwrite_ext_addr_en = 1'b0;
      vwrite_offset_en = 1'b0;
      vwrite_int_addr_en = 1'b0;
      vwrite_iterA_en = 1'b0;
      vwrite_perA_en = 1'b0;
      vwrite_shiftA_en = 1'b0;
      vwrite_incrA_en = 1'b0;
      vwrite_startB_en = 1'b0;
      vwrite_dutyB_en = 1'b0;
      vwrite_delayB_en = 1'b0;
      vwrite_iterB_en = 1'b0;
      vwrite_perB_en = 1'b0;
      vwrite_shiftB_en = 1'b0;
      vwrite_incrB_en = 1'b0;
      //vread
      vread_ext_addr_en = 1'b0;
      vread_offset_en = 1'b0;
      vread_pp_en = 1'b0;
      vread_int_addr_en = 1'b0;
      vread_iterA_en = 1'b0;
      vread_perA_en = 1'b0;
      vread_shiftA_en = 1'b0;
      vread_incrA_en = 1'b0;
      vread_startB_en = 1'b0;
      vread_iterB_en = 1'b0;
      vread_perB_en = 1'b0;
      vread_shiftB_en = 1'b0;
      vread_incrB_en = 1'b0;
      vread_iter2B_en = 1'b0;
      vread_per2B_en = 1'b0;
      vread_shift2B_en = 1'b0;
      vread_incr2B_en = 1'b0;
      vread_iter3B_en = 1'b0;
      vread_per3B_en = 1'b0;
      vread_shift3B_en = 1'b0;
      vread_incr3B_en = 1'b0;
      //ix
      vread_ext_en = 1'b0;
      ix_ext_addr_en = 1'b0;
      ix_iterA_en = 1'b0;
      ix_perA_en = 1'b0;
      ix_incrA_en = 1'b0;
      //xyolo
      xyolo_iter_en = 1'b0;
      xyolo_per_en = 1'b0;
      xyolo_shift_en = 1'b0;
      xyolo_b_shift_en = 1'b0;
      xyolo_bias_en = 1'b0;
      xyolo_leaky_en = 1'b0;
      xyolo_sigmoid_en = 1'b0;
      xyolo_sig_mask_en = 1'b0;
      xyolo_maxpool_en = 1'b0;
      xyolo_bypass_en = 1'b0;
      xyolo_bypass_adder_en = 1'b0;
      if(valid & wstrb)
         case(addr)
	    //vwrite
            `VWRITE_CONF_EXT_ADDR : vwrite_ext_addr_en = 1'b1;
            `VWRITE_CONF_OFFSET : vwrite_offset_en = 1'b1;
            `VWRITE_CONF_INT_ADDR : vwrite_int_addr_en = 1'b1;
            `VWRITE_CONF_ITER_A : vwrite_iterA_en = 1'b1;
            `VWRITE_CONF_PER_A: vwrite_perA_en = 1'b1;
            `VWRITE_CONF_SHIFT_A : vwrite_shiftA_en = 1'b1;
            `VWRITE_CONF_INCR_A : vwrite_incrA_en = 1'b1;
            `VWRITE_CONF_START_B : vwrite_startB_en = 1'b1;
            `VWRITE_CONF_DUTY_B : vwrite_dutyB_en = 1'b1;
            `VWRITE_CONF_DELAY_B : vwrite_delayB_en = 1'b1;
            `VWRITE_CONF_ITER_B : vwrite_iterB_en = 1'b1;
            `VWRITE_CONF_PER_B : vwrite_perB_en = 1'b1;
            `VWRITE_CONF_SHIFT_B : vwrite_shiftB_en = 1'b1;
            `VWRITE_CONF_INCR_B : vwrite_incrB_en = 1'b1;
	    //vread
	    `VREAD_CONF_EXT_ADDR : vread_ext_addr_en = 1'b1;
	    `VREAD_CONF_OFFSET : vread_offset_en = 1'b1;
	    `VREAD_CONF_PP : vread_pp_en = 1'b1;
	    `VREAD_CONF_INT_ADDR : vread_int_addr_en = 1'b1;
            `VREAD_CONF_ITER_A : vread_iterA_en = 1'b1;
            `VREAD_CONF_PER_A : vread_perA_en = 1'b1;
            `VREAD_CONF_SHIFT_A : vread_shiftA_en = 1'b1;
            `VREAD_CONF_INCR_A : vread_incrA_en = 1'b1;
            `VREAD_CONF_START_B : vread_startB_en = 1'b1;
            `VREAD_CONF_ITER_B : vread_iterB_en = 1'b1;
            `VREAD_CONF_PER_B : vread_perB_en = 1'b1;
            `VREAD_CONF_SHIFT_B : vread_shiftB_en = 1'b1;
            `VREAD_CONF_INCR_B : vread_incrB_en = 1'b1;
            `VREAD_CONF_ITER2_B : vread_iter2B_en = 1'b1;
            `VREAD_CONF_PER2_B : vread_per2B_en = 1'b1;
            `VREAD_CONF_SHIFT2_B : vread_shift2B_en = 1'b1;
            `VREAD_CONF_INCR2_B : vread_incr2B_en = 1'b1;
            `VREAD_CONF_ITER3_B : vread_iter3B_en = 1'b1;
            `VREAD_CONF_PER3_B : vread_per3B_en = 1'b1;
            `VREAD_CONF_SHIFT3_B : vread_shift3B_en = 1'b1;
            `VREAD_CONF_INCR3_B : vread_incr3B_en = 1'b1;
      	    //ix
      	    `VREAD_CONF_EXT : vread_ext_en = 1'b1;
      	    `IX_CONF_EXT_ADDR : ix_ext_addr_en = 1'b1;
            `IX_CONF_ITER_A : ix_iterA_en = 1'b1;
	    `IX_CONF_PER_A : ix_perA_en = 1'b1;
	    `IX_CONF_INCR_A : ix_incrA_en = 1'b1;
	    //xyolo
	    `XYOLO_CONF_ITER : xyolo_iter_en = 1'b1;
	    `XYOLO_CONF_PER : xyolo_per_en = 1'b1;
	    `XYOLO_CONF_SHIFT : xyolo_shift_en = 1'b1;
	    `XYOLO_CONF_B_SHIFT : xyolo_b_shift_en = 1'b1;
	    `XYOLO_CONF_BIAS : xyolo_bias_en = 1'b1;
	    `XYOLO_CONF_LEAKY : xyolo_leaky_en = 1'b1;
	    `XYOLO_CONF_SIGMOID : xyolo_sigmoid_en = 1'b1;
	    `XYOLO_CONF_SIG_MASK : xyolo_sig_mask_en = 1'b1;
            `XYOLO_CONF_MAXPOOL : xyolo_maxpool_en = 1'b1;
	    `XYOLO_CONF_BYPASS : xyolo_bypass_en = 1'b1;
	    `XYOLO_CONF_BYPASS_ADD : xyolo_bypass_adder_en = 1'b1;
            default : ;
         endcase
   end

   // addr decoder parameters
   always @(posedge clk, posedge clear, posedge rst)
      if(clear || rst) begin
	 //vwrite
         vwrite_ext_addr <= `IO_ADDR_W'b0;
         vwrite_offset <= {`IO_ADDR_W/2{1'b0}};
         vwrite_int_addr <= `VWRITE_ADDR_W'b0;
         vwrite_iterA <= `PIXEL_ADDR_W'b0;
         vwrite_perA <= `PIXEL_ADDR_W'b0;
         vwrite_shiftA <= `PIXEL_ADDR_W'b0;
         vwrite_incrA <= `PIXEL_ADDR_W'b0;
         vwrite_startB <= `VWRITE_ADDR_W'b0;
         vwrite_dutyB <= `PIXEL_ADDR_W'b0;
         vwrite_delayB <= `PIXEL_ADDR_W'b0;
         vwrite_iterB <= `PIXEL_ADDR_W'b0;
         vwrite_perB <= `PIXEL_ADDR_W'b0;
         vwrite_shiftB <= `PIXEL_ADDR_W'b0;
         vwrite_incrB <= `PIXEL_ADDR_W'b0;
	 //vread
	 vread_ext_addr <= `IO_ADDR_W'b0;
	 vread_offset <= {`IO_ADDR_W/2{1'b0}};
	 vread_pp <= 1'b0;
         vread_int_addr <= {`PIXEL_W_ADDR_W{1'b0}};
         vread_iterA <= `EXT_ADDR_W'b0;
         vread_perA <= `EXT_ADDR_W'b0;
	 vread_shiftA <= `EXT_ADDR_W'b0;
	 vread_incrA <= `EXT_ADDR_W'b0;
	 vread_startB <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_iterB <= {`PIXEL_ADDR_W{1'b0}};
	 vread_perB <= `PIXEL_ADDR_W'b0;
	 vread_shiftB <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incrB <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_iter2B <= {`PIXEL_ADDR_W{1'b0}};
	 vread_per2B <= `PIXEL_ADDR_W'b0;
	 vread_shift2B <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incr2B <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_iter3B <= {`PIXEL_ADDR_W{1'b0}};
	 vread_per3B <= `PIXEL_ADDR_W'b0;
	 vread_shift3B <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incr3B <= {`PIXEL_INT_ADDR_W{1'b0}};
         //ix
         vread_ext <= 1'b0;
         ix_ext_addr <= {`IO_ADDR_W{1'b0}};
	 ix_iterA <= {`IX_ADDR_W{1'b0}};
	 ix_perA <= {`IX_ADDR_W{1'b0}};
	 ix_incrA <= {`IX_ADDR_W{1'b0}};
	 //xyolo
   	 xyolo_iter <= `PIXEL_ADDR_W'b0;
	 xyolo_per <= `PIXEL_ADDR_W'b0;
	 xyolo_shift <= {`SHIFT_W{1'b0}};
	 xyolo_b_shift <= {`SHIFT_W{1'b0}};
	 xyolo_bias <= 1'b0;
	 xyolo_leaky <= 1'b0;
	 xyolo_sigmoid <= 1'b0;
	 xyolo_sig_mask <= {`nYOLOvect{1'b0}};
	 xyolo_maxpool <= 1'b0;
	 xyolo_bypass <= 1'b0;
	 xyolo_bypass_adder <= 1'b0;
      end else begin
         integer j;
	 //vwrite
         if(vwrite_ext_addr_en) vwrite_ext_addr <= wdata[`IO_ADDR_W-1:0];
         if(vwrite_offset_en) vwrite_offset <= wdata[`IO_ADDR_W/2-1:0];
         if(vwrite_int_addr_en) vwrite_int_addr <= wdata[`VWRITE_ADDR_W-1:0];
         if(vwrite_iterA_en) vwrite_iterA <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_perA_en) vwrite_perA <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_shiftA_en) vwrite_shiftA <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_incrA_en) vwrite_incrA <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_startB_en) vwrite_startB <= wdata[`VWRITE_ADDR_W-1:0];
         if(vwrite_dutyB_en) vwrite_dutyB <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_delayB_en) vwrite_delayB <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_iterB_en) vwrite_iterB <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_perB_en) vwrite_perB <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_shiftB_en) vwrite_shiftB <= wdata[`PIXEL_ADDR_W-1:0];
         if(vwrite_incrB_en) vwrite_incrB <= wdata[`PIXEL_ADDR_W-1:0];
	 //vread
   	 if(vread_ext_addr_en) vread_ext_addr <= wdata[`IO_ADDR_W-1:0];
   	 if(vread_offset_en) vread_offset <= wdata[`IO_ADDR_W/2-1:0];
   	 if(vread_pp_en) vread_pp <= wdata[0];
         if(vread_int_addr_en) vread_int_addr <= wdata[`PIXEL_W_ADDR_W-1:0];
   	 if(vread_iterA_en) vread_iterA <= wdata[`EXT_ADDR_W-1:0];
	 if(vread_perA_en) vread_perA <= wdata[`EXT_ADDR_W-1:0];
   	 if(vread_shiftA_en) vread_shiftA <= wdata[`EXT_ADDR_W-1:0];
   	 if(vread_incrA_en) vread_incrA <= wdata[`EXT_ADDR_W-1:0];
	 if(vread_startB_en) vread_startB <= wdata[`PIXEL_INT_ADDR_W-1:0];
	 if(vread_iterB_en) vread_iterB <= wdata[`PIXEL_ADDR_W-1:0];
	 if(vread_perB_en) vread_perB <= wdata[`PIXEL_ADDR_W-1:0];
	 if(vread_shiftB_en) vread_shiftB <= wdata[`PIXEL_INT_ADDR_W-1:0];
	 if(vread_incrB_en) vread_incrB <= wdata[`PIXEL_INT_ADDR_W-1:0];
         if(vread_iter2B_en) vread_iter2B <= wdata[`PIXEL_ADDR_W-1:0];
	 if(vread_per2B_en) vread_per2B <= wdata[`PIXEL_ADDR_W-1:0];
   	 if(vread_shift2B_en) vread_shift2B <= wdata[`PIXEL_INT_ADDR_W-1:0];
   	 if(vread_incr2B_en) vread_incr2B <= wdata[`PIXEL_INT_ADDR_W-1:0];
         if(vread_iter3B_en) vread_iter3B <= wdata[`PIXEL_ADDR_W-1:0];
	 if(vread_per3B_en) vread_per3B <= wdata[`PIXEL_ADDR_W-1:0];
   	 if(vread_shift3B_en) vread_shift3B <= wdata[`PIXEL_INT_ADDR_W-1:0];
   	 if(vread_incr3B_en) vread_incr3B <= wdata[`PIXEL_INT_ADDR_W-1:0];
	 //ix
	 if(vread_ext_en) vread_ext <= wdata[0];
	 if(ix_ext_addr_en) ix_ext_addr <= wdata[`IO_ADDR_W-1:0];
	 if(ix_iterA_en) ix_iterA <= wdata[`IX_ADDR_W-1:0];
	 if(ix_perA_en) ix_perA <= wdata[`IX_ADDR_W-1:0];
	 if(ix_incrA_en) ix_incrA <= wdata[`IX_ADDR_W-1:0];
	 //xyolo
   	 if(xyolo_iter_en) xyolo_iter <= wdata[`PIXEL_ADDR_W-1:0];
	 if(xyolo_per_en) xyolo_per <= wdata[`PIXEL_ADDR_W-1:0];
	 if(xyolo_shift_en) xyolo_shift <= wdata[`SHIFT_W-1:0];
	 if(xyolo_b_shift_en) xyolo_b_shift <= wdata[`SHIFT_W-1:0];
	 if(xyolo_bias_en) xyolo_bias <= wdata[0];
	 if(xyolo_leaky_en) xyolo_leaky <= wdata[0];
	 if(xyolo_sigmoid_en) xyolo_sigmoid <= wdata[0];
	 if(xyolo_sig_mask_en) xyolo_sig_mask <= wdata[`nYOLOvect-1:0];
	 if(xyolo_maxpool_en) xyolo_maxpool <= wdata[0];
	 if(xyolo_bypass_en) xyolo_bypass <= wdata[0];
	 if(xyolo_bypass_adder_en) xyolo_bypass_adder <= wdata[0];
      end

   // configurable parameters shadow register
   always @(posedge clk, posedge rst)
      if(rst) begin
	 //vwrite
         vwrite_ext_addr_shadow <= `nSTAGES*`IO_ADDR_W'b0;
         vwrite_ext_addr_pip0 <= `nSTAGES*`IO_ADDR_W'b0;
         vwrite_ext_addr_pip1 <= `nSTAGES*`IO_ADDR_W'b0;
         vwrite_int_addr_shadow <= `VWRITE_ADDR_W'b0;
         vwrite_int_addr_pip0 <= `VWRITE_ADDR_W'b0;
         vwrite_int_addr_pip1 <= `VWRITE_ADDR_W'b0;
         vwrite_iterA_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_iterA_pip0 <= `PIXEL_ADDR_W'b0;
         vwrite_iterA_pip1 <= `PIXEL_ADDR_W'b0;
         vwrite_perA_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_perA_pip0 <= `PIXEL_ADDR_W'b0;
         vwrite_perA_pip1 <= `PIXEL_ADDR_W'b0;
         vwrite_shiftA_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_shiftA_pip0 <= `PIXEL_ADDR_W'b0;
         vwrite_shiftA_pip1 <= `PIXEL_ADDR_W'b0;
         vwrite_incrA_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_incrA_pip0 <= `PIXEL_ADDR_W'b0;
         vwrite_incrA_pip1 <= `PIXEL_ADDR_W'b0;
         vwrite_startB_shadow <= `VWRITE_ADDR_W'b0;
         vwrite_startB_pip <= `VWRITE_ADDR_W'b0;
         vwrite_dutyB_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_dutyB_pip <= `PIXEL_ADDR_W'b0;
         vwrite_delayB_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_delayB_pip <= `PIXEL_ADDR_W'b0;
         vwrite_iterB_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_iterB_pip <= `PIXEL_ADDR_W'b0;
         vwrite_perB_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_perB_pip <= `PIXEL_ADDR_W'b0;
         vwrite_shiftB_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_shiftB_pip <= `PIXEL_ADDR_W'b0;
         vwrite_incrB_shadow <= `PIXEL_ADDR_W'b0;
         vwrite_incrB_pip <= `PIXEL_ADDR_W'b0;
         mask_pip0 <= 1'b0;
	 mask_pip1 <= 1'b0;
	 mask_shadow <= 1'b0;
	 //vread
         vread_ext_addr_shadow <= `nSTAGES*`IO_ADDR_W'b0;
         vread_int_addr_shadow <= {`PIXEL_W_ADDR_W{1'b0}};
         vread_iterA_shadow <= `EXT_ADDR_W'b0;
         vread_perA_shadow <= `EXT_ADDR_W'b0;
	 vread_shiftA_shadow <= `EXT_ADDR_W'b0;
	 vread_incrA_shadow <= `EXT_ADDR_W'b0;
	 vread_startB_shadow <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_startB_pip <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_iterB_shadow <= {`PIXEL_ADDR_W{1'b0}};
	 vread_iterB_pip <= {`PIXEL_ADDR_W{1'b0}};
	 vread_perB_shadow <= `PIXEL_ADDR_W'b0;
	 vread_perB_pip <= `PIXEL_ADDR_W'b0;
	 vread_shiftB_shadow <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_shiftB_pip <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incrB_shadow <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incrB_pip <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_iter2B_shadow <= {`PIXEL_ADDR_W{1'b0}};
	 vread_iter2B_pip <= {`PIXEL_ADDR_W{1'b0}};
	 vread_per2B_shadow <= `PIXEL_ADDR_W'b0;
	 vread_per2B_pip <= `PIXEL_ADDR_W'b0;
	 vread_shift2B_shadow <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_shift2B_pip <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incr2B_shadow <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incr2B_pip <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_iter3B_shadow <= {`PIXEL_ADDR_W{1'b0}};
	 vread_iter3B_pip <= {`PIXEL_ADDR_W{1'b0}};
	 vread_per3B_shadow <= `PIXEL_ADDR_W'b0;
	 vread_per3B_pip <= `PIXEL_ADDR_W'b0;
	 vread_shift3B_shadow <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_shift3B_pip <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incr3B_shadow <= {`PIXEL_INT_ADDR_W{1'b0}};
	 vread_incr3B_pip <= {`PIXEL_INT_ADDR_W{1'b0}};
	 //ix
	 vread_ext_shadow <= 1'b0;
	 vread_ext_pip <= 1'b0;
	 ix_ext_addr_shadow <= {`IO_ADDR_W{1'b0}};
	 ix_iterA_shadow <= {`IX_ADDR_W{1'b0}};
	 ix_perA_shadow <= {`IX_ADDR_W{1'b0}};
	 ix_incrA_shadow <= {`IX_ADDR_W{1'b0}};
	 //xyolo
   	 xyolo_iter_shadow <= `PIXEL_ADDR_W'b0;
   	 xyolo_iter_pip <= `PIXEL_ADDR_W'b0;
	 xyolo_per_shadow <= `PIXEL_ADDR_W'b0;
	 xyolo_per_pip <= `PIXEL_ADDR_W'b0;
	 xyolo_shift_shadow <= {`SHIFT_W{1'b0}};
	 xyolo_shift_pip <= {`SHIFT_W{1'b0}};
	 xyolo_b_shift_shadow <= {`SHIFT_W{1'b0}};
	 xyolo_b_shift_pip <= {`SHIFT_W{1'b0}};
	 xyolo_bias_shadow <= 1'b0;
	 xyolo_bias_pip <= 1'b0;
	 xyolo_leaky_shadow <= 1'b0;
	 xyolo_leaky_pip <= 1'b0;
	 xyolo_sigmoid_shadow <= 1'b0;
	 xyolo_sigmoid_pip <= 1'b0;
	 xyolo_sig_mask_shadow <= {`nYOLOvect{1'b0}};
	 xyolo_sig_mask_pip <= {`nYOLOvect{1'b0}};
	 xyolo_maxpool_shadow <= 1'b0;
	 xyolo_maxpool_pip <= 1'b0;
	 xyolo_bypass_shadow <= 1'b0;
	 xyolo_bypass_pip <= 1'b0;
	 xyolo_bypass_adder_shadow <= 1'b0;
	 xyolo_bypass_adder_pip <= 1'b0;
	 xyolo_delay <= `PIXEL_ADDR_W'b0;
      end else if(run) begin
	 //vwrite
         vwrite_ext_addr_pip0 <= vwrite_ext_addr_bus;
         vwrite_ext_addr_pip1 <= vwrite_ext_addr_pip0;
         vwrite_ext_addr_shadow <= vwrite_ext_addr_pip1;
	 //XOR ensures ping-pong happens when acessing external mem
	 vwrite_int_addr_pip0 <= {vwrite_int_addr_pip0[`VWRITE_ADDR_W-1] ^ |vwrite_iterA_pip0, vwrite_int_addr[`VWRITE_ADDR_W-2:0]};
         vwrite_int_addr_pip1 <= vwrite_int_addr_pip0;
         vwrite_int_addr_shadow <= vwrite_int_addr_pip1;
	 vwrite_iterA_pip0 <= vwrite_iterA;
	 vwrite_iterA_pip1 <= vwrite_iterA_pip0;
	 vwrite_iterA_shadow <= vwrite_iterA_pip1;
	 vwrite_perA_pip0 <= vwrite_perA;
	 vwrite_perA_pip1 <= vwrite_perA_pip0;
	 vwrite_perA_shadow <= vwrite_perA_pip1;
	 vwrite_shiftA_pip0 <= vwrite_shiftA;
	 vwrite_shiftA_pip1 <= vwrite_shiftA_pip0;
	 vwrite_shiftA_shadow <= vwrite_shiftA_pip1;
	 vwrite_incrA_pip0 <= vwrite_incrA;
	 vwrite_incrA_pip1 <= vwrite_incrA_pip0;
	 vwrite_incrA_shadow <= vwrite_incrA_pip1;
	 vwrite_startB_pip <= {vwrite_startB_pip[`VWRITE_ADDR_W-1] ^ |vwrite_iterA_pip0, vwrite_startB[`VWRITE_ADDR_W-2:0]};
	 vwrite_startB_shadow <= vwrite_startB_pip;
         vwrite_dutyB_pip <= vwrite_dutyB;
         vwrite_dutyB_shadow <= vwrite_dutyB_pip;
	 vwrite_delayB_pip <= vwrite_delayB;
	 vwrite_delayB_shadow <= vwrite_delayB_pip;
	 vwrite_iterB_pip <= vwrite_iterB;
	 vwrite_iterB_shadow <= vwrite_iterB_pip;
	 vwrite_perB_pip <= vwrite_perB;
	 vwrite_perB_shadow <= vwrite_perB_pip;
	 vwrite_shiftB_pip <= vwrite_shiftB;
	 vwrite_shiftB_shadow <= vwrite_shiftB_pip;
	 vwrite_incrB_pip <= vwrite_incrB;
	 vwrite_incrB_shadow <= vwrite_incrB_pip;
         mask_pip0 <= |vwrite_offset;
	 mask_pip1 <= mask_pip0;
	 mask_shadow <= mask_pip1;
	 //vread
	 vread_ext_addr_shadow <= vread_ext_addr_bus;
	 //XOR ensures ping-pong happens when acessing external mem
	 vread_int_addr_shadow <= vread_pp ? {vread_int_addr_shadow[`PIXEL_W_ADDR_W-1] ^ |vread_iterA, vread_int_addr[`PIXEL_W_ADDR_W-2:0]} : vread_int_addr;
	 vread_iterA_shadow <= vread_iterA;
	 vread_perA_shadow <= vread_perA;
	 vread_shiftA_shadow <= vread_shiftA;
	 vread_incrA_shadow <= vread_incrA;
	 vread_startB_pip <= vread_pp ? {vread_startB_pip[`PIXEL_INT_ADDR_W-1] ^ |vread_iterA, vread_startB[`PIXEL_INT_ADDR_W-2:0]} : vread_startB;
	 vread_startB_shadow <= vread_startB_pip;
	 vread_iterB_pip <= vread_iterB;
	 vread_iterB_shadow <= vread_iterB_pip;
	 vread_perB_pip <= vread_perB;
	 vread_perB_shadow <= vread_perB_pip;
	 vread_shiftB_pip <= vread_shiftB;
	 vread_shiftB_shadow <= vread_shiftB_pip;
	 vread_incrB_pip <= vread_incrB;
	 vread_incrB_shadow <= vread_incrB_pip;
	 vread_iter2B_pip <= vread_iter2B;
	 vread_iter2B_shadow <= vread_iter2B_pip;
	 vread_per2B_pip <= vread_per2B;
	 vread_per2B_shadow <= vread_per2B_pip;
	 vread_shift2B_pip <= vread_shift2B;
	 vread_shift2B_shadow <= vread_shift2B_pip;
	 vread_incr2B_pip <= vread_incr2B;
	 vread_incr2B_shadow <= vread_incr2B_pip;
	 vread_iter3B_pip <= vread_iter3B;
	 vread_iter3B_shadow <= vread_iter3B_pip;
	 vread_per3B_pip <= vread_per3B;
	 vread_per3B_shadow <= vread_per3B_pip;
	 vread_shift3B_pip <= vread_shift3B;
	 vread_shift3B_shadow <= vread_shift3B_pip;
	 vread_incr3B_pip <= vread_incr3B;
	 vread_incr3B_shadow <= vread_incr3B_pip;
	 //ix
	 vread_ext_pip <= vread_ext;
	 vread_ext_shadow <= vread_ext_pip;
	 ix_ext_addr_shadow <= ix_ext_addr;
	 ix_iterA_shadow <= ix_iterA;
	 ix_perA_shadow <= ix_perA;
	 ix_incrA_shadow <= ix_incrA;
	 //xyolo
	 xyolo_iter_pip <= xyolo_iter;
	 xyolo_iter_shadow <= xyolo_iter_pip;
	 xyolo_per_pip <= xyolo_per;
	 xyolo_per_shadow <= xyolo_per_pip;
	 xyolo_shift_pip <= xyolo_shift;
	 xyolo_shift_shadow <= xyolo_shift_pip;
	 xyolo_b_shift_pip <= xyolo_b_shift;
	 xyolo_b_shift_shadow <= xyolo_b_shift_pip;
	 xyolo_bias_pip <= xyolo_bias;
	 xyolo_bias_shadow <= xyolo_bias_pip;
	 xyolo_leaky_pip <= xyolo_leaky;
	 xyolo_leaky_shadow <= xyolo_leaky_pip;
	 xyolo_sigmoid_pip <= xyolo_sigmoid;
	 xyolo_sigmoid_shadow <= xyolo_sigmoid_pip;
	 xyolo_sig_mask_pip <= xyolo_sig_mask;
	 xyolo_sig_mask_shadow <= xyolo_sig_mask_pip;
	 xyolo_maxpool_pip <= xyolo_maxpool;
	 xyolo_maxpool_shadow <= xyolo_maxpool_pip;
	 xyolo_bypass_pip <= xyolo_bypass;
	 xyolo_bypass_shadow <= xyolo_bypass_pip;
	 xyolo_bypass_adder_pip <= xyolo_bypass_adder;
	 xyolo_bypass_adder_shadow <= xyolo_bypass_adder_pip;
	 xyolo_delay <= vread_lat + vread_ext_pip; //+1 due to reading addr from ix_mem
      end

   // external address calculation based on base and offset values
   genvar i;
   assign vread_ext_addr_bus[`nSTAGES*`IO_ADDR_W-1 -: `IO_ADDR_W] = vread_ext_addr; //base value
   assign vwrite_ext_addr_bus[`nSTAGES*`IO_ADDR_W-1 -: `IO_ADDR_W] = vwrite_ext_addr; //base value
   generate
      for (i=1; i < `nSTAGES; i=i+1) begin : addrgen_calc

         //instantiate vread 4-stage multiplier
         mul_4stage # (
            .DATA_W(`IO_ADDR_W/2),
	    .SIGNED(0)
         ) vread_mul (
            //control
            .clk(clk),
            .ld_acc(1'b1),
            //data
            .inA(i[`IO_ADDR_W/2-1:0]),
            .inB(vread_offset),
            .inC(vread_ext_addr),
            .out(vread_ext_addr_bus[`nSTAGES*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W])
         );

         //instantiate vwrite 4-stage multiplier
         mul_4stage # (
            .DATA_W(`IO_ADDR_W/2),
	    .SIGNED(0)
         ) vwrite_mul (
            //control
            .clk(clk),
            .ld_acc(1'b1),
            //data
            .inA(i[`IO_ADDR_W/2-1:0]),
            .inB(vwrite_offset),
            .inC(vwrite_ext_addr),
            .out(vwrite_ext_addr_bus[`nSTAGES*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W])
         );

      end
   endgenerate

   //
   // common to all stages
   //

   // ix vread external addrgen
   ext_addrgen #(
        .DATA_W(DATABUS_W),
        .EXT_ADDR_W(`IX_ADDR_W),
        .EXT_PERIOD_W(`IX_ADDR_W),
        .MEM_ADDR_W(`IX_W_ADDR_W)
   ) ix_ext_addrgen (
        .clk(clk),
        .rst(rst),
        // Control
        .run(run_reg),
        .done(ix_done),
        // Configuration
        .ext_addr(ix_ext_addr_shadow),
        .int_addr({`IX_W_ADDR_W{1'b0}}),
        .direction(2'b01),
        .iterations(ix_iterA_shadow),
        .period(ix_perA_shadow),
        .duty(ix_perA_shadow),
        .start(`IX_ADDR_W'd0),
        .shift(`IX_ADDR_W'd0),
        .incr(ix_incrA_shadow),
        .delay(`IX_ADDR_W'd0),
        // Databus interface
        .databus_ready(vread_m_resp[`ready((`nSTAGES))]),
        .databus_valid(vread_m_req[`valid((`nSTAGES))]),
        .databus_addr(vread_m_req[`address((`nSTAGES), `IO_ADDR_W)]),
        .databus_rdata(vread_m_resp[`rdata((`nSTAGES))]),
        .databus_wdata(vread_m_req[`wdata((`nSTAGES))]),
        .databus_wstrb(vread_m_req[`wstrb((`nSTAGES))]),
        // internal memory interface
        .valid(ix_enA),
        .we(ix_we),
        .addr(ix_addrA),
        .data_out(ix_inA),
        .data_in({DATABUS_W{1'b0}})
   );

   // ix vread mem
   iob_2p_assim_mem_w_big #(
     .W_DATA_W(DATABUS_W),
     .W_ADDR_W(`IX_W_ADDR_W),
     .R_DATA_W(DATAPATH_W),
     .R_ADDR_W(`IX_ADDR_W)
   ) ix_mem (
     .clk(clk),
     // Writting port
     .w_en(ix_enA & ix_we),
     .w_addr(ix_addrA),
     .data_in(ix_inA),
     // Reading port
     .r_en(vread_enB),
     .r_addr(vread_addrB[`IX_ADDR_W-1:0]),
     .data_out(ix_out)
   );

   //vread internal address generator
   xaddrgen3 # (
      .MEM_ADDR_W(`PIXEL_INT_ADDR_W),
      .PERIOD_W(`PIXEL_ADDR_W)
   ) vread_addrgenB (
      .clk(clk),
      .rst(rst),
      .run(run_reg),
      .iterations(vread_iterB_shadow),
      .period(vread_perB_shadow),
      .duty(vread_perB_shadow),
      .start(vread_startB_shadow),
      .shift(vread_shiftB_shadow),
      .incr(vread_incrB_shadow),
      .delay(`PIXEL_ADDR_W'd0),
      .iterations2(vread_iter2B_shadow),
      .period2(vread_per2B_shadow),
      .shift2(vread_shift2B_shadow),
      .incr2(vread_incr2B_shadow),
      .iterations3(vread_iter3B_shadow),
      .period3(vread_per3B_shadow),
      .shift3(vread_shift3B_shadow),
      .incr3(vread_incr3B_shadow),
      .addr(vread_addrB),
      .mem_en(vread_enB),
      .done(vread_doneB)
   );

   //xyolo address generator
   xaddrgen # (
      .MEM_ADDR_W(`PIXEL_ADDR_W),
      .PERIOD_W(`PIXEL_ADDR_W)
   ) xyolo_addrgen (
      .clk(clk),
      .rst(rst),
      .init(run_reg),
      .run(run_reg & |xyolo_iter_shadow),
      .pause(1'b0),
      .iterations(xyolo_iter_shadow),
      .period(xyolo_per_shadow),
      .duty(xyolo_per_shadow),
      .start(`PIXEL_ADDR_W'b0),
      .shift(-xyolo_per_shadow[`PIXEL_ADDR_W-1:0]),
      .incr(`PIXEL_ADDR_W'b1),
      .delay(xyolo_delay),
      .addr(xyolo_addr),
      .mem_en(),
      .done()
   );

   //compute xyolo load wires
   assign ld_acc = (xyolo_addr == {`PIXEL_ADDR_W{1'd0}});
   assign ld_mp = |mp_cnt;
   assign ld_res = ld_acc4 || xyolo_bypass_shadow || ~xyolo_leaky_shadow;

   //update xyolo registers
   always @ (posedge clk, posedge rst)
      if(rst) begin
	 ld_acc0 <= 1'b0;
         ld_acc1 <= 1'b0;
	 ld_acc2 <= 1'b0;
	 ld_acc3 <= 1'b0;
	 ld_acc4 <= 1'b0;
	 mp_cnt <= 2'b0;
	 nmac_cnt <= {N_MACS_W{1'b0}};
      end else if(run_reg) begin
         ld_acc0 <= 1'b0;
         ld_acc1 <= 1'b0;
	 ld_acc2 <= 1'b0;
	 ld_acc3 <= 1'b0;
	 ld_acc4 <= 1'b0;
	 nmac_cnt <= {N_MACS_W{1'b0}};
         if(xyolo_bypass_shadow)
	   mp_cnt <= 2'd0;
         else
           mp_cnt <= 2'd3;
      end else begin
	 ld_acc0 <= ld_acc;
         ld_acc1 <= ld_acc0;
	 ld_acc2 <= ld_acc1;
	 ld_acc3 <= ld_acc2;
	 ld_acc4 <= ld_acc3;
	 if(ld_res) mp_cnt <= mp_cnt + 1;
	 if(ld_acc1 && ~ld_acc0) nmac_cnt <= nmac_cnt + 1; 
      end

   //vwrite internal address generator
   xaddrgen # (
      .MEM_ADDR_W(`PIXEL_ADDR_W),
      .PERIOD_W(`PIXEL_ADDR_W)
   ) vwrite_addrgenB (
      .clk(clk),
      .rst(rst),
      .init(run_reg),
      .run(run_reg & |vwrite_iterB_shadow),
      .pause(1'b0),
      .iterations(vwrite_iterB_shadow),
      .period(vwrite_perB_shadow),
      .duty(vwrite_dutyB_shadow),
      .start({{`PIXEL_ADDR_W-`VWRITE_ADDR_W{1'b0}}, vwrite_startB_shadow}),
      .shift(vwrite_shiftB_shadow),
      .incr(vwrite_incrB_shadow),
      .delay(vwrite_delayB_shadow),
      .addr(vwrite_addrB),
      .mem_en(vwrite_enB),
      .done(vwrite_doneB)
   );

   // update vwrite enable counter
   always @ (posedge clk, posedge rst)
      if(rst || run_reg)
         vwrite_enB_cnt <= {$clog2(`nYOLOvect)+2{1'b0}};
      else if(vwrite_enB_reg)
	 vwrite_enB_cnt <= vwrite_enB_cnt + 1'b1;

   // enable vwrite decoder
   always @ * begin
      integer j;
      vwrite_enB_stage = {`nYOLOvect{1'b0}};
      for(j = 0; j < `nYOLOvect; j++) begin
         if(xyolo_bypass_shadow)
	    vwrite_enB_stage[j] = ((j*4+3) == vwrite_enB_cnt) ? vwrite_enB_reg : 1'b0;
	 else if(xyolo_bypass_adder_shadow)
	    vwrite_enB_stage[j] = (((j/`nYOLOmacs)*2+1) == vwrite_enB_cnt[$clog2(`nYOLOmacs):0]) ? vwrite_enB_reg : 1'b0;
	 else
	    vwrite_enB_stage[j] = vwrite_enB;
      end
   end

   // register vwrite mem write inputs
   always @ (posedge clk, posedge rst)
      if(rst) begin
	 vwrite_enB_reg <= 1'b0;
         vwrite_enB_stage_reg <= `nYOLOvect'b0;
         vwrite_addrB_r0 <= `PIXEL_ADDR_W'b0;
         vwrite_addrB_r1 <= `PIXEL_ADDR_W'b0;
         vwrite_addrB_stage <= `PIXEL_ADDR_W'b0;
      end else begin
         vwrite_enB_reg <= vwrite_enB;
         vwrite_enB_stage_reg <= vwrite_enB_stage;
         vwrite_addrB_r0 <= vwrite_addrB;
         vwrite_addrB_r1 <= vwrite_addrB_r0;
         vwrite_addrB_stage <= vwrite_addrB_mux;
      end

   //
   // stages
   //

   // register vread mem read inputs
   always @ (posedge clk) begin
      vread_enB_reg <= vread_enB;
      vread_addrB_reg <= vread_ext_shadow ? {vread_startB_shadow[`PIXEL_INT_ADDR_W-1], ix_out[`PIXEL_INT_ADDR_W-2:0]} : vread_addrB;
   end

   //instantiate stages
   generate
     for(i = 0; i < `nSTAGES; i=i+1)  begin : stages

        //check if asking for the same data as previous vread
	wire cond = (databus_addr[`IO_ADDR_W-1:0] == vread_m_req[`address((`nSTAGES-i-1), `IO_ADDR_W)]) & vread_m_req[`valid((`nSTAGES-i-1))];
        wire databus_ready_w = (cond & databus_ready[0]) | vread_m_resp[`ready((`nSTAGES-i-1))];

 	//prepare vwrite mask
 	wire [`PIXEL_ADDR_W-1:0] vwrite_iterA_w = (i == 0) ? vwrite_iterA_shadow : vwrite_iterA_shadow & {`PIXEL_ADDR_W{mask_shadow}};
	wire [`EXT_ADDR_W-1:0] vread_iterA_w = (i == 0) ? vread_iterA_shadow : vread_iterA_shadow & {`EXT_ADDR_W{mask_pip0}};

        //instantiate xyolo_write_stage
        xyolo_write_stage # (
           .DATAPATH_W(DATAPATH_W),
	   .DATABUS_W(DATABUS_W)
        ) stage (
           .clk(clk),
           .rst(rst),
           //control
           .global_run(run_reg),
           .done(stages_done[i]),
           //internal addrgen
           .vread_enB(vread_enB_reg),
           .vwrite_enB(vwrite_enB_stage_reg),
           .vread_addrB(vread_addrB_reg),
           .vwrite_addrB(vwrite_addrB_stage[`VWRITE_ADDR_W-1:0]),
           //load control
           .ld_acc(ld_acc0),
           .ld_mp(ld_mp),
           .ld_res(ld_res),
	   .ld_nmac(nmac_cnt),	 
           //vread config params
           .vread_ext_addr(vread_ext_addr_shadow[`nSTAGES*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W]),
           .vread_int_addr(vread_int_addr_shadow),
           .vread_iterA(vread_iterA_w),
           .vread_perA(vread_perA_shadow),
           .vread_shiftA(vread_shiftA_shadow),
           .vread_incrA(vread_incrA_shadow),
           //vwrite config params
           .vwrite_ext_addr(vwrite_ext_addr_shadow[`nSTAGES*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W]),
           .vwrite_int_addr(vwrite_int_addr_shadow),
           .vwrite_iterA(vwrite_iterA_w),
           .vwrite_perA(vwrite_perA_shadow),
           .vwrite_shiftA(vwrite_shiftA_shadow),
           .vwrite_incrA(vwrite_incrA_shadow),
           //xyolo config params
           .xyolo_bias(xyolo_bias_shadow),
           .xyolo_leaky(xyolo_leaky_shadow),
           .xyolo_sigmoid(xyolo_sigmoid_shadow),
      	   .xyolo_sig_mask(xyolo_sig_mask_shadow),
           .xyolo_maxpool(xyolo_maxpool_shadow),
           .xyolo_bypass(xyolo_bypass_shadow),
           .xyolo_bypass_adder(xyolo_bypass_adder_shadow),
           .xyolo_shift(xyolo_shift_shadow),
           .xyolo_b_shift(xyolo_b_shift_shadow),
      	   //databus interface
      	   .databus_ready({vwrite_m_resp[`ready((`nSTAGES-1-i))], databus_ready_w}),
           .databus_valid({vwrite_m_req[`valid((`nSTAGES-1-i))], vread_m_req[`valid((`nSTAGES-1-i))]}),
           .databus_addr({vwrite_m_req[`address((`nSTAGES-1-i), `IO_ADDR_W)], vread_m_req[`address((`nSTAGES-1-i), `IO_ADDR_W)]}),
           .databus_rdata({vwrite_m_resp[`rdata((`nSTAGES-1-i))], databus_rdata[DATABUS_W-1:0]}),
           .databus_wdata({vwrite_m_req[`wdata((`nSTAGES-1-i))], vread_m_req[`wdata((`nSTAGES-1-i))]}),
           .databus_wstrb({vwrite_m_req[`wstrb((`nSTAGES-1-i))], vread_m_req[`wstrb((`nSTAGES-1-i))]}),
           //input data
           .flow_in_bias(flow_in_bias),
           .flow_in_weight(flow_in_weight)
        );

     end
   endgenerate

   //
   // vread MERGE
   //

   //instantiate merge
   vread_merge #(
      .N_MASTERS(`nSTAGES+1),
      .DATA_W(DATABUS_W),
      .ADDR_W(ADDR_W)
   ) vread_merge (
      //masters interface
      .m_req(vread_m_req),
      .m_resp(vread_m_resp),
      //slave interface
      .s_req(vread_s_req),
      .s_resp(vread_s_resp)
   );

   //unconcatenate merge slave interface back to native interface
   assign databus_addr[`IO_ADDR_W-1:0] = vread_s_req[`address(0, `IO_ADDR_W)];
   assign databus_wdata[DATABUS_W-1:0] = vread_s_req[`wdata(0)];
   assign databus_wstrb[DATABUS_W/8-1:0] = vread_s_req[`wstrb(0)];
   assign databus_valid[0] = vread_s_req[`valid(0)];
   assign vread_s_resp[`rdata(0)] = databus_rdata[DATABUS_W-1:0];
   assign vread_s_resp[`ready(0)] = databus_ready[0];

   //
   // vwrite MERGE
   //

   //instantiate merge
   merge #(
      .N_MASTERS(`nSTAGES),
      .DATA_W(DATABUS_W),
      .ADDR_W(ADDR_W)
   ) vwrite_merge (
      .clk (clk),
      .rst (rst),
      //masters interface
      .m_req(vwrite_m_req),
      .m_resp(vwrite_m_resp),
      //slave interface
      .s_req(vwrite_s_req),
      .s_resp(vwrite_s_resp)
   );

   //unconcatenate merge slave interface back to native interface
   assign databus_addr[2*`IO_ADDR_W-1:`IO_ADDR_W] = vwrite_s_req[`address(0, `IO_ADDR_W)];
   assign databus_wdata[2*DATABUS_W-1:DATABUS_W] = vwrite_s_req[`wdata(0)];
   assign databus_wstrb[2*DATABUS_W/8-1:DATABUS_W/8] = vwrite_s_req[`wstrb(0)];
   assign databus_valid[1] = vwrite_s_req[`valid(0)];
   assign vwrite_s_resp[`rdata(0)] = databus_rdata[2*DATABUS_W-1:DATABUS_W];
   assign vwrite_s_resp[`ready(0)] = databus_ready[1];

endmodule
