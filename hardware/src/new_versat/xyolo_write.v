`timescale 1ns / 1ps

`include "xversat.vh"
`include "xyolo_write.vh"
`include "xyolo_read.vh"

module xyolo_write #(
    	parameter                       DATAPATH_W = 32,
        parameter			DATABUS_W = 256
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
    	input [`nYOLOvect*DATAPATH_W-1:0] flow_in_weight,

	// DMA - number of tranfers per burst
	output [2*`AXI_LEN_W-1:0]       dma_len,
	output [`AXI_SIZE_W-1:0]	dma_size
    );

   // vread latency
   localparam [`PERIOD_W-1:0]           vread_lat = `XYOLO_READ_LAT;

   // local parameter for merge
   localparam                           ADDR_W = `IO_ADDR_W;
   localparam				DATA_W = DATABUS_W;

   // vwrite configuration enables
   reg                                  vwrite_ext_addr_en;
   reg                                  vwrite_offset_en;
   reg                                  vwrite_len_en;
   reg                                  vwrite_size_en;
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
   reg					vread_len_en;
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

   // xyolo configuration enables
   reg					xyolo_iter_en;
   reg					xyolo_per_en;
   reg					xyolo_shift_en;
   reg					xyolo_bias_en;
   reg					xyolo_leaky_en;
   reg					xyolo_maxpool_en;
   reg					xyolo_bypass_en;

   // vwrite configuration parameters
   reg [`IO_ADDR_W-1:0]                 vwrite_ext_addr;
   reg [`nSTAGES*`IO_ADDR_W-1:0]        vwrite_ext_addr_shadow, vwrite_ext_addr_pip0, vwrite_ext_addr_pip1;
   reg [`IO_ADDR_W/2-1:0]               vwrite_offset;
   reg [`AXI_LEN_W-1:0]             	vwrite_len, vwrite_len_shadow, vwrite_len_pip0, vwrite_len_pip1;
   reg [`AXI_SIZE_W-1:0]             	vwrite_size, vwrite_size_shadow, vwrite_size_pip0, vwrite_size_pip1;
   reg [`VWRITE_ADDR_W-1:0]             vwrite_int_addr, vwrite_int_addr_shadow, vwrite_int_addr_pip0, vwrite_int_addr_pip1;
   reg [`MEM_ADDR_W-1:0]                vwrite_iterA, vwrite_iterA_shadow, vwrite_iterA_pip0, vwrite_iterA_pip1;
   reg [`PERIOD_W-1:0]                  vwrite_perA, vwrite_perA_shadow, vwrite_perA_pip0, vwrite_perA_pip1;
   reg [`MEM_ADDR_W-1:0]                vwrite_shiftA, vwrite_shiftA_shadow, vwrite_shiftA_pip0, vwrite_shiftA_pip1;
   reg [`MEM_ADDR_W-1:0]                vwrite_incrA, vwrite_incrA_shadow, vwrite_incrA_pip0, vwrite_incrA_pip1;
   reg [`VWRITE_ADDR_W-1:0]             vwrite_startB, vwrite_startB_pip, vwrite_startB_shadow;
   reg [`PERIOD_W-1:0]                  vwrite_dutyB, vwrite_dutyB_pip, vwrite_dutyB_shadow;
   reg [`PERIOD_W-1:0]                  vwrite_delayB, vwrite_delayB_pip, vwrite_delayB_shadow;
   reg [`MEM_ADDR_W-1:0]                vwrite_iterB, vwrite_iterB_pip, vwrite_iterB_shadow;
   reg [`PERIOD_W-1:0]                  vwrite_perB, vwrite_perB_pip, vwrite_perB_shadow;
   reg [`MEM_ADDR_W-1:0]                vwrite_shiftB, vwrite_shiftB_pip, vwrite_shiftB_shadow;
   reg [`MEM_ADDR_W-1:0]                vwrite_incrB, vwrite_incrB_pip, vwrite_incrB_shadow;
   reg                                  vwrite_bypass_shadow; //used for maxpool and upsample

   // vread configuration parameters
   reg [`IO_ADDR_W-1:0]			vread_ext_addr;
   reg [`nSTAGES*`IO_ADDR_W-1:0]	vread_ext_addr_shadow;
   reg [`IO_ADDR_W/2-1:0]		vread_offset;
   reg [`AXI_LEN_W-1:0]			vread_len, vread_len_shadow;
   reg [`W_ADDR_W-1:0]			vread_int_addr, vread_int_addr_shadow;
   reg [`EXT_ADDR_W-1:0]		vread_iterA, vread_iterA_shadow;
   reg [`EXT_PERIOD_W-1:0]              vread_perA, vread_perA_shadow;
   reg [`EXT_ADDR_W-1:0]                vread_shiftA, vread_shiftA_shadow;
   reg [`EXT_ADDR_W-1:0]                vread_incrA, vread_incrA_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_startB, vread_startB_pip, vread_startB_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_iterB, vread_iterB_pip, vread_iterB_shadow;
   reg [`PERIOD_W-1:0]                  vread_perB, vread_perB_pip, vread_perB_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_shiftB, vread_shiftB_pip, vread_shiftB_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_incrB, vread_incrB_pip, vread_incrB_shadow;
   reg [`MEM_ADDR_W-1:0]		vread_iter2B, vread_iter2B_pip, vread_iter2B_shadow;
   reg [`PERIOD_W-1:0]                  vread_per2B, vread_per2B_pip, vread_per2B_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_shift2B, vread_shift2B_pip, vread_shift2B_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_incr2B, vread_incr2B_pip, vread_incr2B_shadow;
   reg [`MEM_ADDR_W-1:0]		vread_iter3B, vread_iter3B_pip, vread_iter3B_shadow;
   reg [`PERIOD_W-1:0]                  vread_per3B, vread_per3B_pip, vread_per3B_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_shift3B, vread_shift3B_pip, vread_shift3B_shadow;
   reg [`MEM_ADDR_W-1:0]                vread_incr3B, vread_incr3B_pip, vread_incr3B_shadow;

   // xyolo configuration parameters
   reg [`MEM_ADDR_W-1:0]		xyolo_iter, xyolo_iter_pip, xyolo_iter_shadow;
   reg [`PERIOD_W-1:0]			xyolo_per, xyolo_per_pip, xyolo_per_shadow;
   reg [`SHIFT_W-1:0]			xyolo_shift, xyolo_shift_pip, xyolo_shift_shadow;
   reg 					xyolo_bias, xyolo_bias_pip, xyolo_bias_shadow;
   reg					xyolo_leaky, xyolo_leaky_pip, xyolo_leaky_shadow;
   reg					xyolo_maxpool, xyolo_maxpool_pip, xyolo_maxpool_shadow;
   reg					xyolo_bypass, xyolo_bypass_pip, xyolo_bypass_shadow;

   // external address buses
   wire [`nSTAGES*`IO_ADDR_W-1:0]       vwrite_ext_addr_bus, vread_ext_addr_bus;

   // internal addrgen wires and regs
   wire                                 vread_enB, vwrite_enB;
   reg                                  vread_enB_reg;
   wire [`MEM_ADDR_W-1:0]               vread_addrB, vwrite_addrB;
   reg [`MEM_ADDR_W-1:0]                vread_addrB_reg;
   wire                                 vread_doneB, vwrite_doneB;

   // done output
   wire [`nSTAGES-1:0]                  stages_done;
   assign                               done = &{vread_doneB, vwrite_doneB, stages_done};

   // define number of transactions (DMA)
   assign                               dma_len = {vwrite_len_shadow, vread_len_shadow};
   assign				dma_size = vwrite_size_shadow;

   // run signals
   reg                                  run_reg;

   // xyolo wires and regs
   wire [`MEM_ADDR_W-1:0]		xyolo_addr;
   wire                                 ld_acc, ld_mp, ld_res;
   reg                                  ld_acc0, ld_acc1;
   reg [1:0]                            mp_cnt;

   // merge master interface
   wire [`nSTAGES*`REQ_W-1:0]           vread_m_req, vwrite_m_req;
   wire [`nSTAGES*`RESP_W-1:0]          vread_m_resp, vwrite_m_resp;

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
      vwrite_len_en = 1'b0;
      vwrite_size_en = 1'b0;
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
      vread_len_en = 1'b0;
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
      //xyolo
      xyolo_iter_en = 1'b0;
      xyolo_per_en = 1'b0;
      xyolo_shift_en = 1'b0;
      xyolo_bias_en = 1'b0;
      xyolo_leaky_en = 1'b0;
      xyolo_maxpool_en = 1'b0;
      xyolo_bypass_en = 1'b0;
      if(valid & wstrb)
         case(addr)
	    //vwrite
            `VWRITE_CONF_EXT_ADDR : vwrite_ext_addr_en = 1'b1;
            `VWRITE_CONF_OFFSET : vwrite_offset_en = 1'b1;
            `VWRITE_CONF_LEN : vwrite_len_en = 1'b1;
            `VWRITE_CONF_SIZE : vwrite_size_en = 1'b1;
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
	    `VREAD_CONF_LEN : vread_len_en = 1'b1;
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
	    //xyolo
	    `XYOLO_CONF_ITER : xyolo_iter_en = 1'b1;
	    `XYOLO_CONF_PER : xyolo_per_en = 1'b1;
	    `XYOLO_CONF_SHIFT : xyolo_shift_en = 1'b1;
	    `XYOLO_CONF_BIAS : xyolo_bias_en = 1'b1;
	    `XYOLO_CONF_LEAKY : xyolo_leaky_en = 1'b1;
            `XYOLO_CONF_MAXPOOL : xyolo_maxpool_en = 1'b1;
	    `XYOLO_CONF_BYPASS : xyolo_bypass_en = 1'b1;
            default : ;
         endcase
   end

   // addr decoder parameters
   always @(posedge clk, posedge clear, posedge rst)
      if(clear || rst) begin
	 //vwrite
         vwrite_ext_addr <= `IO_ADDR_W'b0;
         vwrite_offset <= {`IO_ADDR_W/2{1'b0}};
	 vwrite_len <= `AXI_LEN_W'b0;
	 vwrite_size <= `AXI_SIZE_W'b0;
         vwrite_int_addr <= `VWRITE_ADDR_W'b0;
         vwrite_iterA <= `MEM_ADDR_W'b0;
         vwrite_perA <= `PERIOD_W'b0;
         vwrite_shiftA <= `MEM_ADDR_W'b0;
         vwrite_incrA <= `MEM_ADDR_W'b0;
         vwrite_startB <= `VWRITE_ADDR_W'b0;
         vwrite_dutyB <= `PERIOD_W'b0;
         vwrite_delayB <= `PERIOD_W'b0;
         vwrite_iterB <= `MEM_ADDR_W'b0;
         vwrite_perB <= `PERIOD_W'b0;
         vwrite_shiftB <= `MEM_ADDR_W'b0;
         vwrite_incrB <= `MEM_ADDR_W'b0;
	 //vread
	 vread_ext_addr <= `IO_ADDR_W'b0;
	 vread_offset <= {`IO_ADDR_W/2{1'b0}};
	 vread_len <= `AXI_LEN_W'b0;
         vread_int_addr <= {`W_ADDR_W{1'b0}};
         vread_iterA <= `EXT_ADDR_W'b0;
         vread_perA <= `EXT_PERIOD_W'b0;
	 vread_shiftA <= `EXT_ADDR_W'b0;
	 vread_incrA <= `EXT_ADDR_W'b0;
	 vread_startB <= `MEM_ADDR_W'b0;
	 vread_iterB <= `MEM_ADDR_W'b0;
	 vread_perB <= `PERIOD_W'b0;
	 vread_shiftB <= `MEM_ADDR_W'b0;
	 vread_incrB <= `MEM_ADDR_W'b0;
	 vread_iter2B <= `MEM_ADDR_W'b0;
	 vread_per2B <= `PERIOD_W'b0;
	 vread_shift2B <= `MEM_ADDR_W'b0;
	 vread_incr2B <= `MEM_ADDR_W'b0;
	 vread_iter3B <= `MEM_ADDR_W'b0;
	 vread_per3B <= `PERIOD_W'b0;
	 vread_shift3B <= `MEM_ADDR_W'b0;
	 vread_incr3B <= `MEM_ADDR_W'b0;
	 //xyolo
   	 xyolo_iter <= `MEM_ADDR_W'b0;
	 xyolo_per <= `PERIOD_W'b0;
	 xyolo_shift <= {`SHIFT_W{1'b0}};
	 xyolo_bias <= 1'b0;
	 xyolo_leaky <= 1'b0;
	 xyolo_maxpool <= 1'b0;
	 xyolo_bypass <= 1'b0;
      end else begin
         integer j;
	 //vwrite
         if(vwrite_ext_addr_en) vwrite_ext_addr <= wdata[`IO_ADDR_W-1:0];
         if(vwrite_offset_en) vwrite_offset <= wdata[`IO_ADDR_W/2-1:0];
   	 if(vwrite_len_en) vwrite_len <= wdata[`AXI_LEN_W-1:0];
   	 if(vwrite_size_en) vwrite_size <= wdata[`AXI_SIZE_W-1:0];
         if(vwrite_int_addr_en) vwrite_int_addr <= wdata[`VWRITE_ADDR_W-1:0];
         if(vwrite_iterA_en) vwrite_iterA <= wdata[`MEM_ADDR_W-1:0];
         if(vwrite_perA_en) vwrite_perA <= wdata[`PERIOD_W-1:0];
         if(vwrite_shiftA_en) vwrite_shiftA <= wdata[`MEM_ADDR_W-1:0];
         if(vwrite_incrA_en) vwrite_incrA <= wdata[`MEM_ADDR_W-1:0];
         if(vwrite_startB_en) vwrite_startB <= wdata[`VWRITE_ADDR_W-1:0];
         if(vwrite_dutyB_en) vwrite_dutyB <= wdata[`PERIOD_W-1:0];
         if(vwrite_delayB_en) vwrite_delayB <= wdata[`PERIOD_W-1:0];
         if(vwrite_iterB_en) vwrite_iterB <= wdata[`MEM_ADDR_W-1:0];
         if(vwrite_perB_en) vwrite_perB <= wdata[`PERIOD_W-1:0];
         if(vwrite_shiftB_en) vwrite_shiftB <= wdata[`MEM_ADDR_W-1:0];
         if(vwrite_incrB_en) vwrite_incrB <= wdata[`MEM_ADDR_W-1:0];
	 //vread
   	 if(vread_ext_addr_en) vread_ext_addr <= wdata[`IO_ADDR_W-1:0];
   	 if(vread_offset_en) vread_offset <= wdata[`IO_ADDR_W/2-1:0];
   	 if(vread_len_en) vread_len <= wdata[`AXI_LEN_W-1:0];
         if(vread_int_addr_en) vread_int_addr <= wdata[`W_ADDR_W-1:0];
   	 if(vread_iterA_en) vread_iterA <= wdata[`EXT_ADDR_W-1:0];
	 if(vread_perA_en) vread_perA <= wdata[`EXT_PERIOD_W-1:0];
   	 if(vread_shiftA_en) vread_shiftA <= wdata[`EXT_ADDR_W-1:0];
   	 if(vread_incrA_en) vread_incrA <= wdata[`EXT_ADDR_W-1:0];
	 if(vread_startB_en) vread_startB <= wdata[`MEM_ADDR_W-1:0];
	 if(vread_iterB_en) vread_iterB <= wdata[`MEM_ADDR_W-1:0];
	 if(vread_perB_en) vread_perB <= wdata[`PERIOD_W-1:0];
	 if(vread_shiftB_en) vread_shiftB <= wdata[`MEM_ADDR_W-1:0];
	 if(vread_incrB_en) vread_incrB <= wdata[`MEM_ADDR_W-1:0];
         if(vread_iter2B_en) vread_iter2B <= wdata[`MEM_ADDR_W-1:0];
	 if(vread_per2B_en) vread_per2B <= wdata[`PERIOD_W-1:0];
   	 if(vread_shift2B_en) vread_shift2B <= wdata[`MEM_ADDR_W-1:0];
   	 if(vread_incr2B_en) vread_incr2B <= wdata[`MEM_ADDR_W-1:0];
         if(vread_iter3B_en) vread_iter3B <= wdata[`MEM_ADDR_W-1:0];
	 if(vread_per3B_en) vread_per3B <= wdata[`PERIOD_W-1:0];
   	 if(vread_shift3B_en) vread_shift3B <= wdata[`MEM_ADDR_W-1:0];
   	 if(vread_incr3B_en) vread_incr3B <= wdata[`MEM_ADDR_W-1:0];
	 //xyolo
   	 if(xyolo_iter_en) xyolo_iter <= wdata[`MEM_ADDR_W-1:0];
	 if(xyolo_per_en) xyolo_per <= wdata[`PERIOD_W-1:0];
	 if(xyolo_shift_en) xyolo_shift <= wdata[`SHIFT_W-1:0];
	 if(xyolo_bias_en) xyolo_bias <= wdata[0];
	 if(xyolo_leaky_en) xyolo_leaky <= wdata[0];
	 if(xyolo_maxpool_en) xyolo_maxpool <= wdata[0];
	 if(xyolo_bypass_en) xyolo_bypass <= wdata[0];
      end

   // configurable parameters shadow register
   always @(posedge clk, posedge rst)
      if(rst) begin
	 //vwrite
         vwrite_ext_addr_shadow <= `nSTAGES*`IO_ADDR_W'b0;
         vwrite_ext_addr_pip0 <= `nSTAGES*`IO_ADDR_W'b0;
         vwrite_ext_addr_pip1 <= `nSTAGES*`IO_ADDR_W'b0;
         vwrite_len_shadow <= `AXI_LEN_W'b0;
         vwrite_len_pip0 <= `AXI_LEN_W'b0;
         vwrite_len_pip1 <= `AXI_LEN_W'b0;
         vwrite_size_shadow <= `AXI_SIZE_W'b0;
         vwrite_size_pip0 <= `AXI_SIZE_W'b0;
         vwrite_size_pip1 <= `AXI_SIZE_W'b0;
         vwrite_int_addr_shadow <= `VWRITE_ADDR_W'b0;
         vwrite_int_addr_pip0 <= `VWRITE_ADDR_W'b0;
         vwrite_int_addr_pip1 <= `VWRITE_ADDR_W'b0;
         vwrite_iterA_shadow <= `MEM_ADDR_W'b0;
         vwrite_iterA_pip0 <= `MEM_ADDR_W'b0;
         vwrite_iterA_pip1 <= `MEM_ADDR_W'b0;
         vwrite_perA_shadow <= `PERIOD_W'b0;
         vwrite_perA_pip0 <= `PERIOD_W'b0;
         vwrite_perA_pip1 <= `PERIOD_W'b0;
         vwrite_shiftA_shadow <= `MEM_ADDR_W'b0;
         vwrite_shiftA_pip0 <= `MEM_ADDR_W'b0;
         vwrite_shiftA_pip1 <= `MEM_ADDR_W'b0;
         vwrite_incrA_shadow <= `MEM_ADDR_W'b0;
         vwrite_incrA_pip0 <= `MEM_ADDR_W'b0;
         vwrite_incrA_pip1 <= `MEM_ADDR_W'b0;
         vwrite_startB_shadow <= `VWRITE_ADDR_W'b0;
         vwrite_startB_pip <= `VWRITE_ADDR_W'b0;
         vwrite_dutyB_shadow <= `PERIOD_W'b0;
         vwrite_dutyB_pip <= `PERIOD_W'b0;
         vwrite_delayB_shadow <= `PERIOD_W'b0;
         vwrite_delayB_pip <= `PERIOD_W'b0;
         vwrite_iterB_shadow <= `MEM_ADDR_W'b0;
         vwrite_iterB_pip <= `MEM_ADDR_W'b0;
         vwrite_perB_shadow <= `PERIOD_W'b0;
         vwrite_perB_pip <= `PERIOD_W'b0;
         vwrite_shiftB_shadow <= `MEM_ADDR_W'b0;
         vwrite_shiftB_pip <= `MEM_ADDR_W'b0;
         vwrite_incrB_shadow <= `MEM_ADDR_W'b0;
         vwrite_incrB_pip <= `MEM_ADDR_W'b0;
         vwrite_bypass_shadow <= 1'b0;
	 //vread
         vread_ext_addr_shadow <= `nSTAGES*`IO_ADDR_W'b0;
         vread_len_shadow <= `AXI_LEN_W'b0;
         vread_int_addr_shadow <= {`W_ADDR_W{1'b0}};
         vread_iterA_shadow <= `EXT_ADDR_W'b0;
         vread_perA_shadow <= `EXT_PERIOD_W'b0;
	 vread_shiftA_shadow <= `EXT_ADDR_W'b0;
	 vread_incrA_shadow <= `EXT_ADDR_W'b0;
	 vread_startB_shadow <= `MEM_ADDR_W'b0;
	 vread_startB_pip <= `MEM_ADDR_W'b0;
	 vread_iterB_shadow <= `MEM_ADDR_W'b0;
	 vread_iterB_pip <= `MEM_ADDR_W'b0;
	 vread_perB_shadow <= `PERIOD_W'b0;
	 vread_perB_pip <= `PERIOD_W'b0;
	 vread_shiftB_shadow <= `MEM_ADDR_W'b0;
	 vread_shiftB_pip <= `MEM_ADDR_W'b0;
	 vread_incrB_shadow <= `MEM_ADDR_W'b0;
	 vread_incrB_pip <= `MEM_ADDR_W'b0;
	 vread_iter2B_shadow <= `MEM_ADDR_W'b0;
	 vread_iter2B_pip <= `MEM_ADDR_W'b0;
	 vread_per2B_shadow <= `PERIOD_W'b0;
	 vread_per2B_pip <= `PERIOD_W'b0;
	 vread_shift2B_shadow <= `MEM_ADDR_W'b0;
	 vread_shift2B_pip <= `MEM_ADDR_W'b0;
	 vread_incr2B_shadow <= `MEM_ADDR_W'b0;
	 vread_incr2B_pip <= `MEM_ADDR_W'b0;
	 vread_iter3B_shadow <= `MEM_ADDR_W'b0;
	 vread_iter3B_pip <= `MEM_ADDR_W'b0;
	 vread_per3B_shadow <= `PERIOD_W'b0;
	 vread_per3B_pip <= `PERIOD_W'b0;
	 vread_shift3B_shadow <= `MEM_ADDR_W'b0;
	 vread_shift3B_pip <= `MEM_ADDR_W'b0;
	 vread_incr3B_shadow <= `MEM_ADDR_W'b0;
	 vread_incr3B_pip <= `MEM_ADDR_W'b0;
	 //xyolo
   	 xyolo_iter_shadow <= `MEM_ADDR_W'b0;
   	 xyolo_iter_pip <= `MEM_ADDR_W'b0;
	 xyolo_per_shadow <= `PERIOD_W'b0;
	 xyolo_per_pip <= `PERIOD_W'b0;
	 xyolo_shift_shadow <= {`SHIFT_W{1'b0}};
	 xyolo_shift_pip <= {`SHIFT_W{1'b0}};
	 xyolo_bias_shadow <= 1'b0;
	 xyolo_bias_pip <= 1'b0;
	 xyolo_leaky_shadow <= 1'b0;
	 xyolo_leaky_pip <= 1'b0;
	 xyolo_maxpool_shadow <= 1'b0;
	 xyolo_maxpool_pip <= 1'b0;
	 xyolo_bypass_shadow <= 1'b0;
	 xyolo_bypass_pip <= 1'b0;
      end else if(run) begin
	 //vwrite
         vwrite_ext_addr_pip0 <= vwrite_ext_addr_bus;
         vwrite_ext_addr_pip1 <= vwrite_ext_addr_pip0;
         vwrite_ext_addr_shadow <= vwrite_ext_addr_pip1;
	 vwrite_len_pip0 <= vwrite_len;
	 vwrite_len_pip1 <= vwrite_len_pip0;
	 vwrite_len_shadow <= vwrite_len_pip1;
	 vwrite_size_pip0 <= vwrite_size;
	 vwrite_size_pip1 <= vwrite_size_pip0;
	 vwrite_size_shadow <= vwrite_size_pip1;
	 //XOR ensures ping-pong happens when acessing external mem
	 vwrite_int_addr_pip0 <= {vwrite_int_addr_pip0[`VWRITE_ADDR_W-1] ^ |vwrite_iterA, vwrite_int_addr[`VWRITE_ADDR_W-2:0]};
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
	 vwrite_startB_pip <= {vwrite_startB_pip[`VWRITE_ADDR_W-1] ^ |vwrite_iterA, vwrite_startB[`VWRITE_ADDR_W-2:0]};
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
         vwrite_bypass_shadow <= xyolo_bypass_shadow;
	 //vread
	 vread_ext_addr_shadow <= vread_ext_addr_bus;
	 vread_len_shadow <= vread_len;
	 //XOR ensures ping-pong happens when acessing external mem
	 vread_int_addr_shadow <= {vread_int_addr_shadow[`W_ADDR_W-1] ^ |vread_iterA, vread_int_addr[`W_ADDR_W-2:0]};
	 vread_iterA_shadow <= vread_iterA;
	 vread_perA_shadow <= vread_perA;
	 vread_shiftA_shadow <= vread_shiftA;
	 vread_incrA_shadow <= vread_incrA;
	 vread_startB_pip <= {vread_startB_pip[`MEM_ADDR_W-1] ^ |vread_iterA, vread_startB[`MEM_ADDR_W-2:0]};
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
	 //xyolo
	 xyolo_iter_pip <= xyolo_iter;
	 xyolo_iter_shadow <= xyolo_iter_pip;
	 xyolo_per_pip <= xyolo_per;
	 xyolo_per_shadow <= xyolo_per_pip;
	 xyolo_shift_pip <= xyolo_shift;
	 xyolo_shift_shadow <= xyolo_shift_pip;
	 xyolo_bias_pip <= xyolo_bias;
	 xyolo_bias_shadow <= xyolo_bias_pip;
	 xyolo_leaky_pip <= xyolo_leaky;
	 xyolo_leaky_shadow <= xyolo_leaky_pip;
	 xyolo_maxpool_pip <= xyolo_maxpool;
	 xyolo_maxpool_shadow <= xyolo_maxpool_pip;
	 xyolo_bypass_pip <= xyolo_bypass;
	 xyolo_bypass_shadow <= xyolo_bypass_pip;
      end

   // external address calculation based on base and offset values
   genvar i;
   assign vread_ext_addr_bus[`nSTAGES*`IO_ADDR_W-1 -: `IO_ADDR_W] = vread_ext_addr; //base value
   assign vwrite_ext_addr_bus[`nSTAGES*`IO_ADDR_W-1 -: `IO_ADDR_W] = vwrite_ext_addr; //base value
   generate
      for (i=1; i < `nSTAGES; i=i+1) begin : addrgen_calc

         //instantiate vread 4-stage multiplier
         mul_4stage # (
            .DATA_W(`IO_ADDR_W/2)
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
            .DATA_W(`IO_ADDR_W/2)
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

   //vread internal address generator
   xaddrgen3 vread_addrgenB (
      .clk(clk),
      .rst(rst),
      .run(run_reg),
      .iterations(vread_iterB_shadow),
      .period(vread_perB_shadow),
      .duty(vread_perB_shadow),
      .start(vread_startB_shadow),
      .shift(vread_shiftB_shadow),
      .incr(vread_incrB_shadow),
      .delay(`PERIOD_W'd0),
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
   xaddrgen xyolo_addrgen (
      .clk(clk),
      .rst(rst),
      .init(run_reg),
      .run(run_reg & |xyolo_iter_shadow),
      .pause(1'b0),
      .iterations(xyolo_iter_shadow),
      .period(xyolo_per_shadow),
      .duty(xyolo_per_shadow),
      .start(`MEM_ADDR_W'b0),
      .shift(-xyolo_per_shadow[`MEM_ADDR_W-1:0]),
      .incr(`MEM_ADDR_W'b1),
      .delay(vread_lat),
      .addr(xyolo_addr),
      .mem_en(),
      .done()
   );

   //compute xyolo load wires
   assign ld_acc = (xyolo_addr == {`MEM_ADDR_W{1'd0}});
   assign ld_mp = |mp_cnt;
   assign ld_res = ld_acc1 || xyolo_bypass_shadow;

   //update xyolo registers
   always @ (posedge clk, posedge rst)
      if(rst) begin
	 ld_acc0 <= 1'b0;
         ld_acc1 <= 1'b0;
	 mp_cnt <= 2'b0;
      end else if(run_reg) begin
         ld_acc0 <= 1'b0;
         ld_acc1 <= 1'b0;
         if(xyolo_bypass_shadow)
	   mp_cnt <= 2'd0;
         else
           mp_cnt <= 2'd3;
      end else begin
	 ld_acc0 <= ld_acc;
         ld_acc1 <= ld_acc0;
	 if(ld_res) mp_cnt <= mp_cnt + 1;
      end

   //vwrite internal address generator
   xaddrgen vwrite_addrgenB (
      .clk(clk),
      .rst(rst),
      .init(run_reg),
      .run(run_reg & |vwrite_iterB_shadow),
      .pause(1'b0),
      .iterations(vwrite_iterB_shadow),
      .period(vwrite_perB_shadow),
      .duty(vwrite_dutyB_shadow),
      .start({{`MEM_ADDR_W-`VWRITE_ADDR_W{1'b0}}, vwrite_startB_shadow}),
      .shift(vwrite_shiftB_shadow),
      .incr(vwrite_incrB_shadow),
      .delay(vwrite_delayB_shadow),
      .addr(vwrite_addrB),
      .mem_en(vwrite_enB),
      .done(vwrite_doneB)
   );

   //
   // stages
   //

   // register vread mem read inputs
   always @ (posedge clk) begin
      vread_enB_reg <= vread_enB;
      vread_addrB_reg <= vread_addrB;
   end

   // first stage
   xyolo_write_stage # (
      .DATAPATH_W(DATAPATH_W),
      .DATABUS_W(DATABUS_W)
   ) stage0 (
      .clk(clk),
      .rst(rst),
       //control
      .global_run(run_reg),
      .done(stages_done[0]),
      //internal addrgen
      .vread_enB(vread_enB_reg),
      .vwrite_enB(vwrite_enB),
      .vread_addrB(vread_addrB_reg),
      .vwrite_addrB(vwrite_addrB[`VWRITE_ADDR_W-1:0]),
      //load control
      .ld_acc(ld_acc0),
      .ld_mp(ld_mp),
      .ld_res(ld_res),
      //vread config params
      .vread_ext_addr(vread_ext_addr_shadow[`nSTAGES*`IO_ADDR_W-1 -: `IO_ADDR_W]),
      .vread_int_addr(vread_int_addr_shadow),
      .vread_iterA(vread_iterA_shadow),
      .vread_perA(vread_perA_shadow),
      .vread_shiftA(vread_shiftA_shadow),
      .vread_incrA(vread_incrA_shadow),
      //vwrite config params
      .vwrite_ext_addr(vwrite_ext_addr_shadow[`nSTAGES*`IO_ADDR_W-1 -: `IO_ADDR_W]),
      .vwrite_int_addr(vwrite_int_addr_shadow),
      .vwrite_iterA(vwrite_iterA_shadow),
      .vwrite_perA(vwrite_perA_shadow),
      .vwrite_shiftA(vwrite_shiftA_shadow),
      .vwrite_incrA(vwrite_incrA_shadow),
      .vwrite_bypass(vwrite_bypass_shadow),
      //xyolo config params
      .xyolo_bias(xyolo_bias_shadow),
      .xyolo_leaky(xyolo_leaky_shadow),
      .xyolo_maxpool(xyolo_maxpool_shadow),
      .xyolo_bypass(xyolo_bypass_shadow),
      .xyolo_shift(xyolo_shift_shadow),
      //databus interface
      .databus_ready({vwrite_m_resp[`ready((`nSTAGES-1))], vread_m_resp[`ready((`nSTAGES-1))]}),
      .databus_valid({vwrite_m_req[`valid((`nSTAGES-1))], vread_m_req[`valid((`nSTAGES-1))]}),
      .databus_addr({vwrite_m_req[`address((`nSTAGES-1), `IO_ADDR_W)], vread_m_req[`address((`nSTAGES-1), `IO_ADDR_W)]}),
      .databus_rdata({vwrite_m_resp[`rdata((`nSTAGES-1))], vread_m_resp[`rdata((`nSTAGES-1))]}),
      .databus_wdata({vwrite_m_req[`wdata((`nSTAGES-1))], vread_m_req[`wdata((`nSTAGES-1))]}),
      .databus_wstrb({vwrite_m_req[`wstrb((`nSTAGES-1))], vread_m_req[`wstrb((`nSTAGES-1))]}),
      //input data
      .flow_in_bias(flow_in_bias),
      .flow_in_weight(flow_in_weight)
   );

   //instantiate stages
   generate
     for(i = 1; i < `nSTAGES; i=i+1)  begin : stages

        //check if asking for the same data as previous vread
        wire cond = vread_m_req[`address((`nSTAGES-i), `IO_ADDR_W)] == vread_m_req[`address((`nSTAGES-i-1), `IO_ADDR_W)] && vread_m_req[`valid((`nSTAGES-i))] && vread_m_req[`valid((`nSTAGES-i-1))];
	wire databus_ready_w = cond ? vread_m_resp[`ready((`nSTAGES-i))] : vread_m_resp[`ready((`nSTAGES-i-1))];
	wire [DATABUS_W-1:0] databus_rdata_w = cond ? vread_m_resp[`rdata((`nSTAGES-i))] : vread_m_resp[`rdata((`nSTAGES-i-1))];

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
           .vwrite_enB(vwrite_enB),
           .vread_addrB(vread_addrB_reg),
           .vwrite_addrB(vwrite_addrB[`VWRITE_ADDR_W-1:0]),
           //load control
           .ld_acc(ld_acc0),
           .ld_mp(ld_mp),
           .ld_res(ld_res),
           //vread config params
           .vread_ext_addr(vread_ext_addr_shadow[`nSTAGES*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W]),
           .vread_int_addr(vread_int_addr_shadow),
           .vread_iterA(vread_iterA_shadow),
           .vread_perA(vread_perA_shadow),
           .vread_shiftA(vread_shiftA_shadow),
           .vread_incrA(vread_incrA_shadow),
           //vwrite config params
           .vwrite_ext_addr(vwrite_ext_addr_shadow[`nSTAGES*`IO_ADDR_W-`IO_ADDR_W*i-1 -: `IO_ADDR_W]),
           .vwrite_int_addr(vwrite_int_addr_shadow),
           .vwrite_iterA(vwrite_iterA_shadow),
           .vwrite_perA(vwrite_perA_shadow),
           .vwrite_shiftA(vwrite_shiftA_shadow),
           .vwrite_incrA(vwrite_incrA_shadow),
           .vwrite_bypass(vwrite_bypass_shadow),
           //xyolo config params
           .xyolo_bias(xyolo_bias_shadow),
           .xyolo_leaky(xyolo_leaky_shadow),
           .xyolo_maxpool(xyolo_maxpool_shadow),
           .xyolo_bypass(xyolo_bypass_shadow),
           .xyolo_shift(xyolo_shift_shadow),
      	   //databus interface
      	   .databus_ready({vwrite_m_resp[`ready((`nSTAGES-1-i))], databus_ready_w}),
           .databus_valid({vwrite_m_req[`valid((`nSTAGES-1-i))], vread_m_req[`valid((`nSTAGES-1-i))]}),
           .databus_addr({vwrite_m_req[`address((`nSTAGES-1-i), `IO_ADDR_W)], vread_m_req[`address((`nSTAGES-1-i), `IO_ADDR_W)]}),
           .databus_rdata({vwrite_m_resp[`rdata((`nSTAGES-1-i))], databus_rdata_w}),
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
   merge #(
      .N_MASTERS(`nSTAGES),
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
