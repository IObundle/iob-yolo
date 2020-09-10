// Latency
`define XYOLO_WRITE_LAT        	6
`define XYOLO_WRITE_BYPASS_LAT  2

//define vread mem write address width
`define PIXEL_W_ADDR_W         (`PIXEL_ADDR_W-$clog2(DATABUS_W/DATAPATH_W))
`define PIXEL_INT_ADDR_W       (`PIXEL_ADDR_W-$clog2(`nYOLOmacs))
`define IX_W_ADDR_W	       (`IX_ADDR_W-$clog2(DATABUS_W/DATAPATH_W))

// xyolo shift width
`define SHIFT_W                 ($clog2(DATAPATH_W)+1)

// vwrites configuration offsets
`define VWRITE_CONF_EXT_ADDR    0
`define VWRITE_CONF_OFFSET      1
`define VWRITE_CONF_INT_ADDR    2
`define VWRITE_CONF_ITER_A      3
`define VWRITE_CONF_PER_A       4
`define VWRITE_CONF_SHIFT_A     5
`define VWRITE_CONF_INCR_A      6
`define VWRITE_CONF_START_B     7
`define VWRITE_CONF_DUTY_B      8
`define VWRITE_CONF_DELAY_B     9
`define VWRITE_CONF_ITER_B      10
`define VWRITE_CONF_PER_B       11
`define VWRITE_CONF_SHIFT_B     12
`define VWRITE_CONF_INCR_B      13

// vread configuration offsets
`define VREAD_CONF_EXT_ADDR    	14
`define VREAD_CONF_OFFSET    	15
`define VREAD_CONF_PP    	16
`define VREAD_CONF_INT_ADDR    	17
`define VREAD_CONF_ITER_A      	18
`define VREAD_CONF_PER_A       	19
`define VREAD_CONF_SHIFT_A     	20
`define VREAD_CONF_INCR_A      	21
`define VREAD_CONF_START_B     	22
`define VREAD_CONF_ITER_B      	23
`define VREAD_CONF_PER_B       	24
`define VREAD_CONF_SHIFT_B     	25
`define VREAD_CONF_INCR_B      	26
`define VREAD_CONF_ITER2_B      27
`define VREAD_CONF_PER2_B       28
`define VREAD_CONF_SHIFT2_B     29
`define VREAD_CONF_INCR2_B      30
`define VREAD_CONF_ITER3_B      31
`define VREAD_CONF_PER3_B       32
`define VREAD_CONF_SHIFT3_B     33
`define VREAD_CONF_INCR3_B      34

// ix configuration offsets
`define VREAD_CONF_EXT 		35
`define IX_CONF_EXT_ADDR 	36
`define IX_CONF_ITER_A  	37
`define IX_CONF_PER_A  		38
`define IX_CONF_INCR_A  	39

// xyolo configuration offsets
`define XYOLO_CONF_ITER  	40
`define XYOLO_CONF_PER   	41
`define XYOLO_CONF_SHIFT   	42
`define XYOLO_CONF_BIAS   	43
`define XYOLO_CONF_LEAKY   	44
`define XYOLO_CONF_SIGMOID   	45
`define XYOLO_CONF_SIG_MASK   	46
`define XYOLO_CONF_MAXPOOL   	47
`define XYOLO_CONF_BYPASS   	48
`define XYOLO_CONF_BYPASS_ADD  	49

// Address
`define XYOLO_WRITE_ADDR_W 	($clog2(`XYOLO_CONF_BYPASS_ADD+1))
