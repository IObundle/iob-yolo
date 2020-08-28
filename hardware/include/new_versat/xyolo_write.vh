// Latency
`define XYOLO_WRITE_LAT        	5
`define XYOLO_WRITE_BYPASS_LAT  2

//define vread mem write address width
`define PIXEL_W_ADDR_W         (`PIXEL_ADDR_W-$clog2(DATABUS_W/DATAPATH_W))

// xyolo shift width
`define SHIFT_W                 ($clog2(DATAPATH_W)+1)

// vwrites configuration offsets
`define VWRITE_CONF_EXT_ADDR    0
`define VWRITE_CONF_OFFSET      1
`define VWRITE_CONF_LEN         2
`define VWRITE_CONF_INT_ADDR    3
`define VWRITE_CONF_ITER_A      4
`define VWRITE_CONF_PER_A       5
`define VWRITE_CONF_SHIFT_A     6
`define VWRITE_CONF_INCR_A      7
`define VWRITE_CONF_START_B     8
`define VWRITE_CONF_DUTY_B      9
`define VWRITE_CONF_DELAY_B     10
`define VWRITE_CONF_ITER_B      11
`define VWRITE_CONF_PER_B       12
`define VWRITE_CONF_SHIFT_B     13
`define VWRITE_CONF_INCR_B      14

// vread configuration offsets
`define VREAD_CONF_EXT_ADDR    	15
`define VREAD_CONF_OFFSET    	16
`define VREAD_CONF_PP    	17
`define VREAD_CONF_LEN    	18
`define VREAD_CONF_INT_ADDR    	19
`define VREAD_CONF_ITER_A      	20
`define VREAD_CONF_PER_A       	21
`define VREAD_CONF_SHIFT_A     	22
`define VREAD_CONF_INCR_A      	23
`define VREAD_CONF_START_B     	24
`define VREAD_CONF_ITER_B      	25
`define VREAD_CONF_PER_B       	26
`define VREAD_CONF_SHIFT_B     	27
`define VREAD_CONF_INCR_B      	28
`define VREAD_CONF_ITER2_B      29
`define VREAD_CONF_PER2_B       30
`define VREAD_CONF_SHIFT2_B     31
`define VREAD_CONF_INCR2_B      32
`define VREAD_CONF_ITER3_B      33
`define VREAD_CONF_PER3_B       34
`define VREAD_CONF_SHIFT3_B     35
`define VREAD_CONF_INCR3_B      36

// xyolo configuration offsets
`define XYOLO_CONF_ITER  	37
`define XYOLO_CONF_PER   	38
`define XYOLO_CONF_SHIFT   	39
`define XYOLO_CONF_BIAS   	40
`define XYOLO_CONF_LEAKY   	41
`define XYOLO_CONF_SIGMOID   	42
`define XYOLO_CONF_SIG_MASK   	43
`define XYOLO_CONF_MAXPOOL   	44
`define XYOLO_CONF_BYPASS   	45

// Address
`define XYOLO_WRITE_ADDR_W 	($clog2(`XYOLO_CONF_BYPASS+1))
