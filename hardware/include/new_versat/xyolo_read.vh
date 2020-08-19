// Latency
`define XYOLO_READ_LAT              4

//define vread mem write address width
`define WEIGHT_W_ADDR_W		    (`WEIGHT_ADDR_W-$clog2(DATABUS_W/DATAPATH_W))

// Configuration offsets
`define XYOLO_READ_CONF_EXT_ADDR    0
`define XYOLO_READ_CONF_OFFSET      1
`define XYOLO_READ_CONF_LEN         2
`define XYOLO_READ_CONF_INT_ADDR    3
`define XYOLO_READ_CONF_ITER_A      4
`define XYOLO_READ_CONF_PER_A       5
`define XYOLO_READ_CONF_SHIFT_A     6
`define XYOLO_READ_CONF_INCR_A      7
`define XYOLO_READ_CONF_ITER_B      8
`define XYOLO_READ_CONF_PER_B       9
`define XYOLO_READ_CONF_START_B     10
`define XYOLO_READ_CONF_SHIFT_B     11
`define XYOLO_READ_CONF_INCR_B      12
`define BIAS_CONF_EXT_ADDR    	    13
`define BIAS_CONF_INT_ADDR          14
`define BIAS_CONF_START_B           15

// Address
`define XYOLO_READ_ADDR_W	    ($clog2(`BIAS_CONF_START_B+1))
