// Latency
`define XYOLO_READ_LAT              1

// Configuration offsets
`define XYOLO_READ_CONF_EXT_ADDR    0
`define XYOLO_READ_CONF_OFFSET      1
`define XYOLO_READ_CONF_INT_ADDR    2
`define XYOLO_READ_CONF_ITER_A      3
`define XYOLO_READ_CONF_PER_A       4
`define XYOLO_READ_CONF_SHIFT_A     5
`define XYOLO_READ_CONF_INCR_A      6
`define XYOLO_READ_CONF_ITER_B      7
`define XYOLO_READ_CONF_PER_B       8
`define XYOLO_READ_CONF_START_B     9
`define XYOLO_READ_CONF_SHIFT_B     10
`define XYOLO_READ_CONF_INCR_B      11

// Address
`define XYOLO_READ_ADDR_W	    ($clog2(`XYOLO_READ_CONF_INCR_B))
