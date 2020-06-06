// MEM port config bits
// input selection  bits = N_W
// start width = MEM_ADDR_W
// incr width = 3*MEM_ADDR_W
// iterations = 3*MEM_ADDR_W
// shift width = 3*MEM_ADDR_W
// period = 3*PERIOD_W
// duty = PERIOD_W
// delay width = PERIOD_W
// ext, in_wr = 1 bit

// Number of config bits
`define MEMP_CONF_BITS (`N_W + 10*`MEM_ADDR_W + 5*`PERIOD_W + 2)

//MEM configuration offsets
`define MEMP_CONF_ITER 		0
`define MEMP_CONF_PER 		1
`define MEMP_CONF_DUTY 		2
`define MEMP_CONF_SEL 		3
`define MEMP_CONF_START 	4
`define MEMP_CONF_SHIFT 	5
`define MEMP_CONF_INCR 		6
`define MEMP_CONF_DELAY 	7
`define MEMP_CONF_EXT 		8
`define MEMP_CONF_IN_WR 	9
`define MEMP_CONF_ITER2 	10
`define MEMP_CONF_PER2 		11
`define MEMP_CONF_SHIFT2 	12
`define MEMP_CONF_INCR2 	13
`define MEMP_CONF_ITER3 	14
`define MEMP_CONF_PER3 		15
`define MEMP_CONF_SHIFT3 	16
`define MEMP_CONF_INCR3 	17
`define MEMP_CONF_OFFSET 	18

//MEM latency
`define MEMP_LAT 3
