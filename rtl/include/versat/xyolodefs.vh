// YOLO config bits:
// input selection bits = 3 * N_W
// iterations = MEM_ADDR_W
// period, delay = PERIOD_W
// shift = SHIFT_W
// accIN, accOUT, bias, leaky = 1 bit
`define SHIFT_W ($clog2(`DATAPATH_W)+1)
`define YOLO_CONF_BITS (3*`N_W + `MEM_ADDR_W + 2*`PERIOD_W + `SHIFT_W + 4)

//YOLO configuration offsets
`define YOLO_CONF_SELA    0
`define YOLO_CONF_SELB    1
`define YOLO_CONF_SELC    2
`define YOLO_CONF_ITER    3
`define YOLO_CONF_PER     4
`define YOLO_CONF_DELAY   5
`define YOLO_CONF_SHIFT   6
`define YOLO_CONF_ACCIN   7
`define YOLO_CONF_ACCOUT  8
`define YOLO_CONF_BIAS    9
`define YOLO_CONF_LEAKY   10
`define YOLO_CONF_OFFSET  11

//YOLO latency
`define YOLO_LAT 5
