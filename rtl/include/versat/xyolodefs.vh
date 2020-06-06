// YOLO config bits:
// input selection bits = 3 * N_W
// iterations = MEM_ADDR_W
// period, delay = PERIOD_W
// shift = SHIFT_W
// bias, leaky, maxpool = 1 bit
`define SHIFT_W ($clog2(`DATAPATH_W)+1)
`define YOLO_CONF_BITS (3*`N_W + `MEM_ADDR_W + 2*`PERIOD_W + `SHIFT_W + 3)

//YOLO configuration offsets
`define YOLO_CONF_SELA    0
`define YOLO_CONF_SELB    1
`define YOLO_CONF_SELC    2
`define YOLO_CONF_ITER    3
`define YOLO_CONF_PER     4
`define YOLO_CONF_DELAY   5
`define YOLO_CONF_SHIFT   6
`define YOLO_CONF_BIAS    7
`define YOLO_CONF_LEAKY   8
`define YOLO_CONF_MAXPOOL 9
`define YOLO_CONF_OFFSET  10

//YOLO latency
`define YOLO_LAT 5
