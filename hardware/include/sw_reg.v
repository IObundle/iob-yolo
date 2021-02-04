//START_TABLE sw_yoloreg
`SWREG_W(START_ADDR,   DDR_ADDR_W, 0) //Main memory address for Tiny Yolo V3 binary file.
`SWREG_W(IMG_IN_ADDR,  DDR_ADDR_W, 0) //Main memory address for input image.
`SWREG_W(IMG_OUT_ADDR, DDR_ADDR_W, 0) //Main memory address for output image.
`SWREG_W(RUN,                   1, 0) //Signal Tiny Yolo V3 execution start.
`SWREG_R(READY,                 1, 1) //Yolo Core is ready to start an execution (1) or not (0)
