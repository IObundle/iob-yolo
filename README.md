# iob-SoC-Yolo

Repository for the implementation of an object detection application based on the YOLOv3 network, using the iob-soc and iob-versat submodules.

# Software Tests
## Utility Software
- **bootloader:** firmware to use as BOOT - waits to receive another firmware via UART
- **console:** program that runs on the machine, sends the firmware received by the bootloader and handles UART communication

## Test Software
- **ddr_test:** writes N positions of DDR and reads them. Goal: verify correct functioning of the DDR.
- **eth_repeat:** wait for a message from pc and send it back. Goal: verify Ethernet is working.
- **eth_weights:** receive an image and weights for the tiny-yolo CNN from pc, each message is sent back to the pc. Goal: verify Ethernet can receive and send back many frames without blocking.
- **eth_ddr:** same as eth_weights, with the addition that the received data is also saved to DDR. Goal: simultaneous verification of DDR and Ethernet working with lot of data.
- **yolo_sw:** reception of the input image and weights and full execution of the tiny-yolo CNN in RISCV. Goal: baseline for hardware acceleration of tiny-yolo.
- **yolo_sw_full:** Full execution of the YOLO pipeline (image resize, tiny-yolo CNN and detections). Goal: baseline for hardware acceleration of full YOLO pipeline.
- **versat_test:** performs one 3D convolution with 5 input FMs of 5x5 and kernels of 3x3. Goal: verify deep_versat pipeline is working.
- **yolo_hw_full:** Full execution of YOLO pipeline using versat. Goal: adapt yolo-sw code to be easily accelerated by versat.
- **new_versat_test:** Computing yolov3-tiny layer 1 output with nSTAGES of nYOLOvect each. Goal: verify new versat is working.
- **dma_w_test:** Transfering 16 words of 256 bits each from vwrite to DDR via AXI-DMA (**only works in simulation**). Goal: verify AXI-DMA only-write is working.
- **dma_r_test:** Transfering 7 words of 256 bits each from DDR to vread via AXI-DMA (**only works in simulation**). Goal: verify AXI-DMA only-read is working.
- **yolo_hw:** Execution of yolov3-tiny CNN in new_versat. Goal: accelerate yolo with new versat system.
- **pre_cnn:** Performs input image resize before CNN. Goal: accelerate pre_cnn with new versat.
- **pos_cnn:** Performs detection filtering and box/label display after CNN.
- **tiny_yolov3:** Execution of full yolov3-tiny pipeline (pre-CNN, CNN and pos-CNN) in new_versat. Goal: run all pipeline in 30 fps.


# FPGA execution
## Python Dependencies
- Pillow
- numpy

## Input files
- `../tiny_yolov3_in_x4.bin`
- `../tiny_yolov3_out.bin`

## Target launch order
- `make fpga-load` on remote machine (or ld-hw locally)
- `make ld-sw` locally (launches console) 
- `make ld-eth` locally in parallel with `make ld-sw` (send data and receive results)
