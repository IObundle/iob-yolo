# iob-SoC-Yolo

Repository for the implementation of an object detection application based on the YOLOv3 network, using the iob-soc and iob-versat submodules.

# Software Tests
## Utility Software
- **bootloader:** firmware to use as BOOT - waits to receive another firmware via UART
- **ld-sw:** program that runs on the machine and sends the firmware received by the bootloarder

## Test Software
- **ddr_test:** writes N positions of DDR and reads them. Goal: verify correct functioning of the DDR.
- **eth_repeat:** wait for a message from pc and send it back. Goal: verify Ethernet is working.
- **eth_weights:** receive an image and weights for the tiny-yolo CNN from pc, each message is sent back to the pc. Goal: verify Ethernet can receive and send back many frames without blocking.
- **eth_ddr:** same as eth_weights, with the addition that the received data is also saved to DDR. Goal: simultaneous verification of DDR and Ethernet working with lot of data.
- **yolo_sw:** reception of the input image and weights and full execution of the tiny-yolo CNN in RISCV. Goal: baseline for hardware acceleration of tiny-yolo.
- **yolo_sw_full:** Full execution of the YOLO pipeline (image resize, tiny-yolo CNN and detections). Goal: baseline for hardware acceleration of full YOLO pipeline.
