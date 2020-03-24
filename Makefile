#configurable parameters
TEST = yolo_sw
LOOPBACK = 0
XILINX = 1
VCD = 0

#directories
SIM_DIR = simulation/ncsim
LD_SW_DIR = software/ld-sw
ifeq ($(XILINX),1)
   FPGA_DIR = fpga/xilinx/AES-KU040-DB-G
else
   FPGA_DIR = fpga/intel/CYCLONEV-GT-DK
endif
FIRM_DIR = software/$(TEST)

#Run source inside Makefile
SHELL := /bin/bash

all:
	@echo "options: make [sim | fpga | ld-sw | ld-hw | ld-eth | clean]"
	@echo "sim    -> run simulation"
	@echo "fpga   -> generate bitstream"
	@echo "ld-sw  -> read from serial port and, if case, send program through uart"
	@echo "ld-hw  -> send bitstream to FPGA"
	@echo "ld-eth -> communicate through FPGA via Ethernet"
	@echo "clean  -> clean repo"

sim:
	make -C $(SIM_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX) VCD=$(VCD)

fpga:
	make -C $(FPGA_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-sw:
	make -C $(LD_SW_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-hw:
	make -C $(FPGA_DIR) ld-hw

ld-eth:
	@source /opt/pyeth/bin/activate; python $(FIRM_DIR)/eth_comm.py $(XILINX);

clean:
	make -C $(SIM_DIR) clean TEST=$(TEST)
	make -C $(FPGA_DIR) clean TEST=$(TEST)
	make -C $(LD_SW_DIR) clean TEST=$(TEST)

.PHONY: sim fpga clean
