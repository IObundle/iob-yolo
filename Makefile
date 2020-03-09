<<<<<<< HEAD
SIM_DIR = simulation/xcsim

FPGA_DIR = fpga/xilinx/AES-KU040-DB-G

sim:
	make -C $(SIM_DIR) 

fpga:
	make -C $(FPGA_DIR)

clean: 
	make -C  $(SIM_DIR) clean
	make -C fpga/xilinx/AES-KU040-DB-G clean
=======
#configurable parameters
TEST = eth_repeat
LOOPBACK = 0
XILINX = 1

#directories
SIM_DIR = simulation/ncsim
LD_SW_DIR = software/ld-sw
ifeq ($(XILINX),1)
   FPGA_DIR = fpga/xilinx/AES-KU040-DB-G
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
	make -C $(SIM_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

fpga:
	make -C $(FPGA_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-sw:
	make -C $(LD_SW_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-hw:
	make -C $(FPGA_DIR) ld-hw

ld-eth:
	@source /opt/pyeth/bin/activate; python $(FIRM_DIR)/eth_comm.py;

clean:
	make -C $(SIM_DIR) clean TEST=$(TEST)
	make -C $(FPGA_DIR) clean TEST=$(TEST)
	make -C $(LD_SW_DIR) clean TEST=$(TEST)
>>>>>>> 7c2044af785495e83d71e5b5d5eb59b2fc7bd2cc

.PHONY: sim fpga clean
