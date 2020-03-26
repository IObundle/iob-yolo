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
ETH_DIR = submodules/iob-eth

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

sim: rmac
	make -C $(SIM_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX) VCD=$(VCD)

fpga: rmac
	make -C $(FPGA_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-sw: rmac
	make -C $(LD_SW_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-hw:
	make -C $(FPGA_DIR) ld-hw

ld-eth:
	$(eval INTERFACE := $(shell ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep ^"en" | awk '{print $$1}' | sed 's/://g'))
	$(eval RMAC := $(shell ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep -A 1 ^"en" | grep "ether" | awk '{print $$2}' | sed 's/://g'))
	@source /opt/pyeth/bin/activate; python $(FIRM_DIR)/eth_comm.py $(INTERFACE) $(RMAC);

rmac:
	sed -i "/ETH_RMAC_ADDR/d" $(ETH_DIR)/c-driver/iob-eth.h $(ETH_DIR)/rtl/include/iob_eth_defs.vh
	ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep -A 1 ^"en" | grep "ether" | awk '{print $$2}' | sed 's/://g' | sed "s/^/#define ETH_RMAC_ADDR 0x/" >> $(ETH_DIR)/c-driver/iob-eth.h
	ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep -A 1 ^"en" | grep "ether" | awk '{print $$2}' | sed 's/://g' | sed "s/^/\`define ETH_RMAC_ADDR 48'h/" >> $(ETH_DIR)/rtl/include/iob_eth_defs.vh


clean:
	make -C $(SIM_DIR) clean TEST=$(TEST)
	make -C $(FPGA_DIR) clean TEST=$(TEST)
	make -C $(LD_SW_DIR) clean TEST=$(TEST)
	sed -i "/ETH_RMAC_ADDR/d" $(ETH_DIR)/c-driver/iob-eth.h $(ETH_DIR)/rtl/include/iob_eth_defs.vh


.PHONY: sim fpga clean
