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

all:
	@echo "options: make [sim | fpga | ld-sw | ld-hw | clean]"

sim:
	make -C $(SIM_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

fpga:
	make -C $(FPGA_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-sw:
	make -C $(LD_SW_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

ld-hw:
	make -C $(FPGA_DIR) ld-hw

clean:
	make -C $(SIM_DIR) clean TEST=$(TEST)
	make -C $(FPGA_DIR) clean TEST=$(TEST)
	make -C $(LD_SW_DIR) clean TEST=$(TEST)

.PHONY: sim fpga clean
