SIM_DIR = simulation/ncsim
FPGA_DIR = fpga/xilinx/AES-KU040-DB-G
LD_SW_DIR = software/ld-sw

TEST = eth_repeat
LOOPBACK = 0

all:
	@echo "options: make [sim | fpga | ld-sw | ld-hw | clean]"

sim:
	make -C $(SIM_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK)

fpga:
	make -C $(FPGA_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK)

ld-sw:
	make -C $(LD_SW_DIR) TEST=$(TEST) LOOPBACK=$(LOOPBACK)

ld-hw:
	make -C $(FPGA_DIR) ld-hw

clean:
	make -C $(SIM_DIR) clean TEST=$(TEST)
	make -C $(FPGA_DIR) clean TEST=$(TEST)
	make -C $(LD_SW_DIR) clean TEST=$(TEST)

.PHONY: sim fpga clean
