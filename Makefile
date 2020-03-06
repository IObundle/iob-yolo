SIM_DIR = simulation/ncsim
FPGA_DIR = fpga/xilinx/AES-KU040-DB-G
TEST = eth_repeat

all:
	@echo "options: make [sim | fpga | clean]"

sim:
	make -C $(SIM_DIR) TEST=$(TEST)

fpga:
	make -C $(FPGA_DIR) TEST=$(TEST)

clean:
	make -C $(SIM_DIR) clean TEST=$(TEST)
	make -C $(FPGA_DIR) clean TEST=$(TEST)

.PHONY: sim fpga clean
