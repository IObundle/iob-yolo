SIM_DIR = simulation/ncsim
FPGA_DIR = fpga/xilinx/AES-KU040-DB-G
TEST = eth_repeat

sim:
	make -C $(SIM_DIR) TEST=$(TEST)

fpga:
	make -C $(FPGA_DIR) TEST=$(TEST)

clean:
	make -C  $(SIM_DIR) clean TEST=$(TEST)
	#make -C fpga/xilinx/AES-KU040-DB-G clean

.PHONY: sim fpga clean
