SIM_DIR = simulation/xcsim

FPGA_DIR = fpga/xilinx/AES-KU040-DB-G

sim:
	make -C $(SIM_DIR) 

fpga:
	make -C $(FPGA_DIR)

clean: 
	make -C  $(SIM_DIR) clean
	make -C fpga/xilinx/AES-KU040-DB-G clean

.PHONY: sim fpga clean
