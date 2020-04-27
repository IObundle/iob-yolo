#simulation baud rate
BAUD := 10000000
FREQ := 100000000

#software paths
FIRM_DIR = ../../software/$(TEST)
BOOT_DIR = ../../software/bootloader
PYTHON_DIR := ../../submodules/iob-soc/software/python
TB_DIR := ../../rtl/testbench
FILES_DIR = ../../../

#hardware paths
RTL_DIR = ../../rtl
RTL_DIR_IOB_SOC = ../../submodules/iob-soc/rtl
RISCV_DIR = ../../submodules/iob-soc/submodules/iob-rv32
UART_DIR = ../../submodules/iob-soc/submodules/iob-uart
FIFO_DIR = ../../submodules/iob-soc/submodules/fifo
CACHE_DIR = ../../submodules/iob-soc/submodules/iob-cache
AXI_RAM = ../../submodules/iob-soc/submodules/axi-mem
ETH_DIR = ../../submodules/iob-eth
TIMER_DIR = ../../submodules/iob-timer
VERSAT_DIR = ../../submodules/iob-versat

#hardware includes
HW_INCLUDE = -incdir $(RTL_DIR)/include -incdir $(UART_DIR)/rtl/include -incdir $(CACHE_DIR)/rtl/header -incdir $(ETH_DIR)/rtl/include -incdir $(TIMER_DIR) -incdir $(VERSAT_DIR)/rtl/include

#hardware defines
ifeq ($(VCD),1)
   HW_DEFINE = -define VCD
endif
HW_DEFINE += -define PROG_SIZE=$(shell wc -c ../../software/$(TEST)/firmware.bin | head -n1 | cut -d " " -f1)
HW_DEFINE += -define UART_BAUD_RATE=$(BAUD)
HW_DEFINE += -define UART_CLK_FREQ=$(FREQ)
HW_DEFINE += -define SIM
ifeq ($(LOOPBACK),1)
    HW_DEFINE += -define LOOPBACK
endif
ifeq ($(XILINX),1)
    HW_DEFINE += -define XILINX
else
    HW_DEFINE += -define ALTERA
endif
HW_DEFINE += -define FILES_DIR=$(FILES_DIR)

#hardware sources
VSRC =  $(RISCV_DIR)/picorv32.v $(UART_DIR)/rtl/src/iob-uart.v $(FIFO_DIR)/afifo.v $(RTL_DIR)/testbench/$(TEST)_tb.v  $(RTL_DIR)/src/*.v $(RTL_DIR_IOB_SOC)/src/memory/behav/*.v $(CACHE_DIR)/rtl/src/gen_mem_reg.v $(CACHE_DIR)/rtl/src/iob-cache.v $(AXI_RAM)/rtl/axi_ram.v $(ETH_DIR)/rtl/src/*.v $(RTL_DIR_IOB_SOC)/src/int_mem.v $(RTL_DIR_IOB_SOC)/src/iob_generic_interconnect.v $(TIMER_DIR)/*.v $(VERSAT_DIR)/rtl/src/*.v $(VERSAT_DIR)/submodules/mem/tdp_mem/*.v

#simulator flags
CFLAGS = -errormax 15 -status -update -linedebug -sv $(HW_DEFINE) $(HW_INCLUDE)
EFLAGS = -errormax 15 -access +wc -status
SFLAGS = -errormax 15 -status -CMDFILE $(RTL_DIR)/testbench

#simulate
all: default_rmac firmware.hex boot.hex $(VSRC)
	VSRC="$(VSRC)" CFLAGS="$(CFLAGS)" EFLAGS="$(EFLAGS)" SFLAGS="$(SFLAGS)" TEST=$(TEST) ./run_ncsim.sh

#make firmware
firmware.hex:
	make -C $(FIRM_DIR) BAUD=$(BAUD) FREQ=$(FREQ) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX) SIM=1
	cp $(FIRM_DIR)/progmem.hex .
	cp $(FIRM_DIR)/firmware.hex .
	$(PYTHON_DIR)/hex_split.py firmware

#make boot loader
boot.hex:
	make -C $(BOOT_DIR) BAUD=$(BAUD) FREQ=$(FREQ) TEST=$(TEST)
	cp $(BOOT_DIR)/boot.hex .
	$(PYTHON_DIR)/hex_split.py boot
	cp boot.hex boot.dat

#insert default RMAC for ethernet module
default_rmac:
	sed -i "/ETH_RMAC_ADDR/d" $(ETH_DIR)/c-driver/iob-eth.h $(ETH_DIR)/rtl/include/iob_eth_defs.vh
	echo "#define ETH_RMAC_ADDR 0x0123456789AB" >> $(ETH_DIR)/c-driver/iob-eth.h
	echo "\`define ETH_RMAC_ADDR 48'h0123456789AB" >> $(ETH_DIR)/rtl/include/iob_eth_defs.vh

clean:
	@rm -f *# *~ *.vcd *.dat *.hex firmware.bin xmsim.key *.log *.txt
	@rm -rf xcelium.d
	make -C $(FIRM_DIR) clean
	make -C $(BOOT_DIR) clean

.PHONY: all clean
