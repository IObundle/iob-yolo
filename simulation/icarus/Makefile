#simulation baud rate
BAUD := 30000000
FREQ := 100000000

#file paths
ROOT_DIR := ../..
SUBMODULES_DIR := $(ROOT_DIR)/submodules
IOB_SOC_DIR := $(SUBMODULES_DIR)/iob-soc
IOB_SOC_SUBMODULES_DIR := $(IOB_SOC_DIR)/submodules

#software paths
FIRM_DIR := $(ROOT_DIR)/software/$(TEST)
BOOT_DIR := $(ROOT_DIR)/software/bootloader
PYTHON_DIR := $(IOB_SOC_DIR)/software/python
TB_DIR := $(ROOT_DIR)/rtl/testbench
FILES_DIR := $(ROOT_DIR)/../

#hardware paths
RTL_DIR := $(ROOT_DIR)/rtl
RTL_DIR_IOB_SOC := $(IOB_SOC_DIR)/rtl
RISCV_DIR := $(IOB_SOC_SUBMODULES_DIR)/iob-rv32
UART_DIR := $(IOB_SOC_SUBMODULES_DIR)/iob-uart
FIFO_DIR := $(IOB_SOC_SUBMODULES_DIR)/fifo
CACHE_DIR := $(IOB_SOC_SUBMODULES_DIR)/iob-cache
AXI_RAM_DIR := $(IOB_SOC_SUBMODULES_DIR)/axi-mem
ETH_DIR := $(SUBMODULES_DIR)/iob-eth
TIMER_DIR := $(SUBMODULES_DIR)/iob-timer
VERSAT_DIR := $(SUBMODULES_DIR)/iob-versat

#hw includes
HW_INCLUDE := -I. -I$(RTL_DIR)/include -I$(UART_DIR)/rtl/include -I$(CACHE_DIR)/rtl/header -I$(ETH_DIR)/rtl/include -I$(TIMER_DIR) -I$(VERSAT_DIR)/rtl/include

#testbench defines
TB_DEFINE = -DPROG_SIZE=$(shell wc -c $(FIRM_DIR)/firmware.bin | head -n1 | cut -d " " -f1)
TB_DEFINE += -DUART_BAUD_RATE=$(BAUD)
TB_DEFINE += -DUART_CLK_FREQ=$(FREQ)
TB_DEFINE += -DSIM
ifeq ($(LOOPBACK),1)
    TB_DEFINE += -D LOOPBACK
endif
ifeq ($(XILINX),1)
    TB_DEFINE += -D XILINX
else
    TB_DEFINE += -D ALTERA
endif
TB_DEFINE += -D FILES_DIR=$(FILES_DIR)

#icarus verilog simulator
VLOG = iverilog -W all -g2005-sv
ifeq ($(VCD),1)
   VLOG += -D VCD
endif

#hardware sources
VSRC = $(RISCV_DIR)/picorv32.v $(UART_DIR)/rtl/src/iob-uart.v $(FIFO_DIR)/afifo.v $(TB_DIR)/$(TEST)_tb.v  $(RTL_DIR)/src/*.v $(RTL_DIR_IOB_SOC)/src/memory/behav/*.v $(CACHE_DIR)/rtl/src/gen_mem_reg.v $(CACHE_DIR)/rtl/src/iob-cache.v $(AXI_RAM_DIR)/rtl/axi_ram.v $(ETH_DIR)/rtl/src/*.v $(RTL_DIR_IOB_SOC)/src/int_mem.v $(RTL_DIR_IOB_SOC)/src/iob_generic_interconnect.v $(TIMER_DIR)/*.v $(VERSAT_DIR)/rtl/src/*.v $(VERSAT_DIR)/submodules/mem/tdp_mem/*.v

all: default_rmac $(VSRC) firmware boot
	$(VLOG) $(HW_INCLUDE) $(TB_DEFINE) $(VSRC)
	./a.out

firmware:
	make -C $(FIRM_DIR) BAUD=$(BAUD) FREQ=$(FREQ) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX) SIM=1
	cp $(FIRM_DIR)/progmem.hex .
	cp $(FIRM_DIR)/firmware.hex .
	$(PYTHON_DIR)/hex_split.py firmware

boot:
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
	@rm -f *# *~ *.vcd *.dat *.hex *.bin *.vh ./a.out
	make -C $(BOOT_DIR) clean --no-print-directory
	make -C $(FIRM_DIR) clean --no-print-directory

.PHONY: all boot firmware clean
