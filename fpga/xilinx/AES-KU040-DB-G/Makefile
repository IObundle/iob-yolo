#hardware paths
RTL_DIR = ../../../rtl
RTL_DIR_IOB_SOC = ../../../submodules/iob-soc/rtl
RISCV_DIR = ../../../submodules/iob-soc/submodules/iob-rv32
UART_DIR = ../../../submodules/iob-soc/submodules/iob-uart
FIFO_DIR = ../../../submodules/iob-soc/submodules/fifo
ETH_DIR = ../../../submodules/iob-eth

#software paths
FIRM_DIR := ../../../software/$(TEST)
BOOT_DIR := ../../../software/bootloader
PYTHON_DIR := ../../../submodules/iob-soc/software/python

#.bit destination
REMOTE := ${USER}@iobundle.ddns.net:sandbox/iob-soc-yolo/fpga/xilinx/AES-KU040-DB-G
PORT := -P 1418

#implementation parameters
BAUD := 115200
FREQ := 100000000

ifneq ($(shell grep USE_DDR $(RTL_DIR)/include/system.vh | grep -v "//"),)
	USE_DDR = USE_DDR
endif
ifeq ($(LOOPBACK),1)
	DO_LOOPBACK = LOOPBACK	
endif

VSRC = verilog/top_system.v $(RTL_DIR)/include/*.vh $(RTL_DIR_IOB_SOC)/src/memory/behav/*.v $(RTL_DIR)/src/*.v $(RISCV_DIR)/picorv32.v $(UART_DIR)/rtl/src/iob-uart.v $(FIFO_DIR)/afifo.v $(ETH_DIR)/rtl/src/*.v $(RTL_DIR_IOB_SOC)/src/int_mem.v $(RTL_DIR_IOB_SOC)/src/iob_generic_interconnect.v

# work-around for http://svn.clifford.at/handicraft/2016/vivadosig11
export RDI_VERBOSE = False

all: synth_system.bit
	#scp $(PORT) $< $(REMOTE)

synth_system.bit: *.xdc $(VSRC) firmware.hex boot.hex
	rm -f $@.log
	vivado -nojournal -log $@.log -mode batch -source synth_system.tcl -tclargs $(USE_DDR) $(DO_LOOPBACK)
	-grep -B4 -A10 'CLB LUTs' $@.log
	-grep -B1 -A9 ^Slack $@.log && echo

firmware.hex: $(FIRM_DIR)/firmware.hex
	cp $< .
	cp $(FIRM_DIR)/progmem.hex .
	$(PYTHON_DIR)/hex_split.py firmware

$(FIRM_DIR)/firmware.hex: FORCE
	make -C $(FIRM_DIR) BAUD=$(BAUD) FREQ=$(FREQ) LOOPBACK=$(LOOPBACK)

boot.hex: $(BOOT_DIR)/boot.hex
	cp $(BOOT_DIR)/boot.hex .
	$(PYTHON_DIR)/hex_split.py boot
	cp boot.hex boot.dat

$(BOOT_DIR)/boot.hex: FORCE
	make -C $(BOOT_DIR) BAUD=$(BAUD) FREQ=$(FREQ)

ld-hw:
	vivado -nojournal -log $@.log -mode batch -source $@.tcl

clean:
	@rm -rf .Xil/ *.hex *.dat *.map synth_*.log 
	@rm -rf *~ \#*# *#  ../rtl/*~ ../rtl/\#*# ../rtl/*# ./rtl/
	@rm -rf synth_*.mmi synth_*.bit synth_system*.v *.vcd *_tb 
	@rm -rf table.txt tab_*/ *webtalk* *.jou *.log
	@rm -rf xelab.* xsim[._]* xvlog.* uart_loader
	@rm -rf *.ltx fsm_encoding.os
	make -C $(FIRM_DIR) clean --no-print-directory
	make -C $(BOOT_DIR) clean --no-print-directory

.PHONY: all clean ld-hw FORCE
