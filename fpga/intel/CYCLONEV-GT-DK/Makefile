#hardware paths
RTL_DIR = ../../../rtl
RTL_DIR_IOB_SOC = ../../../submodules/iob-soc/rtl
RISCV_DIR = ../../../submodules/iob-soc/submodules/iob-rv32
UART_DIR = ../../../submodules/iob-soc/submodules/iob-uart
FIFO_DIR = ../../../submodules/iob-soc/submodules/fifo
ETH_DIR = ../../../submodules/iob-eth
TIMER_DIR = ../../../submodules/iob-timer

#software paths
FIRM_DIR := ../../../software/$(TEST)
BOOT_DIR := ../../../software/bootloader
PYTHON_DIR := ../../../submodules/iob-soc/software/python
LD_SW_DIR := ../../../software/ld-sw

REMOTE_HOST := iobundle.ddns.net
REMOTE_PORT := 1415
REMOTE_FPGA_DIR := ./sandbox/pm-iob-soc-yolo/fpga/intel/CYCLONEV-GT-DK

BAUD := 115200
FREQ := 100000000

ifneq ($(shell grep USE_DDR $(RTL_DIR)/include/system.vh | grep -v "//"),)
USE_DDR = USE_DDR
endif

VSRC = top_system.v $(RTL_DIR)/include/*.vh $(RTL_DIR_IOB_SOC)/src/memory/behav/*.v $(RTL_DIR)/src/*.v $(RISCV_DIR)/picorv32.v $(UART_DIR)/rtl/src/iob-uart.v $(FIFO_DIR)/afifo.v $(ETH_DIR)/rtl/src/*.v $(RTL_DIR_IOB_SOC)/src/int_mem.v $(RTL_DIR_IOB_SOC)/src/iob_generic_interconnect.v $(TIMER_DIR)/iob_timer.v

all: firmware.hex output_files/top_system.sof
	scp -P $(REMOTE_PORT) $< $(USER)@$(REMOTE_HOST):$(REMOTE_FPGA_DIR)
	scp -P $(REMOTE_PORT) $(FIRM_DIR)/firmware.bin $(USER)@$(REMOTE_HOST):$(REMOTE_FPGA_DIR)

output_files/top_system.sof: rmac top_system.qsf top_system.sdc $(VSRC) boot.hex
	./build.sh

rmac:
	sed -i "/ETH_RMAC_ADDR/d" $(ETH_DIR)/c-driver/iob-eth.h $(ETH_DIR)/rtl/include/iob_eth_defs.vh
	ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep -A 1 ^"en" | grep "ether" | awk '{print $$2}' | sed 's/://g' | sed "s/^/#define ETH_RMAC_ADDR 0x/" >> $(ETH_DIR)/c-driver/iob-eth.h
	ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep -A 1 ^"en" | grep "ether" | awk '{print $$2}' | sed 's/://g' | sed "s/^/\`define ETH_RMAC_ADDR 48'h/" >> $(ETH_DIR)/rtl/include/iob_eth_defs.vh

firmware.hex: $(FIRM_DIR)/firmware.hex
	cp $< .
	cp $(FIRM_DIR)/progmem.hex .
	$(PYTHON_DIR)/hex_split.py firmware

$(FIRM_DIR)/firmware.hex: FORCE
	make -C $(FIRM_DIR) BAUD=$(BAUD) FREQ=$(FREQ)

boot.hex: $(BOOT_DIR)/boot.hex
	cp $(BOOT_DIR)/boot.hex .
	$(PYTHON_DIR)/hex_split.py boot
	cp boot.hex boot.dat

$(BOOT_DIR)/boot.hex:
	make -C $(BOOT_DIR) BAUD=$(BAUD) FREQ=$(FREQ)

ld-hw:
	ssh -p $(REMOTE_PORT) $(USER)@$(REMOTE_HOST) $(REMOTE_FPGA_DIR)/prog.sh $(REMOTE_FPGA_DIR)

clean:
	@rm -rf db incremental_db output_files
	@rm -f *.summary *.rpt *.smsg *.txt *.done *.jdi *.pin *.sof *.sld *.qpf *~
	@rm -f *.dat *.hex *.bin
	make -C $(FIRM_DIR) clean --no-print-directory
	make -C $(BOOT_DIR) clean --no-print-directory
	make -C $(LD_SW_DIR) clean --no-print-directory
	sed -i "/ETH_RMAC_ADDR/d" $(ETH_DIR)/c-driver/iob-eth.h $(ETH_DIR)/rtl/include/iob_eth_defs.vh

.PHONY: all clean ld-hw ld-sw FORCE

.PRECIOUS: output_files/top_system.sof
