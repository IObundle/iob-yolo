FW_DIR = ../$(TEST)
FW = firmware

CONSOLE_DIR = ../../submodules/iob-soc/software/ld-sw
ETH_DIR = ../../submodules/iob-eth

SERIAL := /dev/ttyUSB0
BAUD := 115200
FREQ := 100000000

CC := gcc
CFLAGS := -g -Wall -pedantic

TRGT = console
CSRCS = $(CONSOLE_DIR)/console.c
CHDRS = $(CONSOLE_DIR)/console.h

all: rmac run

rmac:
	sed -i "/ETH_RMAC_ADDR/d" $(ETH_DIR)/c-driver/iob-eth.h $(ETH_DIR)/rtl/include/iob_eth_defs.vh
	ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep -A 1 ^"en" | grep "ether" | awk '{print $$2}' | sed 's/://g' | sed "s/^/#define ETH_RMAC_ADDR 0x/" >> $(ETH_DIR)/c-driver/iob-eth.h
	ifconfig -a | grep -B 1 "ether" | sed '/inet/,+2d' | grep -A 1 ^"en" | grep "ether" | awk '{print $$2}' | sed 's/://g' | sed "s/^/\`define ETH_RMAC_ADDR 48'h/" >> $(ETH_DIR)/rtl/include/iob_eth_defs.vh

$(FW_DIR)/$(FW).bin: $(FW_DIR)/$(FW).c
	make -C $(FW_DIR) BAUD=$(BAUD) FREQ=$(FREQ) LOOPBACK=$(LOOPBACK) XILINX=$(XILINX)

$(FW).bin: $(FW_DIR)/$(FW).bin
	cp $^ .

$(TRGT): $(CSRCS) $(CHDRS)
	$(CC) $(CFLAGS) $(CSRCS) -o $@

run: $(TRGT) $(FW).bin
	./$(TRGT) -s $(SERIAL) -f $(FW).bin

clean:
	@rm -f *# *~
	@rm -f $(TRGT) *.bin
	make -C $(FW_DIR) clean --no-print-directory

.PHONY: all run clean
