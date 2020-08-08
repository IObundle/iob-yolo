ROOT_DIR:=../..
include ../software.mk


#local sources
HDR+=$(SW_DIR)/periphs.h
INCLUDE+=-I$(SW_DIR)/$(TEST)

ifeq ($(PCSIM),1)
SRC+= firmware.c
else
SRC+= firmware.S firmware.c
#local flags
CFLAGS+=-Wl,-Bstatic,-T,../template.lds,-Map,firmware.map,--strip-debug
endif

run: versat firmware.elf

firmware.elf: ../template.lds $(HDR) $(SRC)
	$(TOOLCHAIN_PREFIX)g++ -o $@ $(CFLAGS) $(DEFINE) $(INCLUDE) $(SRC) -lm -lgcc -lc
	$(TOOLCHAIN_PREFIX)objcopy -O binary firmware.elf firmware.bin

pcsim: $(HDR) $(SRC)
	gcc $(CFLAGS) $(DEFINE) -DPCSIM $(INCLUDE) $(SRC) -o pcsim
	./pcsim

versat:
	python $(VERSAT_PYTHON_DIR)/mkvhdr.py . .
	python $(VERSAT_PYTHON_DIR)/mkhdr.py . $(HW_DIR)/include/new_versat .

clean:
	@rm -rf firmware.bin firmware.elf firmware.map *.hex ../periphs.h *~ pcsim *.vh versat.h

.PHONY: run clean


############ ORIGINAL MAKEFILE







# TOOLCHAIN_PREFIX = riscv32-unknown-elf-

# SW_DIR = ..
# VERSAT_LOCAL = ../../rtl/include/new_versat
# PYTHON_DIR = ../../submodules/iob-soc/software/python
# UART_DIR = ../../submodules/iob-soc/submodules/iob-uart/c-driver
# ETH_DIR = ../../submodules/iob-eth/c-driver
# TIMER_DIR = ../../submodules/iob-timer
# CACHE_DIR = ../../submodules/iob-soc/submodules/iob-cache/c-driver
# VERSAT_DIR = ../../submodules/iob-versat
# VERSAT_SW = $(VERSAT_DIR)/software/python
# VERSAT_INC = $(VERSAT_DIR)/hardware/include
# IOB_SOC_FIRM_DIR = ../../submodules/iob-soc/software/firmware

# INCLUDE = -I. -I$(UART_DIR) -I$(ETH_DIR) -I$(TIMER_DIR) -I$(CACHE_DIR) -I$(SW_DIR)

# DEFINE = -DUART_BAUD_RATE=$(BAUD) -DUART_CLK_FREQ=$(FREQ)
# ifeq ($(LOOPBACK),1)
#     DEFINE += -DLOOPBACK
# endif
# ifeq ($(XILINX),1)
#     DEFINE += -DXILINX
# else
#     DEFINE += -DALTERA
# endif
# ifeq ($(SIM),1)
#     DEFINE += -DSIM
# endif

# SRC = firmware.c $(UART_DIR)/iob-uart.c $(IOB_SOC_FIRM_DIR)/firmware.S $(ETH_DIR)/iob-eth.c $(TIMER_DIR)/iob_timer.c

# all: versat firmware.hex

# versat:
# 	python $(VERSAT_SW)/mkvhdr.py . .
# 	python $(VERSAT_SW)/mkhdr.py . $(VERSAT_LOCAL) .

# firmware.hex: $(IOB_SOC_FIRM_DIR)/firmware.lds $(SRC) system.h $(UART_DIR)/iob-uart.h $(ETH_DIR)/iob-eth.h $(TIMER_DIR)/iob_timer.h $(CACHE_DIR)/iob-cache.h $(SW_DIR)/new_versat.hpp versat.h
# 	$(TOOLCHAIN_PREFIX)g++ -O3 -nostdlib -march=rv32im -mabi=ilp32 -o firmware.elf $(DEFINE) $(INCLUDE) $(SRC) -Wl,-Bstatic,-T,$(IOB_SOC_FIRM_DIR)/firmware.lds,-Map,firmware.map,--strip-debug -lgcc -lc
# 	$(TOOLCHAIN_PREFIX)objcopy -O binary firmware.elf firmware.bin
# 	$(eval MEM_SIZE=`$(IOB_SOC_FIRM_DIR)/get_firmsize.sh`)
# 	$(PYTHON_DIR)/makehex.py firmware.bin $(MEM_SIZE) > progmem.hex
# 	$(eval MEM_SIZE=`$(PYTHON_DIR)/get_memsize.py MAINRAM_ADDR_W`)
# 	$(PYTHON_DIR)/makehex.py firmware.bin $(MEM_SIZE) > firmware.hex

# system.h: ../../rtl/include/system.vh
# 	sed s/\`/\#/g ../../rtl/include/system.vh > system.h

# clean:
# 	@rm -rf firmware.bin firmware.elf firmware.map *.hex *.dat *.vh versat.h
# 	@rm -rf uart_loader system.h
# 	@rm -rf ../uart_loader
# 	@rm -rf *.pyc

# .PHONY: all clean
