TOOLCHAIN_PREFIX = riscv32-unknown-elf-

PYTHON_DIR = ../../submodules/iob-soc/software/python/
UART_DIR = ../../submodules/iob-soc/submodules/iob-uart/c-driver
ETH_DIR = ../../submodules/iob-eth/c-driver
TIMER_DIR = ../../submodules/iob-timer
CACHE_DIR = ../../submodules/iob-soc/submodules/iob-cache/c-driver
IOB_SOC_FIRM_DIR = ../../submodules/iob-soc/software/firmware

INCLUDE = -I. -I$(UART_DIR) -I$(ETH_DIR) -I$(TIMER_DIR) -I$(CACHE_DIR)

DEFINE = -DUART_BAUD_RATE=$(BAUD) -DUART_CLK_FREQ=$(FREQ)
ifeq ($(LOOPBACK),1)
    DEFINE += -DLOOPBACK
endif
ifeq ($(XILINX),1)
    DEFINE += -DXILINX
else
    DEFINE += -DALTERA
endif
ifeq ($(SIM),1)
    DEFINE += -DSIM
endif

SRC = firmware.c $(UART_DIR)/iob-uart.c firmware.S $(ETH_DIR)/iob-eth.c $(TIMER_DIR)/iob_timer.c

all: firmware.hex

firmware.hex: $(IOB_SOC_FIRM_DIR)/firmware.lds $(SRC) system.h $(UART_DIR)/iob-uart.h $(ETH_DIR)/iob-eth.h $(TIMER_DIR)/iob_timer.h firmware.h $(CACHE_DIR)/iob-cache.h
	$(TOOLCHAIN_PREFIX)gcc -O3 -ffreestanding  -nostdlib -o firmware.elf $(DEFINE) $(INCLUDE) $(SRC) --std=gnu99 -Wl,-Bstatic,-T,$(IOB_SOC_FIRM_DIR)/firmware.lds,-Map,firmware.map,--strip-debug -lgcc -lc
	$(TOOLCHAIN_PREFIX)objcopy -O binary firmware.elf firmware.bin
	$(eval MEM_SIZE=`$(IOB_SOC_FIRM_DIR)/get_firmsize.sh`)
	$(PYTHON_DIR)/makehex.py firmware.bin $(MEM_SIZE) > progmem.hex
	$(eval MEM_SIZE=`$(PYTHON_DIR)/get_memsize.py MAINRAM_ADDR_W`)
	$(PYTHON_DIR)/makehex.py firmware.bin $(MEM_SIZE) > firmware.hex

system.h: ../../rtl/include/system.vh
	sed s/\`/\#/g ../../rtl/include/system.vh > system.h

clean:
	@rm -rf firmware.bin firmware.elf firmware.map *.hex *.dat
	@rm -rf uart_loader system.h
	@rm -rf ../uart_loader
	@rm -rf detections.*
	@rm -rf *.pyc

.PHONY: all clean
