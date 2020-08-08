# Yolo_SW parameters
INTERM_DATA = 0
FIXED = 1
GEMM = 0

ROOT_DIR:=../..
include ../software.mk
include $(CACHE_DIR)/software/software.mk

#local sources
HDR+=$(SW_DIR)/periphs.h

ifeq ($(PCSIM),1)
SRC+= firmware.c
else
SRC+= firmware.S firmware.c
#local flags
CFLAGS+=--std=gnu99 -Wl,-Bstatic,-T,../template.lds,-Map,firmware.map,--strip-debug
endif

ifeq ($(INTERM_DATA),1)
    DEFINE += -DINTERM_DATA
endif
ifeq ($(FIXED),1)
    DEFINE += -DFIXED
endif
ifeq ($(GEMM),1)
    DEFINE += -DGEMM
endif

run: edit_eth_comm firmware.elf

firmware.elf: ../template.lds $(HDR) $(SRC)
	$(TOOLCHAIN_PREFIX)gcc -o $@ $(CFLAGS) $(DEFINE) $(INCLUDE) $(SRC) -lgcc -lc
	$(TOOLCHAIN_PREFIX)objcopy -O binary firmware.elf firmware.bin

pcsim: edit_eth_comm $(HDR) $(SRC)
	gcc $(CFLAGS) $(DEFINE) -DPCSIM $(INCLUDE) $(SRC) -o pcsim
	./pcsim

edit_eth_comm:
	@sed -i "/interm_data_flag =/d" eth_comm.py
	@sed -i "/fixed_flag =/d" eth_comm.py
ifeq ($(INTERM_DATA),1)
	@sed '1 i interm_data_flag = 1' -i eth_comm.py
else
	@sed '1 i interm_data_flag = 0' -i eth_comm.py
endif
ifeq ($(FIXED),1)
	@sed '1 i fixed_flag = 1' -i eth_comm.py
else
	@sed '1 i fixed_flag = 0' -i eth_comm.py
endif


clean:
	@rm -rf firmware.bin firmware.elf firmware.map *.hex *.pyc *.bin ../periphs.h *~ pcsim tmpLocalSocket
	@sed -i "/interm_data_flag =/d" eth_comm.py
	@sed -i "/fixed_flag =/d" eth_comm.py


.PHONY: run clean
