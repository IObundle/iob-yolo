ROOT_DIR:=../..
include ../software.mk

#include peripherals
dummy:=$(foreach p, $(PERIPHERALS), $(eval include $(SUBMODULES_DIR)/$p/software/embedded/embedded.mk))
#PERIPH_INC:=(for p in $(PERIPHERALS); do (echo $$p; include $(SUBMODULES_DIR)/$$p/software/embedded/embedded.mk); done)

#local sources
HDR+=$(SW_DIR)/periphs.h
INCLUDE+=-I$(SW_DIR)/$(TEST)

ifeq ($(PCSIM),1)
SRC+= firmware.c
else
SRC+= firmware.S firmware.c
#local flags
CFLAGS+=--specs=nano.specs -Wl,-Bstatic,-T,../template.lds,-Map,firmware.map,--strip-debug
endif

run: versat firmware.elf

firmware.elf: ../template.lds $(HDR) $(SRC)
	$(TOOLCHAIN_PREFIX)g++ -o $@ $(CFLAGS) $(DEFINE) $(INCLUDE) $(SRC) -lm -lgcc -lc -lnosys
	$(TOOLCHAIN_PREFIX)objcopy -O binary firmware.elf firmware.bin

pcsim: $(HDR) $(SRC)
	gcc $(CFLAGS) $(DEFINE) -DPCSIM $(INCLUDE) $(SRC) -o pcsim
	./pcsim

versat:
	python3 $(VERSAT_PYTHON_DIR)/mkvhdr.py . .
	python3 $(VERSAT_PYTHON_DIR)/mkhdr.py . $(HW_DIR)/include/new_versat $(HW_DIR)/include/dma .

clean:
	@rm -rf firmware.bin firmware.elf firmware.map *.hex ../periphs.h *~ pcsim *.vh versat.h

.PHONY: run clean
