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
	python $(VERSAT_PYTHON_DIR)/mkhdr.py . $(HW_DIR)/include/versat $(VERSAT_INC_DIR) .

clean:
	@rm -rf firmware.bin firmware.elf firmware.map *.hex ../periphs.h *~ pcsim *.h *.vh

.PHONY: run clean
