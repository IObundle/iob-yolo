ROOT_DIR:=../..
include ../software.mk


#local sources
HDR+=$(SW_DIR)/periphs.h

ifeq ($(PCSIM),1)
SRC+= firmware.c
else
SRC+= firmware.S firmware.c
#local flags
CFLAGS+=--std=gnu99 -Wl,-Bstatic,-T,../template.lds,-Map,firmware.map,--strip-debug
endif

run: firmware.elf

firmware.elf: ../template.lds $(HDR) $(SRC)
	$(TOOLCHAIN_PREFIX)gcc -o $@ $(CFLAGS) $(DEFINE) $(INCLUDE) $(SRC) -lgcc -lc
	$(TOOLCHAIN_PREFIX)objcopy -O binary firmware.elf firmware.bin

pcsim: $(HDR) $(SRC)
	gcc $(CFLAGS) $(DEFINE) -DPCSIM $(INCLUDE) $(SRC) -o pcsim
	./pcsim

clean:
	@rm -rf firmware.bin firmware.elf firmware.map *.hex ../periphs.h *~ pcsim

.PHONY: run clean