ROOT_DIR:=../..
include $(ROOT_DIR)/software/software.mk

ifeq ($(USE_DDR),1)
ifeq ($(RUN_DDR),1)
include $(CACHE_DIR)/software/software.mk
endif
endif

#specific compiler flags
CFLAGS+=--specs=nano.specs -Wl,-Bstatic,-T,../template.lds,-Map,boot.map,--strip-debug

ifeq ($(USE_DDR),1)
ifeq ($(RUN_DDR),1)
#specific sources
BOOT_SRC+=$(BOOT_SRC_DIR)/boot.S boot.c $(CACHE_SW_DIR)/iob-cache.c
#specific includes
BOOT_INCLUDE+=$(incdir)$(CACHE_SW_DIR) 
else
#specific sources
BOOT_SRC+=$(BOOT_SRC_DIR)/boot.S boot.c
endif
endif

run: boot.elf

boot.elf: ../template.lds  $(HDR) $(BOOT_SRC)
	$(TOOLCHAIN_PREFIX)gcc -o $@ $(CFLAGS) $(DEFINE) $(BOOT_INCLUDE) $(BOOT_SRC) -lgcc -lc -lnosys
	$(TOOLCHAIN_PREFIX)objcopy -O binary boot.elf boot.bin

clean:
	@rm -rf boot.bin boot.elf boot.map *.hex ../periphs.h *~

.PHONY: clean
