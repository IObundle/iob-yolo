ROOT_DIR:=../..
include ../software.mk
include $(TEST_DIR)/firmware.mk

#additional compiler flags
ifneq ($(PCSIM),1)
CFLAGS+=--specs=nano.specs -Wl,-Bstatic,-T,../template.lds,-Map,firmware.map,--strip-debug
endif


#SUBMODULES

#cache
ifeq ($(USE_DDR),1)
include $(CACHE_DIR)/software/software.mk
endif

#peripherals
ifeq ($(PCSIM),1)
$(foreach p, $(PERIPHERALS), $(eval include $(SUBMODULES_DIR)/$p/software/pc/pc.mk))
else
$(foreach p, $(PERIPHERALS), $(eval include $(SUBMODULES_DIR)/$p/software/embedded/embedded.mk))
endif

#HEADERS
HDR+=periphs.h

#SOURCES
SRC+= firmware.c $(UART_DIR)/software/printf.c

ifneq ($(PCSIM),1)
SRC+= firmware.S
endif

#RULES
firmware.elf: ../template.lds $(HDR) $(SRC)
	$(TOOLCHAIN_PREFIX)g++ -o $@ $(CFLAGS) $(DEFINE) $(INCLUDE) $(SRC) -lm -lgcc -lc -lnosys
	$(TOOLCHAIN_PREFIX)objcopy -O binary firmware.elf firmware.bin

pcsim: $(HDR) $(SRC)
	gcc $(CFLAGS) $(DEFINE) $(INCLUDE) $(SRC) -o $@
	./$@

versat:
	@python3 $(VERSAT_CNN_PYTHON_DIR)/mkvhdr.py ../$(TEST) .
	@python3 $(VERSAT_CNN_PYTHON_DIR)/mkhdr.py . $(VERSAT_CNN_INC_DIR)/new_versat $(VERSAT_CNN_INC_DIR)/dma .

#peripherals' base addresses
periphs.h: periphs_tmp.h
	@is_diff=`diff -q -N $@ $<`; if [ "$$is_diff" ]; then cp $< $@; fi
	@rm periphs_tmp.h

periphs_tmp.h:
	$(foreach p, $(PERIPHERALS), $(shell echo "#define $p_BASE (1<<$P) |($p<<($P-N_SLAVES_W))" >> $@) )

clean: gen-clean test-clean
	@rm -rf firmware.bin firmware.elf firmware.map *.hex periphs.h pcsim *.vh *.h

.PHONY: run clean versat
