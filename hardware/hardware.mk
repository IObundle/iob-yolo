include $(ROOT_DIR)/system.mk

# submodules
include $(CPU_DIR)/hardware/hardware.mk
ifeq ($(USE_DDR),1)
include $(CACHE_DIR)/hardware/hardware.mk
else
include $(INTERCON_DIR)/hardware/hardware.mk
endif

# include
INC_DIR:=$(ROOT_DIR)/hardware/include
SOC_INC_DIR:=$(SOC_DIR)/hardware/include

INCLUDE+=$(incdir). $(incdir)$(INC_DIR) $(incdir)$(SOC_INC_DIR)

#headers
VHDR+=$(SOC_INC_DIR)/system.vh
#TODO: add versat headers path here later

# sources
SRC_DIR:=$(ROOT_DIR)/hardware/src
SOC_SRC_DIR:=$(SOC_DIR)/hardware/src

#rom
VSRC+=$(SOC_SRC_DIR)/boot_ctr.v \
$(MEM_DIR)/sp_rom/sp_rom.v 

#ram
VSRC+=$(SOC_SRC_DIR)/int_mem.v \
$(SOC_SRC_DIR)/sram.v \
$(MEM_DIR)/tdp_ram/iob_tdp_ram.v

#ddr
ifeq ($(USE_DDR),1)
VSRC+=$(SOC_SRC_DIR)/ext_mem.v
endif

#system
VSRC+=$(SRC_DIR)/system.v

# peripherals
periphs:
	$(eval include $(SOC_SUBMODULES_DIR)/UART/hardware/hardware.mk)
	$(foreach p, $(PERIPHERALS), $(eval include $(SUBMODULES_DIR)/$p/hardware/hardware.mk))

# data files
firmware.hex: $(FIRM_DIR)/firmware.hex
	cp $(FIRM_DIR)/firmware.bin .
ifeq ($(INIT_MEM),1)
	cp $(FIRM_DIR)/firmware.hex .
	$(PYTHON_DIR)/hex_split.py firmware
endif

boot.hex: $(BOOT_DIR)/boot.hex
	cp $(BOOT_DIR)/boot.hex .

.PHONY: periphs
