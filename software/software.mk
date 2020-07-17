define:=-D
incdir:=-I
include $(ROOT_DIR)/system.mk

#submodules
include $(INTERCON_DIR)/software/software.mk

SW_DIR:=$(ROOT_DIR)/software

#include
#INCLUDE+=$(incdir)$(SW_DIR)
INCLUDE+=-I$(SW_DIR)

#headers
HDR=$(SW_DIR)/system.h

#sources (none so far)
#SRC=$(SW_DIR)/*.c

# PCSIM: runs firmware on host machine
ifeq ($(PCSIM),1)
  CFLAGS:=-Os --std=gnu99 -g
  #include peripherals
  # UART special case
  include $(UART_DIR)/software/pc/pc.mk
  # Other peripherals 
  dummy:=$(foreach p, $(PERIPHERALS), $(eval include $(SUBMODULES_DIR)/$p/software/pc/pc.mk))
else #Compile for riscv
  #compiler settings
  TOOLCHAIN_PREFIX:=riscv64-unknown-elf-
  CFLAGS:=-Os -ffreestanding -nostdlib -march=rv32im -mabi=ilp32 --std=gnu99

  #include peripherals
  # UART special case
  include $(UART_DIR)/software/embedded/embedded.mk

  # Boot sources do not include peripheral sources
  BOOT_SRC:=$(SRC)
  BOOT_INCLUDE:=$(INCLUDE)

  # Other peripherals 
  dummy:=$(foreach p, $(PERIPHERALS), $(eval include $(SUBMODULES_DIR)/$p/software/embedded/embedded.mk))
endif

$(SW_DIR)/periphs.h:
	$(shell echo "#define UART_BASE (1<<P) |(UART<<(ADDR_W-2-N_SLAVES_W))" >> ../periphs.h)
	$(foreach p, $(PERIPHERALS), $(shell echo "#define $p_BASE (1<<P) |($p<<(ADDR_W-2-N_SLAVES_W))" >> ../periphs.h) )
