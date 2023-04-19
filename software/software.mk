defmacro:=-D
incdir:=-I
include $(ROOT_DIR)/system.mk

#submodules
include $(INTERCON_DIR)/software/software.mk
#include $(ROOT_DIR)/submodules/VERSAT/software/software.mk
#software directory
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
  CFLAGS:=-O3 -nostdlib -march=rv32im -mabi=ilp32

  #include peripherals
  # UART special case
  include $(UART_DIR)/software/embedded/embedded.mk

  # Boot sources do not include peripheral sources
  BOOT_SRC:=$(SRC)
  BOOT_INCLUDE:=$(INCLUDE)

  #Define SIM
ifeq ($(SIM),1)
DEFINE+=$(defmacro)SIM
endif

endif

$(SW_DIR)/periphs.h:
	$(shell echo "#define UART_BASE (1<<$P) |(UART<<($P-N_SLAVES_W))" >> ../periphs.h)
	$(foreach p, $(PERIPHERALS), $(shell echo "#define $p_BASE (1<<$P) |($p<<($P-N_SLAVES_W))" >> ../periphs.h) )
