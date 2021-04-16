defmacro:=-D
incdir:=-I
include $(ROOT_DIR)/system.mk

#compiler settings
ifeq ($(PCSIM),1)
CFLAGS:=-Os --std=gnu99 -g
else
TOOLCHAIN_PREFIX:=riscv64-unknown-elf-
CFLAGS=-Os -nostdlib -march=$(MFLAGS) -mabi=ilp32
endif

ifeq ($(USE_COMPRESSED),1)
MFLAGS=rv32imc
else
MFLAGS=rv32im
endif

#defines
DDR_ADDR_W=$(FPGA_DDR_ADDR_W)

ifeq ($(PCSIM),1)
DEFINE+=$(defmacro)PCSIM
else
ifeq ($(SIM),1)
DEFINE+=$(defmacro)SIM
DDR_ADDR_W=24
endif
endif

#ddr controller address width
DEFINE+=$(defmacro)DDR_ADDR_W=$(DDR_ADDR_W)

#INCLUDE
INCLUDE+=$(incdir)$(SW_DIR) $(incdir).

#headers
HDR=$(SW_DIR)/system.h

#common sources (none so far)
#SRC=$(SW_DIR)/*.c

.PHONY: periphs.h
