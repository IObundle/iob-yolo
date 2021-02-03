#FIRMWARE
FIRM_ADDR_W:=16

#SRAM
SRAM_ADDR_W:=16

#DDR
ifeq ($(USE_DDR),)
	USE_DDR:=1
endif
ifeq ($(RUN_DDR),)
	RUN_DDR:=0
endif

DDR_ADDR_W:=30

#ROM
BOOTROM_ADDR_W:=12

#Init memory (only works in simulation or FPGA not running DDR)
ifeq ($(INIT_MEM),)
	INIT_MEM:=0
endif

#Choose Firmware (in SW_DIR)
TEST:=tiny_yolov3

#Ethernet
RMAC_ADDR:=00e04c690ba0 #Baba
#RMAC_ADDR:=309c231e624b #Pudim

#Versat
USE_NEW_VERSAT:=1

#VCD
VCD:=0

#Peripheral list (must match respective submodule name)
PERIPHERALS:=TIMER ETHERNET VERSAT #UART is implicit

SIM_LIST="SIMULATOR=icarus" "SIMULATOR=ncsim"
# SIMULATOR:=icarus
#SIMULATOR:=modelsim
SIMULATOR:=ncsim
LOCAL_SIM_LIST=icarus #leave space in the end

ifeq ($(SIMULATOR),ncsim)
	SIM_SERVER=$(MICRO)
	SIM_USER=$(MICRO_USER)
endif

BOARD_LIST="BOARD=CYCLONEV-GT-DK" "BOARD=AES-KU040-DB-G"
#FPGA board (associated with server below)
ifeq ($(BOARD),)
	BOARD:=AES-KU040-DB-G
	#FPGA_BOARD:=CYCLONEV-GT-DK
endif

ifeq ($(BOARD),AES-KU040-DB-G)
	COMPILE_USER=$(USER)
	COMPILE_SERVER=$(COMPILE_USER)@$(PUDIM)
	COMPILE_OBJ=synth_system.bit
	BOARD_USER=$(USER)
	BOARD_SERVER=$(BOARD_USER)@$(BABA)
else
#default
	BOARD=CYCLONEV-GT-DK
	COMPILE_USER=$(USER)
	COMPILE_SERVER=$(COMPILE_USER)@$(PUDIM)
	COMPILE_OBJ=output_files/top_system.sof
	BOARD_USER=$(USER)
	BOARD_SERVER=$(BOARD_USER)@$(PUDIM)
endif


#############################################################
#DO NOT EDIT BEYOND THIS POINT
#############################################################

#object directories
HW_DIR:=$(ROOT_DIR)/hardware
SIM_DIR:=$(HW_DIR)/simulation/$(SIMULATOR)
FPGA_DIR:=$(HW_DIR)/fpga/$(BOARD)

SW_DIR:=$(ROOT_DIR)/software
#submodule paths
SUBMODULES_DIR=$(ROOT_DIR)/submodules

SOC_DIR=$(SUBMODULES_DIR)/iob-soc
SOC_SW_DIR:=$(SOC_DIR)/software

FIRM_DIR:=$(SW_DIR)/$(TEST)

BOOT_DIR:=$(SW_DIR)/bootloader
BOOT_SRC_DIR:=$(SOC_SW_DIR)/bootloader

CONSOLE_DIR:=$(SW_DIR)/console
CONSOLE_SRC_DIR:=$(SOC_SW_DIR)/console

PYTHON_DIR:=$(SOC_SW_DIR)/python

#IOb-SoC - special submodule
SOC_SUBMODULES_DIR=$(SOC_DIR)/submodules

# iob-soc submodules
CPU_DIR:=$(SOC_SUBMODULES_DIR)/iob-picorv32
CACHE_DIR:=$(SOC_SUBMODULES_DIR)/iob-cache
INTERCON_DIR:=$(CACHE_DIR)/submodules/iob-interconnect
MEM_DIR:=$(CACHE_DIR)/submodules/iob-mem
AXI_MEM_DIR:=$(CACHE_DIR)/submodules/axi-mem

#MIG Bus - NOTE: need to manually change fpga .tcl
MIG_BUS_W:=256

#defines
DEFINE+=$(defmacro)BOOTROM_ADDR_W=$(BOOTROM_ADDR_W)
DEFINE+=$(defmacro)SRAM_ADDR_W=$(SRAM_ADDR_W)
DEFINE+=$(defmacro)FIRM_ADDR_W=$(FIRM_ADDR_W)
ifeq ($(USE_DDR),1)
DEFINE+=$(defmacro)USE_DDR
DEFINE+=$(defmacro)DDR_ADDR_W=$(DDR_ADDR_W)
DEFINE+=$(defmacro)MIG_BUS_W=$(MIG_BUS_W)
ifeq ($(RUN_DDR),1)
DEFINE+=$(defmacro)RUN_DDR
endif
endif
ifeq ($(INIT_MEM),1)
DEFINE+=$(defmacro)INIT_MEM 
endif
ifeq ($(USE_NEW_VERSAT),1)
DEFINE+=$(defmacro)USE_NEW_VERSAT
endif
DEFINE+=$(defmacro)N_SLAVES=$(N_SLAVES) 

#address selection bits
E:=31 #extra memory bit
ifeq ($(USE_DDR),1)
P:=30 #periphs
B:=29 #boot controller
else
P:=31
B:=30
endif

DEFINE+=$(defmacro)E=$E
DEFINE+=$(defmacro)P=$P
DEFINE+=$(defmacro)B=$B

SIM_BAUD:=10000000
HW_BAUD:=115200


ifeq ($(MAKECMDGOALS),)
BAUD:=$(SIM_BAUD)
else ifeq ($(MAKECMDGOALS),sim)
BAUD:=$(SIM_BAUD)
else
BAUD:=$(HW_BAUD)
endif

DEFINE+=$(defmacro)BAUD=$(BAUD)
ifeq ($(FREQ),)
DEFINE+=$(defmacro)FREQ=100000000
else
DEFINE+=$(defmacro)FREQ=$(FREQ)
endif



#run target by default
all: run

#create periph indices and directories

#UART - IOb-SoC submodule - slave 0
UART_DIR:=$(SOC_SUBMODULES_DIR)/UART
UART:=0
DEFINE+=$(defmacro)UART=$(UART)

# IOb-SoC-yolo submodules
N_SLAVES:=1
dummy:=$(foreach p, $(PERIPHERALS), $(eval $p_DIR:=$(SUBMODULES_DIR)/$p))
dummy:=$(foreach p, $(PERIPHERALS), $(eval $p=$(N_SLAVES)) $(eval N_SLAVES:=$(shell expr $(N_SLAVES) \+ 1)))
dummy:=$(foreach p, $(PERIPHERALS), $(eval DEFINE+=$(defmacro)$p=$($p)))

#server list
PUDIM:=pudim-flan.iobundle.com
BABA:=baba-de-camelo.iobundle.com
MICRO:=micro7.lx.it.pt

#user list
MICRO_USER=user14
SIM_ROOT_DIR=./$(USER)/sandbox/iob-soc-yolo

REMOTE_ROOT_DIR=./sandbox/iob-soc-yolo

#
# CORE DEFINITIONS
#

CORE_NAME:=YOLO
IS_CORE:=0


# PATHS
YOLO_HW_DIR:=$(YOLO_DIR)/hardware
YOLO_SUBMODULES_DIR:=$(YOLO_DIR)/submodules
TEX_DIR ?= $(YOLO_SUBMODULES_DIR)/TEX

# DOCUMENT
#DOC_TYPE:=pb
DOC_TYPE:=ug
INTEL ?=0
XILINX ?=1

VLINE:="V$(VERSION)"
$(CORE_NAME)_version.txt:
ifeq ($(VERSION),)
	$(error "veriable VERSION is not set")
endif
	echo $(VLINE) > $@

.PHONY: all
