CORE_NAME:=VERSAT_CNN
IS_CORE:=1
USE_NETLIST ?=0
TOP_MODULE:=versat_cnn

#PATHS
VERSAT_CNN_HW_DIR:=$(VERSAT_CNN_DIR)/hardware
VERSAT_CNN_INC_DIR:=$(VERSAT_CNN_HW_DIR)/include
VERSAT_CNN_SRC_DIR:=$(VERSAT_CNN_HW_DIR)/src
VERSAT_CNN_SUBMODULES_DIR:=$(VERSAT_CNN_DIR)/submodules
VERSAT_DIR:=$(VERSAT_CNN_DIR)/submodules/versat
TEX_DIR:=$(VERSAT_CNN_DIR)/submodules/TEX
REMOTE_ROOT_DIR ?=sandbox/iob-versat-cnn

#SIMULATION
SIMULATOR ?=icarus
SIM_SERVER ?=localhost
SIM_USER ?=$(USER)
SIM_DIR ?=hardware/simulation/$(SIMULATOR)

#FPGA
FPGA_FAMILY ?=XCKU
FPGA_USER ?=$(USER)
FPGA_SERVER ?=pudim-flan.iobundle.com
ifeq ($(FPGA_FAMILY),XCKU)
        FPGA_COMP:=vivado
        FPGA_PART:=xcku040-fbva676-1-c
else #default; ifeq ($(FPGA_FAMILY),CYCLONEV-GT)
        FPGA_COMP:=quartus
        FPGA_PART:=5CGTFD9E5F35C7
endif
FPGA_DIR ?= $(VERSAT_CNN_DIR)/hardware/fpga/$(FPGA_COMP)
ifeq ($(FPGA_COMP),vivado)
FPGA_LOG:=vivado.log
else ifeq ($(FPGA_COMP),quartus)
FPGA_LOG:=quartus.log
endif

#ASIC
ASIC_NODE ?=umc130
ASIC_SERVER ?=micro5.lx.it.pt
ASIC_COMPILE_ROOT_DIR ?=$(ROOT_DIR)/sandbox/iob-versat-cnn
ASIC_USER ?=user14
ASIC_DIR ?=hardware/asic/$(ASIC_NODE)

XILINX ?=1
INTEL ?=1

VLINE:="V$(VERSION)"
$(CORE_NAME)_version.txt:
ifeq ($(VERSION),)
	$(error "variable VERSION is not set")
endif
	echo $(VLINE) > version.txt
