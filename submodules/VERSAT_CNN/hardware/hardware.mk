include $(VERSAT_CNN_DIR)/core.mk

# include
INCLUDE+=$(incdir) $(VERSAT_CNN_INC_DIR)
INCLUDE+=$(incdir) $(VERSAT_DIR)/hardware/include

#Versat yolo includes
ifeq ($(USE_NEW_VERSAT),1)
INCLUDE+=$(incdir)$(VERSAT_CNN_INC_DIR)/new_versat
endif
INCLUDE+=$(incdir)$(VERSAT_CNN_INC_DIR)/versat

# headers
VHDR+=$(wildcard $(VERSAT_CNN_INC_DIR)/*.vh)
VHDR+=$(wildcard $(VERSAT_DIR)/hardware/include/*.vh)

# sources
VSRC+=$(wildcard $(VERSAT_CNN_SRC_DIR)/*.v)

#Versat yolo
ifeq ($(USE_NEW_VERSAT),1)
VSRC+=$(wildcard $(VERSAT_CNN_SRC_DIR)/new_versat/*.v)
else
VSRC+=$(wildcard $(VERSAT_CNN_SRC_DIR)/versat/*.v)
endif

clean_hw:
	@rm -rf $(VERSAT_CNN_HW_DIR)/fpga/vivado/XCKU $(VERSAT_CNN_HW_DIR)/fpga/quartus/CYCLONEV-GT

.PHONY: clean_hw
