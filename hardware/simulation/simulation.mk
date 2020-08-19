include $(ROOT_DIR)/hardware/hardware.mk

#testbench defines 
DEFINE+=$(defmacro)SIM
ifeq ($(VCD),1)
DEFINE+=$(defmacro)VCD
endif

#testbench source files
VSRC+=$(TB_DIR)/$(TEST)_tb.v $(AXI_MEM_DIR)/rtl/axi_ram.v
