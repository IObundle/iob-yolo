include $(ROOT_DIR)/hardware/hardware.mk

#testbench defines 
DEFINE+=$(define)SIM
ifeq ($(VCD),1)
DEFINE+=$(define)VCD
endif

#testbench source file
VSRC+=$(HW_DIR)/testbench/$(TEST)_tb.v $(AXI_MEM_DIR)/rtl/axi_ram.v

firmware.bin: $(FIRM_DIR)/firmware.bin
	cp $(FIRM_DIR)/firmware.bin .

clean: sim_clean
	@rm -f *# *~ *.vcd *.dat *.hex *.bin *.vh

.PHONY: clean
