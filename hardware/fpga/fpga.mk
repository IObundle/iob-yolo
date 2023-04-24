include $(ROOT_DIR)/hardware/hardware.mk

#board specific top level source
VSRC+=./verilog/top_system.v

#console command
CONSOLE_CMD=$(PYTHON_DIR)/console -s /dev/usb-uart
#CONSOLE_CMD='make ld-sw'
ifeq ($(INIT_MEM),0)
CONSOLE_CMD+=-f
endif

GRAB_CMD=board_client.py grab 300

FPGA_PROG=./prog.sh

load:
	cp $(FIRM_DIR)/firmware.bin .
	$(GRAB_CMD) -p '$(FPGA_PROG)' -c '$(CONSOLE_CMD)'

compile: periphs firmware $(COMPILE_OBJ)

$(COMPILE_OBJ): $(wildcard *.sdc) $(VSRC) $(VHDR) boot.hex
	./build.sh "$(INCLUDE)" "$(DEFINE)" "$(VSRC)"

.PRECIOUS: $(COMPILE_OBJ)

.PHONY: load compile
