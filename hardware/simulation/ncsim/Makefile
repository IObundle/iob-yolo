ROOT_DIR:=../../..

defmacro:=-define 
incdir:=-incdir 

include ../simulation.mk

#simulator flags
CFLAGS = -errormax 15 -status -update -linedebug -sv $(INCLUDE) $(DEFINE)
EFLAGS = -errormax 15 -access +wc -status
SFLAGS = -errormax 15 -status -CMDFILE $(ROOT_DIR)/hardware/testbench

export VSRC CFLAGS EFLAGS SFLAGS 

#simulate
run: $(VSRC) $(VHDR) firmware boot.hex periphs
	./run_ncsim.sh $(TEST)

clean: hw-clean
	@rm -rf xcelium.d xmsim.key *.log

.PHONY: clean
