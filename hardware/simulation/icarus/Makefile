ROOT_DIR:=../../..

defmacro:=-D
incdir:=-I

include ../simulation.mk

#simulator flags
VLOG = iverilog -W all -g2005-sv $(INCLUDE) $(DEFINE) 

#run the simulator
run: $(VSRC) $(VHDR) firmware boot.hex periphs
	$(VLOG) $(VSRC)
	./a.out

clean: hw-clean
	@rm -f ./a.out

.PHONY: clean
