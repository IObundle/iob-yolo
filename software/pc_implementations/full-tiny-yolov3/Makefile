EXEC = embedded
CC = gcc
COMMON = -Isrc/
CFLAGS = -Wall -Wno-unused-result -Wno-unknown-pragmas -Wfatal-errors -fPIC -O3 -pg -lm
EXECOBJ = embedded.o
FILE = src/embedded.c


ifeq ($(EIGHT_BITS),1)
	DEFS = -D EIGHT_BITS
#	rm ../yolov3-tiny_8-bit_conv.hex
#else
#	rm ../yolov3-tiny_float_conv.hex	
endif
ifeq ($(FIXED),1)
	DEFS = -D FIXED
endif
ifeq ($(INTERM_DATA),1)
	DEFS += -D INTERM_DATA
endif
ifeq ($(LINEAR_EXP),1)
	DEFS += -D LINEAR_EXP
endif
ifeq ($(GEMM),1)
	DEFS += -D GEMM
endif
ifeq ($(mAP),1)
	DEFS += -D mAP
endif
ifeq ($(MAX_MIN),1)
	DEFS += -D MAX_MIN
endif

$(EXEC):
	@ $(CC) -g $(COMMON) $(DEFS) $(CFLAGS) -c $(FILE)
	@ $(CC) -g $(EXECOBJ) -o $@ -pg -lm

clean-json:
	@rm -rf ./map/*.json
	
clean-hex-out:
	rm ../yolov3-tiny_*_conv.hex

clean:
	@rm -rf $(EXEC) $(EXECOBJ)
	@rm -rf *.out *.hex *.txt
