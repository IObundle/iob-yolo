EXEC = embedded
CC = gcc
COMMON = -Isrc/
CFLAGS = -Wall -Wno-unused-result -Wno-unknown-pragmas -Wfatal-errors -fPIC -O3 -pg
EXECOBJ = embedded.o
FILE = src/embedded.c

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
	@ $(CC) $(COMMON) $(DEFS) $(CFLAGS) -c $(FILE)
	@ $(CC) $(EXECOBJ) -o $@ -pg

clean-json:
	@rm -rf ./map/*.json
	
clean:
	@rm -rf $(EXEC) $(EXECOBJ)
	@rm -rf *.txt *.out *.hex