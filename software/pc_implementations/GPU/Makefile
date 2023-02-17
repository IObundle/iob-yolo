EXEC = embedded
CC = gcc
COMMON = -Isrc/
CFLAGS = -Wall -Wno-unused-result -Wno-unknown-pragmas -Wfatal-errors -fPIC -O3 -pg
LDFLAGS = -lm
OBJ = embedded_gpu.o embedded_gpu_kernels.o embedded.o
FILE = src/embedded.c

GPU=1
ARCH= -gencode arch=compute_52,code=compute_52
SRCDIR = ./src/
OBJDIR = ./obj/
NVCC = nvcc
SLIB=libdarknet.so
ALIB=libdarknet.a
AR=ar
ARFLAGS=rcs
VPATH=./src/

EXEC_CUDA=kernelTest

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
ifeq ($(GPU),1)
	DEFS += -D GPU
	LDFLAGS += -L/usr/local/cuda/lib64 -lcuda -lcudart -lcublas -lcurand -lstdc++
	COMMON += -I/usr/local/cuda/include
endif

OBJS = $(addprefix $(OBJDIR), $(OBJ))
DEPS = $(wildcard src/*.h) Makefile

all: obj $(OBJS) $(SLIB) $(ALIB) $(EXEC)

$(EXEC): $(ALIB)
#	@ $(CC) $(COMMON) $(DEFS) $(CFLAGS) -c $(FILE)
#	@ $(CC) $(OBJ) -o $@ -pg $(LDFLAGS)
	$(CC) $(COMMON) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(ALIB)

$(ALIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $^ 

$(SLIB): $(OBJS)
	$(CC) $(CFLAGS) -shared $^ -o $@ $(LDFLAGS)

$(OBJDIR)%.o: %.c $(DEPS)
	$(CC) $(COMMON) $(CFLAGS) $(DEFS) -c $< -o $@

$(OBJDIR)%.o: %.cu $(DEPS)
	$(NVCC) $(ARCH) $(COMMON) --compiler-options "$(CFLAGS) $(DEFS)" -c $< -o $@

obj:
	mkdir -p obj

cuda: ./src/kernelTest.cu
	$(NVCC) $(ARCH) -lcublas $< -o $(EXEC_CUDA)
	./$(EXEC_CUDA)

clean:
	@rm -rf $(EXEC) $(OBJS) $(SLIB) $(ALIB) $(OBJDIR)/* $(EXEC_CUDA)
	@rm -rf *.txt *.out
	@make -C bin/ clean
