# Yolo_SW parameters
INTERM_DATA = 0
FIXED = 1
GEMM = 0

#defines
ifeq ($(INTERM_DATA),1)
DEFINE+=$(defmacro)INTERM_DATA
endif

ifeq ($(FIXED),1)
DEFINE+=$(defmacro)FIXED
endif

ifeq ($(GEMM),1)
DEFINE+=$(defmacro)GEMM
endif

#headers
HDR+=$(TEST_DIR)/*.h

#sources
SRC+=$(TEST_DIR)/*.c

#rules
run: edit_eth_comm firmware.elf

edit_eth_comm:
	@sed -i "/interm_data_flag =/d" eth_comm.py
	@sed -i "/fixed_flag =/d" eth_comm.py
ifeq ($(INTERM_DATA),1)
	@sed '1 i interm_data_flag = 1' -i eth_comm.py
else
	@sed '1 i interm_data_flag = 0' -i eth_comm.py
endif
ifeq ($(FIXED),1)
	@sed '1 i fixed_flag = 1' -i eth_comm.py
else
	@sed '1 i fixed_flag = 0' -i eth_comm.py
endif

test-clean:
	@sed -i "/interm_data_flag =/d" eth_comm.py
	@sed -i "/fixed_flag =/d" eth_comm.py

.PHONY: edit_eth_comm test-clean
