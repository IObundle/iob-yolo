TEST_DIR:=$(SW_DIR)/$(TEST)

#headers
HDR+=$(TEST_DIR)/*.h

#sources
SRC+=$(TEST_DIR)/*.c

#run target
run: versat firmware.elf

test-clean:

.PHONY: test-clean