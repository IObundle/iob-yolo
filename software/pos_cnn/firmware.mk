TEST_DIR:=$(SW_DIR)/$(TEST)

#sources
SRC+=$(TEST_DIR)/*.c

#run target
run: versat firmware.elf

test-clean:

.PHONY: test-clean
