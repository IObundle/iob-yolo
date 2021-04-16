TEST_DIR:=$(SW_DIR)/$(TEST)

#sources
SRC+=$(TEST_DIR)/*.c

#run target
run: firmware.elf

test-clean:
	@rm -rf tmpLocalSocket

.PHONY: test-clean
