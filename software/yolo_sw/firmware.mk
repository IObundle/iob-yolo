TEST_DIR:=$(SW_DIR)/$(TEST)

#headers
HDR+=$(TEST_DIR)/*.h

#sources
SRC+=$(TEST_DIR)/*.c

#rules
run: firmware.elf
