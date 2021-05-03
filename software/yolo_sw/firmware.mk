#headers
HDR+=$(TEST_DIR)/*.h

#sources
SRC+=$(TEST_DIR)/*.c

#rules
run: firmware.elf
