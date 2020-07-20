ROOT_DIR:=.
include ./system.mk

run: sim

sim: setsim firmware bootloader
ifeq ($(SIMULATOR),ncsim)
	ssh $(MICRO_USER)@$(SIM_SERVER) "if [ ! -d $(MICRO_ROOT_DIR) ]; then mkdir -p $(MICRO_ROOT_DIR); fi"
	rsync -avz --exclude .git . $(MICRO_USER)@$(SIM_SERVER):$(MICRO_ROOT_DIR) 
	ssh $(MICRO_USER)@$(SIM_SERVER) "cd $(MICRO_ROOT_DIR); make -C $(SIM_DIR)"
else
	make -C $(SIM_DIR)
endif

fpga: firmware bootloader
	ssh $(USER)@$(FPGA_COMPILE_SERVER) "if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi"
	rsync -avz --exclude .git . $(USER)@$(FPGA_COMPILE_SERVER):$(REMOTE_ROOT_DIR) 
	ssh $(USER)@$(FPGA_COMPILE_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) compile"


fpga-load: fpga
ifeq ($(FPGA_BOARD_SERVER),$(FPGA_COMPILE_SERVER))
	ssh $(USER)@$(FPGA_BOARD_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) load"
else
	ssh $(USER)@$(FPGA_BOARD_SERVER) "if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi"
	ssh $(USER)@$(FPGA_COMPILE_SERVER) "cd $(REMOTE_ROOT_DIR); rsync -avz --exclude .git . $(USER)@$(FPGA_BOARD_SERVER):$(REMOTE_ROOT_DIR)"
	ssh $(USER)@$(FPGA_BOARD_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) load"
endif

fpga-clean: clean
	ssh $(USER)@$(FPGA_BOARD_SERVER) "if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean; fi"
	ssh $(USER)@$(FPGA_COMPILE_SERVER) "if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean; fi"

fpga-clean-ip: fpga-clean
	ssh $(USER)@$(FPGA_COMPILE_SERVER) "if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean-ip; fi"


run-firmware: firmware
	ssh $(USER)@$(FPGA_BOARD_SERVER) "if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi"
	rsync -avz --exclude .git . $(USER)@$(FPGA_BOARD_SERVER):$(REMOTE_ROOT_DIR) 
	ssh $(USER)@$(FPGA_BOARD_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(CONSOLE_DIR) run"

firmware:
	make -C $(FIRM_DIR) BAUD=$(BAUD) SIM=$(SIM)

bootloader: firmware
	make -C $(BOOT_DIR) BAUD=$(BAUD)

pcsim:
	make -C $(FIRM_DIR) pcsim BAUD=$(BAUD) PCSIM=1

setsim:
	$(eval SIM=1)


ld-eth:
	ssh $(USER)@$(FPGA_BOARD_SERVER) "cd $(REMOTE_ROOT_DIR); make eth-comm"

eth-comm:
	$(eval RMAC := $(shell ethtool -P $(RMAC_INTERFACE) | awk '{print $$3}' | sed 's/://g'))
ifneq ($(SIM),1)
	@source /opt/pyeth/bin/activate; python $(FIRM_DIR)/eth_comm.py $(RMAC_INTERFACE) $(RMAC) $(FIRM_DIR); deactivate;
else
	python $(FIRM_DIR)/eth_comm.py $(RMAC_INTERFACE) $(RMAC) $(FIRM_DIR) PCsim
endif
ifneq (,$(wildcard $(FIRM_DIR)/write_image.py))
	@echo "Creating image file with detections...\n"
	@python $(FIRM_DIR)/write_image.py $(FIRM_DIR)/detections.bin
	@echo "Opening image file...\n"
	@display $(FIRM_DIR)/detections.png
endif

clean: 
ifeq ($(SIMULATOR),ncsim)
	ssh $(MICRO_USER)@$(SIM_SERVER) "cd $(MICRO_ROOT_DIR); make -C $(SIM_DIR) clean"
else
	make -C $(SIM_DIR) clean
endif
	make -C $(FIRM_DIR) clean
	make -C $(BOOT_DIR) clean


.PHONY: sim fpga fpga-clean fpga-clean-ip fpga-load firmware bootloader clean run-firmware
