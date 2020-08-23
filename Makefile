ROOT_DIR:=.
include ./system.mk

run: sim

sim-loc:
	make -C hardware/simulation/ncsim

ld-sw:
	make -C software/console run

ld-hw:
	make -C $(FPGA_DIR) load

sim: setsim firmware bootloader
ifeq ($(SIMULATOR),$(filter $(SIMULATOR), $(LOCAL_SIM_LIST)))
	make -C $(SIM_DIR)  INIT_MEM=$(INIT_MEM) USE_DDR=$(USE_DDR) RUN_DDR=$(RUN_DDR)
else
	ssh $(SIM_USER)@$(SIM_SERVER) "if [ ! -d $(SIM_ROOT_DIR) ]; then mkdir -p $(SIM_ROOT_DIR); fi"
	rsync -avz --exclude .git $(ROOT_DIR) $(SIM_USER)@$(SIM_SERVER):$(SIM_ROOT_DIR)
	#ssh $(SIM_USER)@$(SIM_SERVER) 'cd $(SIM_ROOT_DIR); make -C $(SIM_DIR) INIT_MEM=$(INIT_MEM) USE_DDR=$(USE_DDR) RUN_DDR=$(RUN_DDR)'
endif

sim-clean: clean
ifeq ($(SIMULATOR),$(filter $(SIMULATOR), $(LOCAL_SIM_LIST)))
	make -C $(SIM_DIR) clean SIMULATOR=$(SIMULATOR)
else
	rsync -avz --exclude .git $(ROOT_DIR) $(SIM_USER)@$(SIM_SERVER):$(SIM_ROOT_DIR)
	ssh $(SIM_USER)@$(SIM_SERVER) 'if [ -d $(SIM_ROOT_DIR) ]; then cd $(SIM_ROOT_DIR); make -C $(SIM_DIR) clean SIMULATOR=$(SIMULATOR); fi'
endif

fpga: firmware bootloader
ifeq ($(BOARD),$(filter $(BOARD), $(LOCAL_COMPILER_LIST)))
	make -C $(FPGA_DIR) compile INIT_MEM=$(INIT_MEM) USE_DDR=$(USE_DDR) RUN_DDR=$(RUN_DDR)
else
	ssh $(BOARD_SERVER) 'if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi'
	rsync -avz --exclude .git $(ROOT_DIR) $(COMPILE_SERVER):$(REMOTE_ROOT_DIR)
	ssh $(COMPILE_SERVER) 'cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) compile INIT_MEM=$(INIT_MEM) USE_DDR=$(USE_DDR) RUN_DDR=$(RUN_DDR)'
ifneq ($(COMPILE_SERVER),$(BOARD_SERVER))
	scp $(COMPILE_SERVER):$(REMOTE_ROOT_DIR)/$(FPGA_DIR)/$(COMPILE_OBJ) $(FPGA_DIR)
endif
endif

# ssh $(USER)@$(FPGA_COMPILE_SERVER) "if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi"
# rsync -avz --exclude .git . $(USER)@$(FPGA_COMPILE_SERVER):$(REMOTE_ROOT_DIR) 
# ssh $(USER)@$(FPGA_COMPILE_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) compile"



fpga-load: firmware bootloader
ifeq ($(BOARD),$(filter $(BOARD), $(LOCAL_BOARD_LIST)))
	make -C $(FPGA_DIR) load
else
	ssh $(BOARD_SERVER) 'if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi'
	rsync -avz --exclude .git $(ROOT_DIR) $(BOARD_SERVER):$(REMOTE_ROOT_DIR) 
	ssh $(BOARD_SERVER) 'cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) load'
endif




# ifeq ($(FPGA_BOARD_SERVER),$(FPGA_COMPILE_SERVER))
# 	ssh $(USER)@$(FPGA_BOARD_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) load"
# else
# 	ssh $(USER)@$(FPGA_BOARD_SERVER) "if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi"
# 	ssh $(USER)@$(FPGA_COMPILE_SERVER) "cd $(REMOTE_ROOT_DIR); rsync -avz --exclude .git . $(USER)@$(FPGA_BOARD_SERVER):$(REMOTE_ROOT_DIR)"
# 	ssh $(USER)@$(FPGA_BOARD_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) load"
# endif

fpga-clean: clean
ifeq ($(BOARD),$(filter $(BOARD), $(LOCAL_COMPILER_LIST)))
	make -C $(FPGA_DIR) clean BOARD=$(BOARD)
else
	rsync -avz --exclude .git $(ROOT_DIR) $(COMPILE_SERVER):$(REMOTE_ROOT_DIR)
	ssh $(COMPILE_SERVER) 'if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean BOARD=$(BOARD); fi'
endif
ifeq ($(BOARD),$(filter $(BOARD), $(LOCAL_BOARD_LIST)))
	make -C $(FPGA_DIR) clean BOARD=$(BOARD)
else
	rsync -avz --exclude .git $(ROOT_DIR) $(BOARD_SERVER):$(REMOTE_ROOT_DIR)
	ssh $(BOARD_SERVER) 'if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean BOARD=$(BOARD); fi'
endif

# ssh $(USER)@$(FPGA_BOARD_SERVER) "if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean; fi"
# ssh $(USER)@$(FPGA_COMPILE_SERVER) "if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean; fi"

fpga-clean-ip: fpga-clean
ifeq ($(BOARD), $(filter $(BOARD), $(LOCAL_COMPILER_LIST)))
	make -C $(FPGA_DIR) clean-ip
else
	ssh $(COMPILE_SERVER) 'cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean-ip'
endif

# ssh $(USER)@$(FPGA_COMPILE_SERVER) "if [ -d $(REMOTE_ROOT_DIR) ]; then cd $(REMOTE_ROOT_DIR); make -C $(FPGA_DIR) clean-ip; fi"


run-hw: firmware
ifeq ($(BOARD),$(filter $(BOARD), $(LOCAL_BOARD_LIST)))
	make -C $(CONSOLE_DIR) run INIT_MEM=$(INIT_MEM)
else
	ssh $(BOARD_SERVER) 'if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi'
	rsync -avz --exclude .git $(ROOT_DIR) $(BOARD_SERVER):$(REMOTE_ROOT_DIR) 
	#ssh $(BOARD_SERVER) 'cd $(REMOTE_ROOT_DIR); make -C $(CONSOLE_DIR) run INIT_MEM=$(INIT_MEM)'
endif

# ssh $(USER)@$(FPGA_BOARD_SERVER) "if [ ! -d $(REMOTE_ROOT_DIR) ]; then mkdir -p $(REMOTE_ROOT_DIR); fi"
# rsync -avz --exclude .git . $(USER)@$(FPGA_BOARD_SERVER):$(REMOTE_ROOT_DIR) 
# ssh $(USER)@$(FPGA_BOARD_SERVER) "cd $(REMOTE_ROOT_DIR); make -C $(CONSOLE_DIR) run"

firmware:
	make -C $(FIRM_DIR) BAUD=$(BAUD) SIM=$(SIM)

bootloader: firmware
	make -C $(BOOT_DIR) BAUD=$(BAUD)

pcsim:
	make -C $(FIRM_DIR) clean
	make -C $(FIRM_DIR) pcsim BAUD=$(BAUD) PCSIM=1

clean-sw:
	make -C $(FIRM_DIR) clean
	make -C $(BOOT_DIR) clean

setsim:
	$(eval SIM=1)

#Run on FPGA_BOARD_SERVER
# for PCSIM: make ld-eth SIM=1
ld-eth:
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

clean: clean-sw sim-clean
	make -C $(CONSOLE_DIR) clean


.PHONY: sim fpga fpga-clean fpga-clean-ip fpga-load firmware bootloader clean run-hw clean-sw ld-sw ld-hw sim-loc sim-clean
