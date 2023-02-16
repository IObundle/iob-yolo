ROOT_DIR:=.
include ./system.mk

WEIGHTS_FILE := yolov3-tiny.weights
TRANSFORMED_WEIGHTS_FILE = yolov3-tiny_batch_fixed.weights

CLASSES_FILE := classes.bin
TRANSFORMED_CLASSES_FILE := classes_transformed.bin

RGB_FILE := rgb.bin

IMAGE_FILE := image.png

RESOURCE_DIR := $(ROOT_DIR)/resources

$(RESOURCE_DIR)/$(TRANSFORMED_WEIGHTS_FILE): $(RESOURCE_DIR)/$(WEIGHTS_FILE)
	python3 $(SW_DIR)/python/transformWeights.py $(RESOURCE_DIR)/$(WEIGHTS_FILE) $(RESOURCE_DIR)/$(TRANSFORMED_WEIGHTS_FILE)

$(RESOURCE_DIR)/$(TRANSFORMED_CLASSES_FILE): $(RESOURCE_DIR)/$(CLASSES_FILE)
	python3 $(SW_DIR)/python/transformClass.py $(RESOURCE_DIR)/$(CLASSES_FILE) $(RESOURCE_DIR)/$(TRANSFORMED_CLASSES_FILE)

$(RESOURCE_DIR)/$(RGB_FILE):
	python3 $(SW_DIR)/python/outputRGB.py $(RESOURCE_DIR)/$(RGB_FILE)

data.bin: $(RESOURCE_DIR)/$(TRANSFORMED_WEIGHTS_FILE) $(RESOURCE_DIR)/$(TRANSFORMED_CLASSES_FILE) $(RESOURCE_DIR)/$(RGB_FILE)
	cp $(RESOURCE_DIR)/$(TRANSFORMED_WEIGHTS_FILE) data.bin
	cat $(RESOURCE_DIR)/$(RGB_FILE) >> data.bin
	cat $(RESOURCE_DIR)/$(TRANSFORMED_CLASSES_FILE) >> data.bin

py : data.bin

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

sim-waves: $(SIM_DIR)/system.vcd
	echo $(SIM_DIR)
	gtkwave $^ &

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
	@source /opt/pyeth/bin/activate; python $(FIRM_DIR)/eth_comm.py $(RMAC_INTERFACE) $(RMAC) $(FIRM_DIR) ./resources/data.bin; deactivate;
else
	python $(FIRM_DIR)/eth_comm.py $(RMAC_INTERFACE) $(RMAC) $(FIRM_DIR) PCsim
endif
ifneq (,$(wildcard $(FIRM_DIR)/write_image.py))
	@echo "Creating image file with detections...\n"
	@python $(FIRM_DIR)/write_image.py $(FIRM_DIR)/detections.bin
	@echo "Opening image file...\n"
	@display $(FIRM_DIR)/detections.png
endif

#
# DOCUMENT
#

doc:
	make -C document/$(DOC_TYPE) $(DOC_TYPE).pdf

doc-clean:
	make -C document/$(DOC_TYPE) clean

doc-pdfclean:
	make -C document/$(DOC_TYPE) pdfclean


clean: clean-sw sim-clean doc-clean
	make -C $(CONSOLE_DIR) clean


.PHONY: sim fpga fpga-clean fpga-clean-ip fpga-load firmware bootloader clean run-hw clean-sw ld-sw ld-hw sim-loc sim-clean
