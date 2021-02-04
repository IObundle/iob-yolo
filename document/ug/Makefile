YOLO_DIR:=../..
ROOT_DIR:=$(YOLO_DIR)
include $(YOLO_DIR)/system.mk

IS_TAB:=gen_is_tab.tex axi4_m_is_tab.tex uart_is_tab.tex #cpu_nat_s_is_tab.tex cpu_axi4lite_s_is_tab.tex cpu_axi4_m_is_tab.tex

REG_TAB:=sw_yoloreg_tab.tex

BD_TAB:=bd_tab.tex

PARAM_TAB:=sp_tab.tex

SRC:=ug.tex $(wildcard ./*.tex) $(wildcard ../*.tex) $(BD_TAB) $(PARAM_TAB) $(IS_TAB) $(REG_TAB)

#TD_FIGS:=datadelay_3.pdf fsync_w_2.pdf i2s.pdf inverted_bclk.pdf inverted_fsync.pdf n_channel_3.pdf sample_w_16.pdf rx_master.pdf

include $(YOLO_DIR)/submodules/TEX/document/document.mk

gen_is_tab.tex: $(YOLO_HW_DIR)/include/gen_if.v
	$(TEX_SW_DIR)/io2tex.py $< $@

#cpu_nat_s_is_tab.tex: $(INTERCON_DIR)/hardware/include/cpu_nat_s_if.v
#	$(TEX_SW_DIR)/io2tex.py $< $@

#cpu_axi4lite_s_is_tab.tex: $(INTERCON_DIR)/hardware/include/cpu_axi4lite_s_if.v
#	$(TEX_SW_DIR)/io2tex.py $< $@

axi4_m_is_tab.tex: $(YOLO_HW_DIR)/include/axi4_m_if.v
	$(TEX_SW_DIR)/io2tex.py $< $@

uart_is_tab.tex: $(YOLO_HW_DIR)/include/uart_if.v
	$(TEX_SW_DIR)/io2tex.py $< $@

sw_yoloreg_tab.tex: $(YOLO_HW_DIR)/include/sw_reg.v
	$(TEX_SW_DIR)/swreg2tex.py $<

bd_tab.tex: $(YOLO_HW_DIR)/src/system_core.v
	$(TEX_SW_DIR)/block2tex.py $@ $(YOLO_HW_DIR)/src/system_core.v

sp_tab.tex: $(YOLO_HW_DIR)/src/system_core.v $(YOLO_HW_DIR)/include/export.vh
	$(TEX_SW_DIR)/param2tex.py $(YOLO_HW_DIR)/src/system_core.v $@ $(YOLO_HW_DIR)/include/export.vh

clean: texclean
	@rm -f $(IS_TAB) $(REG_TAB) $(BD_TAB) $(PARAM_TAB)

.PHONY: clean
