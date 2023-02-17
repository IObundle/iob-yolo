#!/usr/bin/bash
export XILINXPATH=/opt/Xilinx
export LM_LICENSE_FILE=$XILINXPATH/Xilinx.lic
source /opt/Xilinx/Vivado/2020.2/settings64.sh
vivado -nojournal -log $@.log -mode batch -source ld-hw.tcl
