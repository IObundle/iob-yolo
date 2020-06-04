#
# SYNTHESIS AND IMPLEMENTATION SCRIPT
#

#include
read_verilog ../../../rtl/include/versat/xconfdefs.vh
read_verilog ../../../rtl/include/versat/xdefs.vh
read_verilog ../../../rtl/include/versat/xmuladdlitedefs.vh
read_verilog ../../../rtl/include/system.vh
read_verilog ../../../submodules/iob-soc/submodules/iob-uart/rtl/include/iob-uart.vh
read_verilog ../../../submodules/iob-eth/rtl/include/iob_eth_defs.vh
read_verilog ../../../submodules/iob-versat/rtl/include/xaludefs.vh
read_verilog ../../../submodules/iob-versat/rtl/include/xalulitedefs.vh
read_verilog ../../../submodules/iob-versat/rtl/include/xbsdefs.vh
read_verilog ../../../submodules/iob-versat/rtl/include/xmemdefs.vh
read_verilog ../../../submodules/iob-versat/rtl/include/xmuladddefs.vh
read_verilog ../../../submodules/iob-versat/rtl/include/xmuldefs.vh
read_verilog ../../../submodules/iob-versat/submodules/versat-io/rtl/include/versat-io.vh
if { [file exists [lindex $argv $argc-1]] == 1} {
    #use xversat.vh of firmware folder
    read_verilog [lindex $argv $argc-1]
} else {
    read_verilog ../../../submodules/iob-versat/rtl/include/versat/xversat.vh
}

#clock
if { [lindex $argv 0] != {USE_DDR} } {
    read_verilog verilog/clock_wizard.v
}

#system
read_verilog verilog/top_system.v
read_verilog ../../../rtl/src/system.v
read_verilog ../../../submodules/iob-soc/rtl/src/iob_generic_interconnect.v

#picorv32
read_verilog ../../../submodules/iob-soc/submodules/iob-rv32/picorv32.v

#uart
read_verilog ../../../submodules/iob-soc/submodules/iob-uart/rtl/src/iob-uart.v

#memory
read_verilog ../../../submodules/iob-soc/rtl/src/memory/behav/rom.v
read_verilog ../../../submodules/iob-soc/rtl/src/memory/behav/ram.v
read_verilog ../../../submodules/iob-soc/rtl/src/int_mem.v
read_verilog ../../../submodules/iob-soc/rtl/src/memory/behav/iob_1p_mem.v

#ethernet
read_verilog ../../../submodules/iob-eth/rtl/src/iob_eth_alt_s2p_mem.v
read_verilog ../../../submodules/iob-eth/rtl/src/iob_eth_crc.v
read_verilog ../../../submodules/iob-eth/rtl/src/iob_eth_rx.v
read_verilog ../../../submodules/iob-eth/rtl/src/iob_eth_tx.v
read_verilog ../../../submodules/iob-eth/rtl/src/iob_eth.v

#timer
read_verilog ../../../submodules/iob-timer/iob_timer.v

#versat
read_verilog ../../../rtl/src/versat/xconf_reg.v
read_verilog ../../../rtl/src/versat/xdata_eng.v
read_verilog ../../../rtl/src/versat/xmuladdlite.v
read_verilog ../../../submodules/iob-versat/rtl/src/xaddrgen.v
read_verilog ../../../submodules/iob-versat/rtl/src/xaddrgen2.v
read_verilog ../../../submodules/iob-versat/rtl/src/xalu.v
read_verilog ../../../submodules/iob-versat/rtl/src/xalulite.v
read_verilog ../../../submodules/iob-versat/rtl/src/xbs.v
read_verilog ../../../submodules/iob-versat/rtl/src/xclz.v
read_verilog ../../../submodules/iob-versat/rtl/src/xconf.v
read_verilog ../../../submodules/iob-versat/rtl/src/xconf_mem.v
read_verilog ../../../submodules/iob-versat/rtl/src/xinmux.v
read_verilog ../../../submodules/iob-versat/rtl/src/xmem.v
read_verilog ../../../submodules/iob-versat/rtl/src/xmul.v
read_verilog ../../../submodules/iob-versat/rtl/src/xmul_pipe.v
read_verilog ../../../submodules/iob-versat/rtl/src/xmuladd.v
read_verilog ../../../submodules/iob-versat/rtl/src/xstage.v
read_verilog ../../../submodules/iob-versat/rtl/src/xversat.v
read_verilog ../../../submodules/iob-versat/submodules/mem/tdp_mem/iob_tdp_mem.v
set_property file_type SystemVerilog [get_files xaddrgen.v]
set_property file_type SystemVerilog [get_files xaddrgen2.v]
set_property file_type SystemVerilog [get_files xalu.v]
set_property file_type SystemVerilog [get_files xalulite.v]
set_property file_type SystemVerilog [get_files xbs.v]
set_property file_type SystemVerilog [get_files xclz.v]
set_property file_type SystemVerilog [get_files xconf.v]
set_property file_type SystemVerilog [get_files xconf_mem.v]
set_property file_type SystemVerilog [get_files xconf_reg.v]
set_property file_type SystemVerilog [get_files xdata_eng.v]
set_property file_type SystemVerilog [get_files xinmux.v]
set_property file_type SystemVerilog [get_files xmem.v]
set_property file_type SystemVerilog [get_files xmul.v]
set_property file_type SystemVerilog [get_files xmul_pipe.v]
set_property file_type SystemVerilog [get_files xmuladd.v]
set_property file_type SystemVerilog [get_files xmuladdlite.v]
set_property file_type SystemVerilog [get_files xstage.v]
set_property file_type SystemVerilog [get_files xversat.v]

set_property part xcku040-fbva676-1-c [current_project]

if { [lindex $argv 0] == {USE_DDR} } {
    
    read_verilog ../../../submodules/iob-soc/submodules/fifo/afifo.v
    read_verilog ../../../submodules/iob-soc/submodules/iob-cache/rtl/header/iob-cache.vh
    read_verilog ../../../submodules/iob-soc/submodules/iob-cache/rtl/src/iob-cache.v
    read_verilog ../../../submodules/iob-soc/submodules/iob-cache/rtl/src/gen_mem_reg.v

    read_xdc ./ddr.xdc


    if { ![file isdirectory "ip"]} {
        file mkdir ./ip
    }

    #async interconnect MIG<->Cache
    if { [file isdirectory "ip/axi_interconnect_0"] } {
        read_ip ./ip/axi_interconnect_0/axi_interconnect_0.xci
        report_property [get_files ./ip/axi_interconnect_0/axi_interconnect_0.xci]
    } else {
        create_ip -name axi_interconnect -vendor xilinx.com -library ip -version 1.7 -module_name axi_interconnect_0 -dir ./ip -force
        
        report_property [get_ips axi_interconnect_0]
        
        set_property -dict [list CONFIG.NUM_SLAVE_PORTS {1} CONFIG.S00_AXI_IS_ACLK_ASYNC {1} CONFIG.S00_AXI_READ_FIFO_DEPTH {32} CONFIG.S00_AXI_DATA_WIDTH {32}] [get_ips axi_interconnect_0]
        
        generate_target {instantiation_template} [get_files ./ip/axi_interconnect_0/axi_interconnect_0.xci]
        generate_target all [get_files ./ip/axi_interconnect_0/axi_interconnect_0.xci]
        
        
        read_ip ./ip/axi_interconnect_0/axi_interconnect_0.xci
        report_property [get_files ./ip/axi_interconnect_0/axi_interconnect_0.xci]
        
        synth_ip [get_files ./ip/axi_interconnect_0/axi_interconnect_0.xci]
    }
    
    if { [file isdirectory "ip/ddr4_0"] } {
	read_ip ./ip/ddr4_0/ddr4_0.xci
        report_property [get_files ./ip/ddr4_0/ddr4_0.xci]
    } else {
        create_ip -name ddr4 -vendor xilinx.com -library ip -version 2.2 -module_name ddr4_0 -dir ./ip -force
        
        report_property [get_ips ddr4_0]

        set_property -dict [list CONFIG.C0.DDR4_TimePeriod {1250} CONFIG.C0.DDR4_InputClockPeriod {4000} CONFIG.C0.DDR4_CLKOUT0_DIVIDE {5} CONFIG.C0.DDR4_MemoryPart {EDY4016AABG-DR-F} CONFIG.C0.DDR4_DataWidth {32} CONFIG.C0.DDR4_AxiSelection {true} CONFIG.C0.DDR4_CasLatency {11} CONFIG.C0.DDR4_CasWriteLatency {11} CONFIG.C0.DDR4_AxiDataWidth {32} CONFIG.C0.DDR4_AxiAddressWidth {30} CONFIG.ADDN_UI_CLKOUT1_FREQ_HZ {100} CONFIG.C0.BANK_GROUP_WIDTH {1}] [get_ips ddr4_0]
	
        generate_target {instantiation_template} [get_files ./ip/ddr4_0/ddr4_0.xci]
        generate_target all [get_files ./ip/ddr4_0/ddr4_0.xci]


        read_ip ./ip/ddr4_0/ddr4_0.xci

        report_property [get_files ./ip/ddr4_0/ddr4_0.xci]
        
        synth_ip [get_files ./ip/ddr4_0/ddr4_0.xci]
    }

}

read_xdc ./synth_system.xdc

if { [lindex $argv 0] == {LOOPBACK} || [lindex $argv 1] == {LOOPBACK} } {
   synth_design -part xcku040-fbva676-1-c -top top_system -verilog_define "XILINX" -verilog_define "LOOPBACK" 
} else {
   synth_design -part xcku040-fbva676-1-c -top top_system -verilog_define "XILINX"
}

opt_design
place_design
route_design

report_utilization
report_timing

write_bitstream -force synth_system.bit
write_verilog -force synth_system.v
