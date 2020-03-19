create_clock -period 10.00 [get_ports clk]

create_clock -period 40.00 [get_ports ENET_RX_CLK]

derive_pll_clocks

derive_clock_uncertainty

set_clock_groups -asynchronous \
-group [get_clocks clk] \
-group [get_clocks ENET_RX_CLK] 
