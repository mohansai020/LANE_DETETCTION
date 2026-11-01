create_clock -name CLOCK_50 -period 20.000 [get_ports {CLOCK_50}]
create_clock -name OV_PCLK  -period 20.000 [get_ports {OV_PCLK}]
derive_pll_clocks
derive_clock_uncertainty
set_clock_groups -asynchronous -group [get_clocks {CLOCK_50}] -group [get_clocks {OV_PCLK}] -group [get_clocks {*pll1|clk[0]}] -group [get_clocks {*pll1|clk[1]}]
set_input_delay -clock OV_PCLK -max 15.0 [get_ports {OV_D[*] OV_HREF OV_VSYNC}]
set_input_delay -clock OV_PCLK -min 10.0 [get_ports {OV_D[*] OV_HREF OV_VSYNC}]
set_false_path -from [get_ports {KEY[*] SW[*] OV_SIOD}]
set_false_path -to [get_ports {LEDR[*] LEDG[*] HEX* VGA_* OV_XCLK OV_RESET_N OV_PWDN OV_SIOC OV_SIOD}]
