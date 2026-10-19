# Odyssey.sdc
#          


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3
derive_pll_clocks
derive_clock_uncertainty


#**************************************************************
# Input  Clock
#**************************************************************

create_clock -name INA_CLK 	   	-period 8.138	INA_CLK

create_clock -name {PHY_CLK_125} -period 8.000 -waveform { 0.000 4.000 } [get_ports {PHY_CLK_125}]

create_clock -name {PHY_RX_CLOCK} -period 8.000 -waveform { 2.000 6.000 } [get_ports {PHY_RX_CLOCK}]

set_clock_groups -exclusive -group {PHY_RX_CLOCK}

create_clock -name {virt_PHY_RX_CLOCK} -period 8.000 -waveform { 0.000 4.000 } 

set_clock_groups -exclusive -group {virt_PHY_RX_CLOCK}


#**************************************************************
# PLL generated
#**************************************************************

set C_80kHz       PLL_SLOW|altpll_component|auto_generated|pll1|clk[1]

set C125_0_deg    PLL_125|altpll_component|auto_generated|pll1|clk[0]
set C125_90_deg   PLL_125|altpll_component|auto_generated|pll1|clk[1]
set C12_5MHz      PLL_125|altpll_component|auto_generated|pll1|clk[2]

#create genenerated clock for internal PHY Tx data clock.
create_generated_clock -source [get_pins {PLL_125|altpll_component|auto_generated|pll1|inclk[0]}] \
  -name tx_clock -duty_cycle 50.00 [get_pins {PLL_125|altpll_component|auto_generated|pll1|clk[0]}] -add
 
set_clock_groups -exclusive -group {tx_clock}
  
#**************************************************************
# Create Generated Clock (internal to the FPGA)
#**************************************************************
# clock for flash
create_generated_clock -name flash_clock -source $C12_5MHz -divide 2 

set_output_delay -add_delay -clock flash_clock 10 {DCLK DATAOUT FLASH_NCE}

set_input_delay  -add_delay -clock flash_clock 10 {DATAIN}

#create generated clock for PLL transmit clock output with 90 phase shift
create_generated_clock -source [get_pins {PLL_125|altpll_component|auto_generated|pll1|inclk[0]}] \
  -name PHY_TX_CLOCK -phase 150.00 -duty_cycle 50.00 [get_pins {PLL_125|altpll_component|auto_generated|pll1|clk[1]}] -add

set_clock_groups -exclusive -group {PHY_TX_CLOCK}

#************************************************************** 
# Set Input Delay
#**************************************************************

# data from PHY
set_input_delay  -max 0.8  -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX[*]}] 
set_input_delay  -min -0.8 -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX[*]}] 				
set_input_delay  -max 0.8 -clock_fall -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX[*]}]	
set_input_delay  -min -0.8 -clock_fall -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX[*]}]	

set_input_delay  -max 0.8  -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX_DV}] 
set_input_delay  -min -0.8 -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX_DV}] 				
set_input_delay  -max 0.8 -clock_fall -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX_DV}]	
set_input_delay  -min -0.8 -clock_fall -clock [get_clocks virt_PHY_RX_CLOCK] -add_delay [get_ports {PHY_RX_DV}]

# Set false paths to remove irrelevant setup and hold analysis
set_false_path -fall_from [get_clocks virt_PHY_RX_CLOCK] -rise_to [get_clocks {PHY_RX_CLOCK}] -setup
set_false_path -rise_from [get_clocks virt_PHY_RX_CLOCK] -fall_to [get_clocks {PHY_RX_CLOCK}] -setup
set_false_path -fall_from [get_clocks virt_PHY_RX_CLOCK] -fall_to [get_clocks {PHY_RX_CLOCK}] -hold
set_false_path -rise_from [get_clocks virt_PHY_RX_CLOCK] -rise_to [get_clocks {PHY_RX_CLOCK}] -hold


#**************************************************************
# Set Output Delay
#**************************************************************

set_output_delay  -max 1.0  -clock PHY_TX_CLOCK [get_ports {PHY_TX[*]}]  -add_delay
set_output_delay  -min -0.8 -clock PHY_TX_CLOCK [get_ports {PHY_TX[*]}]  -add_delay
set_output_delay  -max 1.0  -clock PHY_TX_CLOCK [get_ports {PHY_TX[*]}]  -clock_fall -add_delay
set_output_delay  -min -0.8 -clock PHY_TX_CLOCK [get_ports {PHY_TX[*]}]  -clock_fall -add_delay 

set_output_delay  -max 1.0  -clock PHY_TX_CLOCK [get_ports {PHY_TX_EN}] -add_delay
set_output_delay  -min -0.8 -clock PHY_TX_CLOCK [get_ports {PHY_TX_EN}]  -add_delay
set_output_delay  -max 1.0  -clock PHY_TX_CLOCK [get_ports {PHY_TX_EN}]  -clock_fall -add_delay
set_output_delay  -min -0.8 -clock PHY_TX_CLOCK [get_ports {PHY_TX_EN}]  -clock_fall -add_delay

# Set false paths to remove irrelevant setup and hold analysis
set_false_path -fall_from [get_clocks tx_clock] -rise_to [get_clocks PHY_TX_CLOCK] -setup
set_false_path -rise_from [get_clocks tx_clock] -fall_to [get_clocks PHY_TX_CLOCK] -setup
set_false_path -fall_from [get_clocks tx_clock] -fall_to [get_clocks PHY_TX_CLOCK] -hold
set_false_path -rise_from [get_clocks tx_clock] -rise_to [get_clocks PHY_TX_CLOCK] -hold

#**************************************************************
# Set Clock Groups
#**************************************************************
set_clock_groups -asynchronous  -group { \
					INA_CLK    \
					PLL_SLOW|altpll_component|auto_generated|pll1|clk[0]  \
				       } \
				-group { \
					PHY_RX_CLOCK \
					PHY_CLK_125 \
					PLL_125|altpll_component|auto_generated|pll1|clk[0]   \
					PLL_125|altpll_component|auto_generated|pll1|clk[1]   \
					PLL_125|altpll_component|auto_generated|pll1|clk[2]   \
					PHY_TX_CLOCK \
					tx_clock \
					flash_clock  \
				       } 

								
					
#**************************************************************
# Set Maximum Delay (for setup or recovery; low-level, over-riding timing adjustments)
#************************************************************** 

set_max_delay -from PHY_RX_CLOCK -to PHY_RX_CLOCK 15

set_max_delay -from PLL_125|altpll_component|auto_generated|pll1|clk[0] -to PLL_125|altpll_component|auto_generated|pll1|clk[0] 9

set_max_delay -from tx_clock -to tx_clock 9

#**************************************************************
# Set Minimum Delay (for hold or removal; low-level, over-riding timing adjustments)
#**************************************************************

set_min_delay -from PHY_RX_CLOCK -to PHY_RX_CLOCK -1




#**************************************************************
# Set False Paths
#**************************************************************

#set_false_path -from [get_clocks {PLL_inst|altpll_component|auto_generated|pll1|clk[1] }] -to [get_clocks {network_inst|rgmii_send_inst|tx_pll_inst|altpll_component|auto_generated|pll1|clk[2]}]

# Set false path to generated clocks that feed output pins
set_false_path -to [get_ports {PHY_TX_CLOCK}]

# don't need fast paths to the outputs so set false paths so Timing will be ignored
set_false_path -to {UO[*] PHY_MDC ANT_2 ANT_3 TUNE PTT_out u_tx led1 led2 led3 led4 test1 test2 PHY_RESET_N PHY_MDIO}

#don't need fast paths from the following inputs
set_false_path -from  {KEY_DASH KEY_DOT PTT_in_UO u_rx PHY_MDIO random_bit}



