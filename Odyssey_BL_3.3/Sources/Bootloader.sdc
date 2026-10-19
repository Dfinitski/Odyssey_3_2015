#**************************************************************
# Odyssey-2 Bootloader 3.3  -  Bootloader.sdc
# Cyclone IV E EP4CE55F23C8  <->  KSZ9031RNX (RGMII, 1000BASE-T)
#**************************************************************
#
# Clock domains
#   INA_CLK 122.88 MHz -> PLL_SLOW c0 80 kHz  : pic_control, phy_cfg/MDIO, Reconfigure
#   PHY_CLK_125 (CLK125_NDO) -> PLL_125
#        c0 125 MHz  0 deg  : RGMII TX data (net_tx, ddio_out)
#        c1 125 MHz 90 deg  : GTX_CLK forwarding (ddio_out1 -> PHY_TX_CLOCK)
#        c2 12.5 MHz        : flash, LEDs
#   PHY_RX_CLOCK 125 MHz    : RGMII RX (ddio_in), net_rx, net_proc
#
# The three groups are mutually asynchronous. Every crossing in the RTL is
# either a 2-FF synchroniser (cdc_sync, toggles/levels) or a quasi-static
# bus that is guaranteed stable for many destination clocks before it is
# sampled (handshake by toggle), or a dual-clock M9K page RAM.
#

set_time_format -unit ns -decimal_places 3

#**************************************************************
# Base clocks
#**************************************************************
create_clock -name INA_CLK      -period 8.138 [get_ports {INA_CLK}]
create_clock -name PHY_CLK_125  -period 8.000 [get_ports {PHY_CLK_125}]
create_clock -name PHY_RX_CLOCK -period 8.000 [get_ports {PHY_RX_CLOCK}]

# virtual launch clock for RGMII RX data (same frequency/phase as RX_CLK at the pin)
create_clock -name rx_virt      -period 8.000

derive_pll_clocks
derive_clock_uncertainty

set clk_tx    {PLL_125|altpll_component|auto_generated|pll1|clk[0]}
set clk_tx90  {PLL_125|altpll_component|auto_generated|pll1|clk[1]}
set clk_12    {PLL_125|altpll_component|auto_generated|pll1|clk[2]}
set clk_slow  {PLL_SLOW|altpll_component|auto_generated|pll1|clk[0]}

# GTX_CLK as it appears on the PHY_TX_CLOCK pin (forwarded through ddio_out1)
create_generated_clock -name gtx_clk_out \
    -source [get_pins {PLL_125|altpll_component|auto_generated|pll1|clk[1]}] \
    [get_ports {PHY_TX_CLOCK}]

#**************************************************************
# Clock groups
#**************************************************************
set_clock_groups -asynchronous \
    -group [get_clocks [list INA_CLK $clk_slow]] \
    -group [get_clocks [list PHY_CLK_125 $clk_tx $clk_tx90 $clk_12 gtx_clk_out]] \
    -group [get_clocks {PHY_RX_CLOCK rx_virt}]

#**************************************************************
# RGMII receive  (KSZ9031 -> FPGA), DDR, clock centred by the PHY
#**************************************************************
# KSZ9031 default (RGMII v2.0 with internal delay): RX_CLK is delayed by the
# PHY (1.2 ns TYPICAL, sect. 3.9.3), table 7-1 gives TsetupT/TholdT >= 1.2 ns.
# Because the 1.2 ns is only typical, setup is taken 0.2 ns tighter (1.0).
# If setup slack is short, add RX_CLK pad skew in phy_cfg.v (SKEW_CLK[4:0],
# +60 ps/step) and raise rgmii_rx_tsu / lower rgmii_rx_th by the same amount.
# rgmii_rx_board = max skew between RX_CLK and RXD/RX_CTL traces.
set rgmii_rx_tsu   1.0
set rgmii_rx_th    1.2
set rgmii_rx_board 0.1

# Data captured at edge C was launched by rx_virt at C-4 ns (opposite edge):
#   latest arrival  = C - tsu  ->  max delay = 4 - tsu (+board)
#   earliest change of the next data (launched at C) = C + th -> min delay = th (-board)
set rx_max [expr 4.0 - $rgmii_rx_tsu + $rgmii_rx_board]
set rx_min [expr $rgmii_rx_th - $rgmii_rx_board]

set rx_ports [get_ports {PHY_RX[*] PHY_RX_DV}]
set_input_delay -clock rx_virt -max $rx_max $rx_ports
set_input_delay -clock rx_virt -min $rx_min $rx_ports
set_input_delay -clock rx_virt -max $rx_max $rx_ports -clock_fall -add_delay
set_input_delay -clock rx_virt -min $rx_min $rx_ports -clock_fall -add_delay

# setup: opposite edge (rise->fall, fall->rise); hold: same edge
set_false_path -setup -rise_from [get_clocks rx_virt] -rise_to [get_clocks PHY_RX_CLOCK]
set_false_path -setup -fall_from [get_clocks rx_virt] -fall_to [get_clocks PHY_RX_CLOCK]
set_false_path -hold  -rise_from [get_clocks rx_virt] -fall_to [get_clocks PHY_RX_CLOCK]
set_false_path -hold  -fall_from [get_clocks rx_virt] -rise_to [get_clocks PHY_RX_CLOCK]

#**************************************************************
# RGMII transmit  (FPGA -> KSZ9031), DDR, clock centred by the FPGA (90 deg)
#**************************************************************
# KSZ9031 adds no delay on GTX_CLK/TXD by default: the MAC must deliver a
# centred clock. Requirement at the PHY pins (table 7-1, receiver side):
# TsetupR >= 1.0 ns, TholdR >= 1.0 ns.
set rgmii_tx_tsu   1.0
set rgmii_tx_th    1.0
set rgmii_tx_board 0.1

set tx_ports [get_ports {PHY_TX[*] PHY_TX_EN}]
set_output_delay -clock gtx_clk_out -max [expr  $rgmii_tx_tsu + $rgmii_tx_board] $tx_ports
set_output_delay -clock gtx_clk_out -min [expr -$rgmii_tx_th  - $rgmii_tx_board] $tx_ports
set_output_delay -clock gtx_clk_out -max [expr  $rgmii_tx_tsu + $rgmii_tx_board] $tx_ports -clock_fall -add_delay
set_output_delay -clock gtx_clk_out -min [expr -$rgmii_tx_th  - $rgmii_tx_board] $tx_ports -clock_fall -add_delay

# data launched on c0 rise is captured by gtx rise 2 ns later (same edge):
# setup same edge, hold opposite edge
set_false_path -setup -rise_from [get_clocks $clk_tx] -fall_to [get_clocks gtx_clk_out]
set_false_path -setup -fall_from [get_clocks $clk_tx] -rise_to [get_clocks gtx_clk_out]
set_false_path -hold  -rise_from [get_clocks $clk_tx] -rise_to [get_clocks gtx_clk_out]
set_false_path -hold  -fall_from [get_clocks $clk_tx] -fall_to [get_clocks gtx_clk_out]

# the forwarded clock itself is a clock path, not a data path
set_false_path -to [get_ports {PHY_TX_CLOCK}]

#**************************************************************
# EPCS (bit-banged in the 12.5 MHz domain, >= 80 ns between any data
# change and the DCLK edge, all pins from/to IOE registers - see .qsf)
#**************************************************************
set_false_path -to   [get_ports {DCLK DATAOUT FLASH_NCE}]
set_false_path -from [get_ports {DATAIN}]

#**************************************************************
# Slow / asynchronous I/O
#**************************************************************
# MDIO/MDC run at 80 kHz (MDC is the clock itself), reset, MCU UART, keys,
# relays, LEDs, test pins - none of them needs I/O timing.
set_false_path -to   [get_ports {PHY_MDC PHY_MDIO PHY_RESET_N NCONFIG}]
set_false_path -from [get_ports {PHY_MDIO}]
set_false_path -to   [get_ports {UO[*] BAND[*] TUNE ANT_2 ANT_3 PTT_out PTT_out_mcu LNA u_tx}]
set_false_path -to   [get_ports {led1 led2 led3 led4 test1 test2}]
set_false_path -from [get_ports {KEY_DASH KEY_DOT PTT_in_UO PTT_in u_rx random_bit}]
