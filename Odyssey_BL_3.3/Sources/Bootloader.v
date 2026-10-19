//
// Bootloader V3.3 for Odyssey-2 project
//
// N7DDC, David Fainitski, April 2018  (original 2.0 / 3.2)
// 3.3 (2026): network core rewritten - FCS check, clean clock-domain
//             crossings, GTX_CLK forwarded through DDIO, ARP filter fix,
//             page read-back, safe retransmission, PHY configuration.
//

/*
	EPCS64 : 8 MB, 128 sectors x 64 KB, 32768 pages x 256 bytes

	Slot 0 : sectors   0.. 31  H'000000..H'1FFFFF   pages     0.. 8191   Bootloader (factory)
	Slot 1 : sectors  32.. 63  H'200000..H'3FFFFF   pages  8192..16383   firmware 1
	Slot 2 : sectors  64.. 95  H'400000..H'5FFFFF   pages 16384..24575   firmware 2
	Slot 3 : sectors  96..127  H'600000..H'7FFFFF   pages 24576..32767   firmware 3

	UDP port 50000, every request/reply has a 32-byte command block:

	MAC  request  'MAC' SN 28x0
	     reply    'MAC' SN ver[3] caps[1] 14x0 IP[4] MAC[6]     (2.3 tools read MAC at 26..31)
	     Also answered when sent to the Ethernet broadcast (discovery).
	WIP  request  'WIP' SN 24x0 IP[4]          reply 'WIP' SN 24x0 IP[4]
	     also as Ethernet broadcast 'WIP' SN 18x0 MAC[6] IP[4] - accepted only for our MAC
	     (change the static IP from any subnet). IP 0.0.0.0 / 255.255.255.255 rejected.
	ERS  request  'ERS' SN A1 A0 26x0          reply same, after 32 sectors are erased
	WPD  request  'WPD' SN A1 A0 26x0 + 256 B  reply 'WPD' SN A1 A0 26x0 after programming
	RPD  request  'RPD' SN A1 A0 26x0          reply 'RPD' SN A1 A0 26x0 + 256 B   (3.3)
	RES  request  'RES' 29x0                   no reply, MCU resets the radio

	A1:A0 = page number. Data bytes are plain .rbf bytes (bit reversal for the
	AS configuration flash is done in Flash.v).

	caps (3.3) : bit0 RPD, bit1 safe retransmission, bit2 discovery, bit3 FCS check, bit4 broadcast WIP
*/

module Bootloader (
// Clock
input INA_CLK,              //122.88MHz from ADC

// PHY
output  	wire     [3:0]PHY_TX,
input   	wire     [3:0]PHY_RX,
input		wire     PHY_RX_DV,				// RGMII RX_CTL
output	wire     PHY_TX_CLOCK,			// RGMII GTX_CLK
output	wire 	 	PHY_TX_EN,				// RGMII TX_CTL
input		wire	 	PHY_RX_CLOCK,      	// RGMII RX_CLK
input		wire     PHY_CLK_125,			// 125MHz CLK125_NDO from PHY
inout		wire     PHY_MDIO,
output	wire 	 	PHY_MDC,
output	wire     PHY_RESET_N,

// FLASH interface
output DCLK,
output DATAOUT,
input DATAIN,
output FLASH_NCE,

// Reload
output  	wire     NCONFIG,

// input
input KEY_DOT,
input KEY_DASH,
input PTT_in_UO,
input PTT_in,
input random_bit,

// output
output [7:0]UO,
output [2:0] BAND,
output TUNE,
output ANT_2,
output ANT_3,
output PTT_out,
output PTT_out_mcu,
output LNA,

// UART connection
input u_rx,
output u_tx,

// LEDs
output led1,
output led2,
output led3,
output led4,
output  test1,
output  test2
);

assign NCONFIG     = 1'd0;
assign BAND        = 3'd0;
assign PTT_out_mcu = 1'b0;
assign LNA         = 1'b0;

parameter [23:0] fw_version = "3.3";

//--------------------------------------------------------
// 					Clocks
//--------------------------------------------------------
//  PHY_CLK_125 -> PLL_125 : c0 125 MHz 0 deg   (tx_clock, TX data)
//                           c1 125 MHz 90 deg  (GTX_CLK forwarding, 2.0 ns)
//                           c2 12.5 MHz        (flash, LEDs)
//  INA_CLK     -> PLL_SLOW: c0 80 kHz          (MCU UART, MDIO, remote update)
//  PHY_RX_CLOCK             125 MHz            (RX, packet processing)

wire C125_0_deg, C125_90_deg;
wire C12_5MHz;
wire pll125_locked;
reg  pll125_areset = 1'b1;
C125_PLL PLL_125(.areset(pll125_areset), .inclk0(PHY_CLK_125), .c0(C125_0_deg), .c1(C125_90_deg),
                 .c2(C12_5MHz), .locked(pll125_locked));

wire C_80kHz;
wire clock_ready;
SLOW_PLL PLL_SLOW (.inclk0(INA_CLK), .c0(C_80kHz), .locked(clock_ready));

wire tx_clock = C125_0_deg;
wire rx_clock = PHY_RX_CLOCK;

assign test1 = INA_CLK;
assign test2 = C_80kHz;

//--------------------------------------------------------
// Relay / UO "running light" (80 kHz domain)
//--------------------------------------------------------
reg [23:0] test_cnt = 24'd16000;  // 0.2s
reg [4:0] blink_cnt = 0;

always @(posedge C_80kHz)
begin
   if(test_cnt != 0) test_cnt <= test_cnt - 1'd1;
	else
	begin
	   test_cnt <= 24'd16000;
		blink_cnt <= blink_cnt + 1'd1;
	end
   // NOTE (unchanged from 3.2): state 11 lasts one 80 kHz clock only, so
   // PTT_out is never really keyed by the running light.
   if (blink_cnt == 11) blink_cnt <= 1'd0;
end

assign UO[0]   = PTT_in_UO ? 1'b0 : blink_cnt == 0;
assign UO[1]   = PTT_in_UO ? 1'b0 : blink_cnt == 1;
assign UO[2]   = PTT_in_UO ? 1'b0 : blink_cnt == 2;
assign UO[3]   = PTT_in_UO ? 1'b0 : blink_cnt == 3;
assign UO[4]   = PTT_in_UO ? 1'b0 : blink_cnt == 4;
assign UO[5]   = PTT_in_UO ? 1'b0 : blink_cnt == 5;
assign UO[6]   = PTT_in_UO ? 1'b0 : blink_cnt == 6;
assign UO[7]   = PTT_in_UO ? 1'b0 : blink_cnt == 7;
assign TUNE    = PTT_in_UO ? 1'b0 : blink_cnt == 8;
assign ANT_2   = PTT_in_UO ? 1'b0 : blink_cnt == 9;
assign ANT_3   = PTT_in_UO ? 1'b0 : blink_cnt == 10;
assign PTT_out = PTT_in_UO ? 1'b0 : blink_cnt == 11;

//---------------------------------------------------------------------------------
// Connection to MCU (80 kHz domain)
//---------------------------------------------------------------------------------
wire [23:0] start_addr;
wire [31:0] local_IP;
wire [47:0] local_MAC;
wire [15:0] local_Port = 16'd50000;
wire addr_ready;
wire bl_mode;
wire reconfig_req;
wire write_ip_t, dev_reset_t;
wire [31:0] ip_to_write;

pic_control #(fw_version) pic_cntrl (.run(clock_ready), .clock(C_80kHz), ._write_ip(write_ip_t), ._ip_to_write(ip_to_write),
                                     .u_rx(u_rx), .u_tx(u_tx),
                                     .IP(local_IP), .MAC(local_MAC), .start_addr(start_addr), .addr_ready(addr_ready),
                                     .bl_mode(bl_mode), .bl_req(~KEY_DOT & ~KEY_DASH),
                                     .reconfig_req(reconfig_req), ._reset_req(dev_reset_t), ._random_bit(random_bit));

// PHY held in reset until the MCU confirmed bootloader mode
assign PHY_RESET_N = bl_mode;

// CLK125_NDO only starts after the PHY leaves reset: hold PLL_125 in reset
// until 10 ms after RESET_N went high (Cyclone IV: reset the PLL after a
// late-starting input clock is stable).
reg [9:0] pll_rst_cnt = 10'd0;
always @(posedge C_80kHz)
  if (!bl_mode)                  begin pll_rst_cnt <= 10'd0; pll125_areset <= 1'b1; end
  else if (pll_rst_cnt != 10'd800) begin pll_rst_cnt <= pll_rst_cnt + 1'd1; pll125_areset <= 1'b1; end
  else                             pll125_areset <= 1'b0;

//----------------------------------------------------------------------------------
// PHY MDIO configuration (KSZ9031, MDC = 80 kHz)
//----------------------------------------------------------------------------------
wire phy_cfg_done, phy_found;
wire [4:0] phy_addr;
phy_cfg phy_config(.clock(C_80kHz), .run(bl_mode), .mdio_pin(PHY_MDIO), .mdc_pin(PHY_MDC),
                   .done(phy_cfg_done), .found(phy_found), .phy_addr(phy_addr));

//------------------------------------------------------------------------------
//  Remote Update (80 kHz domain)
//------------------------------------------------------------------------------
Reconfigure Recon_inst(.reset(1'd0), .clock(C_80kHz), .BootAddress(start_addr),
                       .control(reconfig_req), .CRC_error(), .done(), .addr_ready(addr_ready));

//------------------------------------------------------------------------------
//  80 kHz -> rx_clock : IP/MAC valid flag (IP/MAC buses are quasi-static)
//------------------------------------------------------------------------------
wire ip_ready_rx;
// nothing is answered before IP/MAC are valid and the TX/flash PLL is locked
cdc_sync #(1) ipr_sync (.siga(addr_ready & bl_mode & pll125_locked & ~pll125_areset), .rstb(1'b0), .clkb(rx_clock), .sigb(ip_ready_rx));

//---------------------------------------------------------------------------------
// Receive (rx_clock)
//---------------------------------------------------------------------------------
wire        speed, duplex, link_ok, network_rx_active;
wire        acc, acc_arp, acc_icmp, acc_udp, acc_bcast, acc_long, acc_page_ok;
wire [42*8-1:0] rx_hdr;
wire [32*8-1:0] rx_cmd;
wire        pg_we;
wire [7:0]  pg_addr, pg_data;
wire        page_wr_allow;

net_rx rx_inst (
  .clk(rx_clock), .rxd_pin(PHY_RX), .rxctl_pin(PHY_RX_DV),
  .local_mac(local_MAC), .local_ip(local_IP), .local_port(local_Port),
  .page_wr_allow(page_wr_allow),
  .link_1000(speed), .full_duplex(duplex), .link_ok(link_ok), .active(network_rx_active),
  .acc(acc), .acc_arp(acc_arp), .acc_icmp(acc_icmp), .acc_udp(acc_udp), .acc_bcast(acc_bcast),
  .acc_long(acc_long), .acc_page_ok(acc_page_ok), .hdr(rx_hdr), .cmd(rx_cmd),
  .pg_we(pg_we), .pg_addr(pg_addr), .pg_data(pg_data)
);

//---------------------------------------------------------------------------------
// Packet processing (rx_clock)
//---------------------------------------------------------------------------------
wire [42*8-1:0] txb_hdr;
wire [32*8-1:0] txb_pay;
wire [1:0]  txb_kind;
wire        tx_start_t, tx_done_t;
wire        fl_req_t, fl_done_t;
wire [1:0]  fl_op;
wire [15:0] fl_page;

net_proc #(.FW_VERSION(fw_version)) proc_inst (
  .clk(rx_clock), .local_mac(local_MAC), .local_ip(local_IP), .local_port(local_Port),
  .ip_ready(ip_ready_rx), .link_ok(link_ok),
  .acc(acc), .acc_arp(acc_arp), .acc_icmp(acc_icmp), .acc_udp(acc_udp), .acc_bcast(acc_bcast),
  .acc_long(acc_long), .acc_page_ok(acc_page_ok), .hdr(rx_hdr), .cmd(rx_cmd),
  .page_wr_allow(page_wr_allow),
  .txb_hdr(txb_hdr), .txb_pay(txb_pay), .txb_kind(txb_kind), .tx_start_t(tx_start_t), .tx_done_t(tx_done_t),
  .fl_req_t(fl_req_t), .fl_op(fl_op), .fl_page(fl_page), .fl_done_t(fl_done_t),
  .write_ip_t(write_ip_t), .ip_to_write(ip_to_write), .dev_reset_t(dev_reset_t),
  .busy_flash(), .busy_tx()
);

//--------------------------------------------------------------------------------
// Page buffers (M9K, dual clock)
//   wr page : rx_clock -> C12_5MHz  (WPD payload to flash)
//   rd page : C12_5MHz -> tx_clock  (flash read-back for RPD)
//--------------------------------------------------------------------------------
wire [7:0] wr_raddr, wr_q;
wire       rd_we;
wire [7:0] rd_waddr, rd_wdata, rd_raddr, rd_q;

page_ram wr_page (.wclk(rx_clock), .we(pg_we), .waddr(pg_addr), .wdata(pg_data),
                  .rclk(C12_5MHz), .raddr(wr_raddr), .q(wr_q));

page_ram rd_page (.wclk(C12_5MHz), .we(rd_we), .waddr(rd_waddr), .wdata(rd_wdata),
                  .rclk(tx_clock), .raddr(rd_raddr), .q(rd_q));

//--------------------------------------------------------------------------------
// FLASH memory operation (12.5 MHz)
//--------------------------------------------------------------------------------
wire flash_busy;
flash flash_inst(.clock(C12_5MHz), .req_t(fl_req_t), .op(fl_op), .page(fl_page), .done_t(fl_done_t), .busy(flash_busy),
                 .wr_raddr(wr_raddr), .wr_q(wr_q), .rd_we(rd_we), .rd_waddr(rd_waddr), .rd_wdata(rd_wdata),
                 .DCLK(DCLK), .DATAOUT(DATAOUT), .DATAIN(DATAIN), .FLASH_NCE(FLASH_NCE));

//--------------------------------------------------------------------------------------
// Transmit (tx_clock)
//--------------------------------------------------------------------------------------
wire network_tx_active;

net_tx tx_inst (
  .clk(tx_clock), .txb_hdr(txb_hdr), .txb_pay(txb_pay), .txb_kind(txb_kind),
  .start_t(tx_start_t), .done_t(tx_done_t), .active(network_tx_active),
  .ram_raddr(rd_raddr), .ram_q(rd_q),
  .PHY_TX(PHY_TX), .PHY_TX_EN(PHY_TX_EN)
);

// GTX_CLK: forwarded through a DDIO output (same IOE path as the data),
// clocked by the 90 deg PLL output -> clock edge in the middle of the data eye.
ddio_out1 gtx_clk_ddio (
  .datain_h (1'b1),
  .datain_l (1'b0),
  .outclock (C125_90_deg),
  .dataout  (PHY_TX_CLOCK)
);

//-----------------------------------------------------------
//  		LED Control  (12.5 MHz domain)
//-----------------------------------------------------------
wire rx_act_s, tx_act_s, speed_s, duplex_s, bl_mode_s;
cdc_sync #(5) led_sync (.siga({network_rx_active, network_tx_active, speed, duplex, bl_mode}),
                        .rstb(1'b0), .clkb(C12_5MHz), .sigb({rx_act_s, tx_act_s, speed_s, duplex_s, bl_mode_s}));

wire STATUS_LED;
wire DEBUG_LED1;
wire DEBUG_LED2;
wire DEBUG_LED3;

parameter half_second = 2500000; // at 12.5MHz clock rate
parameter dimmer = 3;  // LED bright 0 - 100 %

reg [7:0] dim_cnt = 0;
always @(posedge C12_5MHz)  if (dim_cnt != 100) dim_cnt <= dim_cnt + 1'd1; else dim_cnt <= 0;

reg led_on = 1'b0;
always @(posedge C12_5MHz) led_on <= (dim_cnt <= dimmer);

assign led1 = STATUS_LED & led_on;               // Heart Beat
assign led2 = DEBUG_LED1 & bl_mode_s & led_on;   // connection's status
assign led3 = DEBUG_LED2 & led_on;               // receive from PHY
assign led4 = DEBUG_LED3 & led_on;               // transmit to PHY

Led_flash Flash_LED1(.clock(C12_5MHz), .signal(rx_act_s), .LED(DEBUG_LED2), .period(half_second));
Led_flash Flash_LED2(.clock(C12_5MHz), .signal(tx_act_s), .LED(DEBUG_LED3), .period(half_second));

//Flash STATUS_LED
reg [26:0]HB_counter = 27'd0;
always @(posedge PHY_CLK_125) HB_counter <= HB_counter + 1'b1;
assign STATUS_LED = HB_counter[25];  // Blink

parameter clock_speed = 12_500_000; 	// 12.5MHz clock

// fast flash if no Ethernet connection, slow flash if 100T and on if 1000T
Led_control #(clock_speed) Control_LED0(.clock(C12_5MHz), .on(speed_s & duplex_s), .fast_flash(!duplex_s),
                                        .slow_flash(!speed_s & duplex_s), .LED(DEBUG_LED1));

endmodule
