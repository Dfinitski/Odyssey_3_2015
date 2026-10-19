//
//  HPSDR - High Performance Software Defined Radio
//
//  Metis code.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


//  Metis code copyright 2010, 2011, 2012, 2013 Phil Harman VK6APH, Alex Shovkoplyas, VE3NEA.
//
//  Odyssey-III, 2026 rework of the network glue:
//   * DHCP works: DHCP data/length/ports/IP/MAC are now multiplexed into
//     udp_send/ip_send/mac_send (before, DHCP frames were sent with the SDR
//     payload and SDR addresses, and port 68 was dropped on receive)
//   * DHCP when the MCU sends IP 0.0.0.0; source IP 0.0.0.0 while asking;
//     lease renewal at lease/2 WITHOUT resetting the radio; if the renewal
//     fails the old IP is kept and DHCP is retried; APIPA 169.254.1.1 only if
//     no server answered at start
//   * ICMP echo (ping) enabled
//   * gratuitous ARP twice after the IP becomes valid
//   * rx-domain copies of remote MAC/IP for arp/icmp/dhcp (were tx-synchronised
//     buses sampled again in the rx domain)
//   * tx_pll reset input / lock output
//

module network #(
  parameter [21:0] TICKS_PER_SEC = 22'd2_500_000,   // clock_2_5MHz ticks per second
  parameter [31:0] MIN_RENEW     = 32'd60,          // minimum seconds until renewal
  parameter [27:0] GARP_DLY1     = 28'd25_000_000,  // tx clocks: 0.2 s
  parameter [27:0] GARP_DLY2     = 28'd250_000_000  // tx clocks: 2 s
)(

	//input
	input udp_tx_request,
	input [15:0] udp_tx_length,
	input [7:0] udp_tx_data,
	input [7:0] port_ID,
	input [31:0] static_ip,
	input pll_areset,                 // hold the TX PLL in reset (PHY clock not stable yet)

	//output
	output rx_clock,
	output tx_clock,
	output udp_rx_active,
	output udp_tx_enable,
	output [7:0] udp_rx_data,
	output udp_tx_active,
	input [47:0] local_mac,
	output broadcast,
	output [15:0]to_port,
	output pll_locked,

   //status output
   output network_state,
   output [7:0] network_status,
   output dhcp_timeout,
   output dhcp_success,
   output dhcp_failed,

   //hardware pins
   output [3:0]PHY_TX,
   output PHY_TX_EN,
   output PHY_TX_CLOCK,
   input  [3:0]PHY_RX,
   input  PHY_DV,
   input  PHY_RX_CLOCK,
   input  PHY_CLK125,

	output test1,
   output test2
  );

localparam false = 1'b0, true = 1'b1;

wire clock_2_5MHz;
wire phy_connected;
wire dhcp_tx_request;
wire [7:0]  dhcp_tx_data;
wire [15:0] dhcp_tx_length;
wire [47:0] dhcp_destination_mac;
wire [31:0] dhcp_destination_ip;
wire [15:0] dhcp_destination_port;
wire [31:0] lease;
wire [31:0] server_ip;
wire [31:0] ip_accept;
wire [15:0] udp_destination_port;
wire [15:0] udp_destination_port_sync;
wire [47:0] udp_destination_mac;
wire [47:0] udp_destination_mac_sync;
wire [31:0] udp_destination_ip;
wire [31:0] udp_destination_ip_sync;
wire dhcp_tx_enable_tx;
wire dhcp_renewal;


//-----------------------------------------------------------------------------
//                   state machine (clock_2_5MHz domain)
//-----------------------------------------------------------------------------
reg  [31:0] local_ip = 32'd0;
wire [31:0] apipa_ip = {8'd169, 8'd254, 8'd1, 8'd1};

localparam
  ST_START         = 3'd0,
  ST_DHCP_REQUEST  = 3'd1,
  ST_DHCP_WAIT     = 3'd2,
  ST_DHCP_BOUND    = 3'd3,    // have a lease, wait for lease/2
  ST_RUNNING       = 3'd4,    // static or APIPA
  ST_DHCP_RESTART  = 3'd5;    // after a NAK: wait until the NAK flag is cleared

reg [2:0]  state = ST_START;
reg        have_ip = false;       // local_ip valid -> radio may run
reg        renew_phase = false;   // trying to renew an existing lease
reg        is_apipa = false;
reg [2:0]  tries = 3'd0;
reg [31:0] wait_secs = 32'd0;
reg [3:0]  dhcp_seconds_timer = 4'd0;   // "secs" field of the DHCP message
reg        dhcp_enable = false;          // DHCP receive enable
reg        dhcp_tx_enable = false;       // start a DHCP transmission
reg        force_discover = false;
reg        dhcp_mode = false;            // IP comes from DHCP
reg [31:0] lease_left = 32'hFFFF_FFFF;   // seconds until the lease expires

// 1 second tick
reg [21:0] tick_cnt = 22'd0;
wire sec_tick = (tick_cnt == TICKS_PER_SEC - 1'd1);
always @(posedge clock_2_5MHz) tick_cnt <= sec_tick ? 22'd0 : tick_cnt + 1'd1;

// level from the rx domain, synchronised
wire dhcp_success_s, dhcp_failed_s;
sync sync_dsucc(.clock(clock_2_5MHz), .sig_in(dhcp_success), .sig_out(dhcp_success_s));
sync sync_dfail(.clock(clock_2_5MHz), .sig_in(dhcp_failed),  .sig_out(dhcp_failed_s));
wire phy_connected_s;
sync sync_phyc (.clock(clock_2_5MHz), .sig_in(phy_connected), .sig_out(phy_connected_s));

//reset all child modules
wire rx_reset, tx_reset;
sync sync_inst1(.clock(rx_clock), .sig_in(state == ST_START), .sig_out(rx_reset));
sync sync_inst2(.clock(tx_clock), .sig_in(state == ST_START), .sig_out(tx_reset));

assign network_state = !have_ip;     // 1 = hold the SDR code in reset
assign dhcp_timeout  = is_apipa;

always @(posedge clock_2_5MHz)
  //if connection lost, wait until reconnects
  if ((state != ST_START) && !phy_connected_s) begin
    state <= ST_START;
    have_ip <= false;
    dhcp_enable <= false;
    dhcp_tx_enable <= false;
  end
  // lease expired (server unreachable for the whole lease): give the IP up
  else if (dhcp_mode && have_ip && sec_tick && lease_left == 32'd1) begin
    have_ip <= false;
    local_ip <= 32'd0;
    renew_phase <= false;
    force_discover <= true;
    tries <= 3'd4;
    dhcp_seconds_timer <= 4'd0;
    dhcp_enable <= false;
    state <= ST_DHCP_REQUEST;
  end
  else begin
  if (dhcp_mode && have_ip && sec_tick && lease_left != 32'hFFFF_FFFF) lease_left <= lease_left - 1'd1;
  case (state)
    ST_START: begin
      dhcp_enable <= false;
      dhcp_tx_enable <= false;
      is_apipa <= false;
      if (phy_connected_s) begin
        if (static_ip == 32'h00_00_00_00) begin   // MCU asks for DHCP
          dhcp_mode <= true;
          local_ip <= 32'd0;                      // DISCOVER/REQUEST from 0.0.0.0
          have_ip <= false;
          renew_phase <= false;
          force_discover <= true;
          tries <= 3'd4;
          dhcp_seconds_timer <= 4'd0;
          state <= ST_DHCP_REQUEST;
        end
        else begin
          dhcp_mode <= false;
          local_ip <= static_ip;                  // work with static IP
          have_ip <= true;
          state <= ST_RUNNING;
        end
      end
    end

    // send DISCOVER (or REQUEST when renewing)
    ST_DHCP_REQUEST: begin
      dhcp_tx_enable <= true;
      dhcp_enable <= true;
      wait_secs <= (tries == 3'd4) ? 32'd3 : 32'd2;
      state <= ST_DHCP_WAIT;
    end

    // wait for ACK; the dhcp module sends the REQUEST after an OFFER by itself
    ST_DHCP_WAIT: begin
      force_discover <= false;
      dhcp_tx_enable <= false;                  // 400 ns pulse (level is re-sampled by the dhcp FSM)
      if (sec_tick && dhcp_seconds_timer != 4'd15) dhcp_seconds_timer <= dhcp_seconds_timer + 1'd1;
      if (dhcp_success_s) begin
        local_ip <= ip_accept;
        have_ip <= true;
        renew_phase <= false;
        dhcp_enable <= false;
        dhcp_tx_enable <= false;
        lease_left <= (lease == 32'd0) ? 32'hFFFF_FFFF : lease;
        // renew at lease/2 (12 h if the server gave no lease time, >= 60 s)
        wait_secs <= (lease == 32'd0) ? 32'd43_200 : ((lease >> 1) < MIN_RENEW ? MIN_RENEW : (lease >> 1));
        state <= ST_DHCP_BOUND;
      end
      else if (dhcp_failed_s) begin              // NAK
        if (have_ip) begin                       // renewal refused: stop using the IP
          have_ip <= false;
          local_ip <= 32'd0;
        end
        renew_phase <= false;
        force_discover <= true;
        tries <= 3'd4;
        dhcp_seconds_timer <= 4'd0;
        dhcp_enable <= false;                    // clears the NAK flag in dhcp.v
        state <= ST_DHCP_RESTART;
      end
      else if (sec_tick) begin
        if (wait_secs > 32'd1) wait_secs <= wait_secs - 1'd1;
        else if (tries != 3'd0) begin            // retransmit
          tries <= tries - 1'd1;
          dhcp_tx_enable <= false;
          state <= ST_DHCP_REQUEST;
        end
        else if (!have_ip) begin                 // nobody answered at start: APIPA
          local_ip <= apipa_ip;
          have_ip <= true;
          is_apipa <= true;
          dhcp_enable <= false;
          state <= ST_RUNNING;
        end
        else if (renew_phase) begin              // renewal failed: try a full DISCOVER
          renew_phase <= false;
          force_discover <= true;
          tries <= 3'd4;
          dhcp_seconds_timer <= 4'd0;
          state <= ST_DHCP_REQUEST;
        end
        else begin                               // keep the old IP, try again in 60 s
          dhcp_enable <= false;
          wait_secs <= 32'd60;
          state <= ST_DHCP_BOUND;
        end
      end
    end

    // lease is valid, radio runs; renew at lease/2
    ST_DHCP_BOUND: begin
      dhcp_enable <= false;
      dhcp_tx_enable <= false;
      if (sec_tick) begin
        if (wait_secs > 32'd1) wait_secs <= wait_secs - 1'd1;
        else begin
          renew_phase <= true;
          tries <= 3'd3;
          dhcp_seconds_timer <= 4'd0;
          state <= ST_DHCP_REQUEST;
        end
      end
    end

    ST_DHCP_RESTART:
      if (!dhcp_failed_s) state <= ST_DHCP_REQUEST;

    // static or APIPA ip address
    ST_RUNNING: state <= ST_RUNNING;

    default: state <= ST_START;
  endcase
  end


//-----------------------------------------------------------------------------
//                           interconnections
//-----------------------------------------------------------------------------
localparam PT_ARP = 2'd0, PT_ICMP = 2'd1, PT_DHCP = 2'd2, PT_UDP = 2'd3;

reg tx_ready = false;
reg tx_start = false;
reg [1:0] tx_protocol = PT_UDP;

wire tx_is_icmp = tx_protocol == PT_ICMP;
wire tx_is_arp  = tx_protocol == PT_ARP;
wire tx_is_udp  = tx_protocol == PT_UDP;
wire tx_is_dhcp = tx_protocol == PT_DHCP;

//udp_send out
wire [7:0]  udp_data;
wire [15:0] udp_length;

//rgmii_recv out
wire rgmii_rx_active;
wire [7:0] rx_data;

//mac_recv
wire mac_rx_enable = rgmii_rx_active;
wire mac_rx_active;
wire rx_is_arp;
wire [47:0] remote_mac;

//ip_recv
wire ip_rx_enable = mac_rx_active && !rx_is_arp;
wire ip_rx_active;
wire rx_is_icmp;
wire [31:0] remote_ip;
wire [31:0] to_ip;

//udp_recv
wire udp_rx_enable = ip_rx_active && !rx_is_icmp;
wire dhcp_rx_active;
assign udp_rx_data = rx_data;

//arp
wire arp_rx_enable = mac_rx_active && rx_is_arp;
wire arp_tx_enable = tx_start && tx_is_arp;
wire arp_tx_request;
wire arp_tx_active;
wire [7:0] arp_tx_data;
wire [47:0] arp_destination_mac;

//icmp
wire icmp_rx_enable = ip_rx_active && rx_is_icmp;
wire icmp_tx_enable = tx_start && tx_is_icmp;
wire icmp_tx_request;
wire icmp_tx_active;
wire [7:0] icmp_data;
wire [15:0] icmp_length;
wire [47:0] icmp_destination_mac;
wire [31:0] icmp_destination_ip;

//udp (SDR) and dhcp share udp_send
assign udp_tx_enable      = tx_start && tx_is_udp;      // to sdr_send
wire   dhcp_udp_tx_enable = tx_start && tx_is_dhcp;     // to dhcp
wire   udp_send_enable    = udp_tx_enable | dhcp_udp_tx_enable;

wire [7:0]  udp_send_data   = tx_is_dhcp ? dhcp_tx_data   : udp_tx_data;
wire [15:0] udp_send_length = tx_is_dhcp ? dhcp_tx_length : udp_tx_length;
wire [15:0] local_port      = tx_is_dhcp ? 16'd68 : 16'd1024;
wire [7:0]  udp_port_ID     = tx_is_dhcp ? 8'd0   : port_ID;
wire [15:0] destination_port = tx_is_dhcp ? dhcp_destination_port : udp_destination_port_sync;

//ip_send in
wire        ip_tx_enable   = tx_is_icmp ? icmp_tx_active : udp_tx_active;
wire [7:0]  ip_tx_data_in  = tx_is_icmp ? icmp_data      : udp_data;
wire [15:0] ip_tx_length   = tx_is_icmp ? icmp_length    : udp_length;
wire [31:0] destination_ip = tx_is_icmp ? icmp_destination_ip :
                             tx_is_dhcp ? dhcp_destination_ip : udp_destination_ip_sync;
wire [7:0] ip_tx_data;
wire ip_tx_active;

//mac_send in
wire mac_tx_enable = arp_tx_active || ip_tx_active;
wire [7:0] mac_tx_data_in = tx_is_arp ? arp_tx_data : ip_tx_data;
wire [47:0] destination_mac = tx_is_arp  ? arp_destination_mac  :
                              tx_is_icmp ? icmp_destination_mac :
                              tx_is_dhcp ? dhcp_destination_mac : udp_destination_mac_sync;
wire [7:0] mac_tx_data;
wire mac_tx_active;

//rgmii_send
wire [7:0] rgmii_tx_data_in = mac_tx_data;
wire rgmii_tx_enable = mac_tx_active;
wire rgmii_tx_active;

//-----------------------------------------------------------------------------
//                              debug output
//-----------------------------------------------------------------------------
assign network_status = {phy_connected, phy_connected, phy_connected, udp_rx_active, udp_rx_enable, rgmii_rx_active, rgmii_tx_active, mac_rx_active};


sync sync_dtx(.clock(tx_clock), .sig_in(dhcp_tx_enable), .sig_out(dhcp_tx_enable_tx));

// tx arbiter: one frame at a time, priority ARP > ICMP > DHCP > SDR
// 2026: tx_start comes TX_SETUP cycles after tx_protocol is chosen, so the
// muxed length / IP / port have settled through the pipelined header logic
// of udp_send and ip_send (IP checksum) before the first byte is sent.
localparam TX_SETUP = 4'd8;
reg [3:0] tx_wait = 4'd0;
always @(posedge tx_clock)
  if (rgmii_tx_active || tx_reset) begin
    tx_ready <= false;
    tx_start <= false;
    tx_wait  <= 4'd0;
  end
  else if (tx_ready) begin
    if (tx_wait == TX_SETUP) tx_start <= true;
    else tx_wait <= tx_wait + 1'd1;
  end
  else begin
    tx_wait <= 4'd0;
    if (arp_tx_request) begin
      tx_protocol <= PT_ARP;
      tx_ready <= true;
    end
    else if (icmp_tx_request) begin
      tx_protocol <= PT_ICMP;
      tx_ready <= true;
    end
    else if (dhcp_tx_request) begin
      tx_protocol <= PT_DHCP;
      tx_ready <= true;
    end
    else if (udp_tx_request)  begin
      tx_protocol <= PT_UDP;
      tx_ready <= true;
    end
  end

//-----------------------------------------------------------------------------
//                  gratuitous ARP after the IP becomes valid
//-----------------------------------------------------------------------------
wire have_ip_tx;
sync sync_hip(.clock(tx_clock), .sig_in(have_ip), .sig_out(have_ip_tx));
reg        have_ip_tx_d = false;
reg [27:0] garp_tmr = 28'd0;
reg [1:0]  garp_cnt = 2'd0;
reg        garp_req = false;
always @(posedge tx_clock) begin
  garp_req <= false;
  have_ip_tx_d <= have_ip_tx;
  if (!have_ip_tx) begin garp_cnt <= 2'd0; garp_tmr <= 28'd0; end
  else if (!have_ip_tx_d) begin garp_cnt <= 2'd2; garp_tmr <= GARP_DLY1; end   // first after 0.2 s
  else if (garp_cnt != 2'd0) begin
    if (garp_tmr == 28'd0) begin
      garp_req <= true;
      garp_cnt <= garp_cnt - 1'd1;
      garp_tmr <= GARP_DLY2;          // second 2 s later
    end
    else garp_tmr <= garp_tmr - 1'd1;
  end
end

//-----------------------------------------------------------------------------
//                               receive
//-----------------------------------------------------------------------------

rgmii_recv rgmii_recv_inst (
  .active(rgmii_rx_active),
  .phy_connected(phy_connected),
  .reset(rx_reset),
  .rx_clock(rx_clock),
  .rx_data(rx_data),
  .PHY_RX(PHY_RX),
  .PHY_DV(PHY_DV),
  .PHY_RX_CLOCK(PHY_RX_CLOCK),
  .test1(test1),
  .test2(test2)
  );

mac_recv mac_recv_inst(
  .rx_enable(mac_rx_enable),
  .active(mac_rx_active),
  .is_arp(rx_is_arp),
  .remote_mac(remote_mac),
  .clock(rx_clock),
  .data(rx_data),
  .local_mac(local_mac),
  .broadcast(broadcast)
  );

ip_recv ip_recv_inst(
  .active(ip_rx_active),
  .is_icmp(rx_is_icmp),
  .remote_ip(remote_ip),
  .clock(rx_clock),
  .rx_enable(ip_rx_enable),
  .broadcast(broadcast),
  .data(rx_data),
  .to_ip(to_ip)
  );

udp_recv udp_recv_inst(
	.clock(rx_clock),
	.rx_enable(udp_rx_enable),
	.data(rx_data),
	.to_ip(to_ip),
   .local_ip(local_ip),
   .broadcast(broadcast),
	.remote_mac(remote_mac),
   .remote_ip(remote_ip),
	.active(udp_rx_active),
	.dhcp_active(dhcp_rx_active),
	.to_port(to_port),
	.udp_destination_ip(udp_destination_ip),
   .udp_destination_mac(udp_destination_mac),
	.udp_destination_port(udp_destination_port)
	);

//-----------------------------------------------------------------------------
//                           receive/reply
//-----------------------------------------------------------------------------
arp arp_inst(
  .rx_enable(arp_rx_enable),
  .tx_enable(arp_tx_enable),
  .garp_req(garp_req),
  .tx_active(arp_tx_active),
  .tx_data(arp_tx_data),
  .destination_mac(arp_destination_mac),
  .reset(tx_reset),
  .rx_clock(rx_clock),
  .rx_data(rx_data),
  .tx_clock(tx_clock),
  .local_mac(local_mac),
  .local_ip(local_ip),
  .tx_request(arp_tx_request),
  .remote_mac(remote_mac)
);

icmp icmp_inst (
  .rx_enable(icmp_rx_enable),
  .tx_enable(icmp_tx_enable),
  .tx_request(icmp_tx_request),
  .tx_active(icmp_tx_active),
  .tx_data(icmp_data),
  .destination_mac(icmp_destination_mac),
  .destination_ip(icmp_destination_ip),
  .length(icmp_length),
  .dst_unreachable(),
  .broadcast(broadcast),
  .to_ip(to_ip),
  .local_ip(local_ip),
  .remote_mac(remote_mac),
  .remote_ip(remote_ip),
  .reset(tx_reset),
  .rx_clock(rx_clock),
  .rx_data(rx_data),
  .tx_clock(tx_clock)
);


dhcp dhcp_inst(
  //rx in
  .rx_clock(rx_clock),
  .rx_data(rx_data),
  .rx_enable(dhcp_enable),
  .dhcp_rx_active(dhcp_rx_active),
  //rx out
  .lease(lease),
  .server_ip(server_ip),
  //tx in
  .reset(tx_reset),
  .tx_clock(tx_clock),
  .udp_tx_enable(dhcp_udp_tx_enable),
  .tx_enable(dhcp_tx_enable_tx),
  .udp_tx_active(udp_tx_active),
  .remote_mac(remote_mac),            // rx domain: sender of the ACK = server
  .remote_ip(remote_ip),
  .dhcp_seconds_timer(dhcp_seconds_timer),
  .force_discover(force_discover),
  // tx_out
  .dhcp_tx_request(dhcp_tx_request),
  .tx_data(dhcp_tx_data),
  .length(dhcp_tx_length),
  .ip_accept(ip_accept),
  //constants
  .local_mac(local_mac),
  .dhcp_destination_mac(dhcp_destination_mac),
  .dhcp_destination_ip(dhcp_destination_ip),
  .dhcp_destination_port(dhcp_destination_port),
  .unicast_renewal(dhcp_renewal),
  // result
  .dhcp_success(dhcp_success),
  .dhcp_failed(dhcp_failed)
  );


//-----------------------------------------------------------------------------
//                     rx to tx clock domain transfers (SDR peer)
//-----------------------------------------------------------------------------

// quasi-static buses: they change only when a new PC/port starts talking
cdc_sync #(32) cdc_sync_inst7 (.siga(udp_destination_ip),   .rstb(1'b0), .clkb(tx_clock), .sigb(udp_destination_ip_sync));
cdc_sync #(48) cdc_sync_inst8 (.siga(udp_destination_mac),  .rstb(1'b0), .clkb(tx_clock), .sigb(udp_destination_mac_sync));
cdc_sync #(16) cdc_sync_inst9 (.siga(udp_destination_port), .rstb(1'b0), .clkb(tx_clock), .sigb(udp_destination_port_sync));

//-----------------------------------------------------------------------------
//                               send
//-----------------------------------------------------------------------------

udp_send udp_send_inst (
  .reset(tx_reset),
  .clock(tx_clock),
  .tx_enable(udp_send_enable),
  .data_in(udp_send_data),
  .length_in(udp_send_length),
  .local_port(local_port),
  .destination_port(destination_port),
  .active(udp_tx_active),
  .data_out(udp_data),
  .length_out(udp_length),
  .port_ID(udp_port_ID)
  );

ip_send ip_send_inst (
  .data_in(ip_tx_data_in),
  .tx_enable(ip_tx_enable),
  .is_icmp(tx_is_icmp),
  .length(ip_tx_length),
  .destination_ip(destination_ip),
  .data_out(ip_tx_data),
  .active(ip_tx_active),
  .clock(tx_clock),
  .reset(tx_reset),
  // DHCP DISCOVER/REQUEST come from 0.0.0.0, only a renewal uses our IP
  .local_ip((tx_is_dhcp && !dhcp_renewal) ? 32'd0 : local_ip)
  );

mac_send mac_send_inst (
  .data_in(mac_tx_data_in),
  .tx_enable(mac_tx_enable),
  .destination_mac(destination_mac),
  .data_out(mac_tx_data),
  .active(mac_tx_active),
  .clock(tx_clock),
  .local_mac(local_mac),
  .reset(tx_reset)
  );

rgmii_send rgmii_send_inst (
  .data(rgmii_tx_data_in),
  .tx_enable(rgmii_tx_enable),
  .active(rgmii_tx_active),
  .tx_clock(tx_clock),
  .PHY_TX(PHY_TX),
  .PHY_TX_EN(PHY_TX_EN),
  .PHY_TX_CLOCK(PHY_TX_CLOCK),
  .PHY_CLK125(PHY_CLK125),
  .clock_2_5MHz(clock_2_5MHz),
  .pll_areset(pll_areset),
  .pll_locked(pll_locked)
  );

endmodule
