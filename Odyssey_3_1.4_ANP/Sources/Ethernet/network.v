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


module network (
	output clock_12_5MHz,
	output clock_2_5MHz,

	//input
	input udp_tx_request,
	input [15:0] udp_tx_length, 
	input [7:0] udp_tx_data,
	input phy_connected,
	input [7:0] port_ID,
	input [31:0] static_ip,
	
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


  //status output
  output network_state,  
  output [7:0] network_status,
  output dhcp_timeout,
  output dhcp_success,
  output dhcp_failed,
 // output icmp_rx_enable,  

  //hardware pins
  output [3:0]PHY_TX,
  output PHY_TX_EN,            
  output PHY_TX_CLOCK,         
  input  [3:0]PHY_RX,     
  input  PHY_DV,                
  input  PHY_RX_CLOCK,         
  input  PHY_CLK125
  );


assign dhcp_timeout = (dhcp_timer == 0);

//-----------------------------------------------------------------------------
//                              debug output
//-----------------------------------------------------------------------------
assign network_status = {phy_connected, phy_connected, phy_connected, udp_rx_active, udp_rx_enable, rgmii_rx_active, rgmii_tx_active, mac_rx_active};


//-----------------------------------------------------------------------------
//                             state machine
//-----------------------------------------------------------------------------
//IP addresses
reg  [31:0] local_ip;
wire [31:0] apipa_ip = {8'd169, 8'd254, 8'd1, 8'd1};


localparam 
  ST_START         = 4'd0,  
  ST_DHCP_REQUEST  = 4'd1,
  ST_DHCP          = 4'd2,
  ST_DHCP_RENEW    = 4'd3,
  ST_DHCP_RETRY    = 4'd4,
  ST_RUNNING       = 4'd5;
  
  

reg [3:0] state = ST_START;
reg [23:0] dhcp_timer;
reg [37:0] dhcp_renew_timer;  // holds number of seconds before DHCP IP address must be renewed
reg [3:0] dhcp_seconds_timer;   // number of seconds since the DHCP request started



//reset all child modules
wire rx_reset, tx_reset;
sync sync_inst1(.clock(rx_clock), .sig_in(state == ST_START), .sig_out(rx_reset));  
sync sync_inst2(.clock(tx_clock), .sig_in(state == ST_START), .sig_out(tx_reset));   

assign network_state = (state==ST_RUNNING || state==ST_DHCP_RENEW) ? 1'd0 : 1'd1; // Let network code know we have a valid IP address so can run when needed.

always @(negedge clock_2_5MHz)
  //if connection lost, wait until reconnects
  if ((state > ST_START) && !phy_connected) 
    state <= ST_START;
  else
  //
  case (state)
    ST_START: 
	     if (phy_connected) 
		  begin
		     if(static_ip==32'h00_00_00_00)	      // needs to be 0.0.0.0 for DHCP
			  begin
			     dhcp_timer <= 24'd7_500_000;	   // set dhcp time out to 3 seconds  
				  dhcp_seconds_timer = 4'd4;        // 
              state <= ST_DHCP_REQUEST;
			  end
			  else
			  begin
		        local_ip <= static_ip;   // work with static IP
				  dhcp_enable <= 1'b0;     // disable dhcp receive
				  state <= ST_RUNNING;
			  end
		  end
		  
    // send initial dhcp discover and request on power up
    ST_DHCP_REQUEST: 
	 begin	
      dhcp_tx_enable <= 1'b1;           // set dhcp flag
      dhcp_enable <= 1'b1;              // enable dhcp receive
      state <= ST_DHCP;
    end
	 
    // wait for dhcp success, fail or time out.  Do time out here since same clock speed for 100/1000T
    // If DHCP provided IP address then set lease timeout to lease/2 seconds.
    ST_DHCP: 
	 begin
      dhcp_tx_enable <= 1'b0;         // clear dhcp flag
      if (dhcp_success) 
		begin
        local_ip <= ip_accept;   
        if (lease == 32'd0)
          dhcp_renew_timer <= 43_200 * 2_500_000;  // use 43,200 seconds (12 hours) if no lease time set
        else
          dhcp_renew_timer <= (lease >> 1) * 2_500_000;  // set timer to half lease time.
        state <= ST_DHCP_RENEW;
      end
      else if(dhcp_timer == 0) 
		begin  
        dhcp_renew_timer <= 18'h020000; // delay 50 ms
        dhcp_timer <= 22'd2_500_000;    // reset dhcp timer to one second    
        // Retransmit Discover 
        if (dhcp_seconds_timer!=0)
		  begin 
		     dhcp_seconds_timer <= dhcp_seconds_timer - 1'd1;
           state <= ST_DHCP_RETRY;     // retransmit the Discover request
		  end
        else     // no DHCP Offer received in 15 seconds; use apipa
		  begin
          local_ip <= apipa_ip;
          state <= ST_RUNNING;
        end
      end
      else
        dhcp_timer <= dhcp_timer - 1'd1;
    end
    
	 //
	 ST_DHCP_RETRY:
	 begin  // Initial DHCP IP address was not obtained.  Try again.
      dhcp_enable <= 1'b0;                // disable dhcp receive
      if (dhcp_renew_timer == 0)
        state <= ST_DHCP_REQUEST;
      else
        dhcp_renew_timer <= dhcp_renew_timer - 1'd1;
    end

	 //
    ST_DHCP_RENEW: 
	 begin  // 
      dhcp_enable <= 1'b0;                // disable dhcp receive
      if (dhcp_renew_timer == 0)
        state <= ST_DHCP_REQUEST;
      else
        dhcp_renew_timer <= dhcp_renew_timer - 1'd01;
    end

    // static or APIPA ip address obtained
    ST_RUNNING: 
	    state <= ST_RUNNING;
   
    endcase     


//-----------------------------------------------------------------------------
//                           interconnections
//-----------------------------------------------------------------------------
localparam PT_ARP = 2'd0, PT_ICMP = 2'd1, PT_DHCP = 2'd2, PT_UDP = 2'd3;
localparam false = 1'b0, true = 1'b1;



reg tx_ready = false;
reg tx_start = false;
reg [1:0] tx_protocol;

wire tx_is_icmp = tx_protocol == PT_ICMP;
wire tx_is_arp = tx_protocol  == PT_ARP;
wire tx_is_udp = tx_protocol  == PT_UDP;
wire tx_is_dhcp = tx_protocol == PT_DHCP;


//udp = dhcp or udp, they have separate data
wire [7:0]  udp_data;
wire [15:0] udp_length;
wire [15:0] destination_port;
wire [31:0] to_ip;


//rgmii_recv out
wire rgmii_rx_active;  
wire [7:0] rx_data;

//mac_recv in
wire mac_rx_enable = rgmii_rx_active;

wire rx_is_arp;

//ip_recv in
wire ip_rx_enable = mac_rx_active && !rx_is_arp;
//ip_recv out
wire ip_rx_active;
wire rx_is_icmp;

//udp_recv in
wire udp_rx_enable = ip_rx_active && !rx_is_icmp;
assign udp_tx_enable = tx_start && tx_is_udp;
//udp_recv out
assign udp_rx_data = rx_data;

//arp in
wire arp_rx_enable = mac_rx_active && rx_is_arp;
wire arp_tx_enable = tx_start && tx_is_arp;
//arp out
wire arp_tx_request;
wire arp_tx_active;
wire [7:0] arp_tx_data;
wire [47:0] arp_destination_mac;

//// icmp in
wire  icmp_rx_enable = ip_rx_active && rx_is_icmp;
wire icmp_tx_enable = tx_start && tx_is_icmp;

//icmp out
wire icmp_tx_request;
wire icmp_tx_active;
wire [7:0] icmp_data;
wire [15:0] icmp_length;
wire [47:0] icmp_destination_mac;
wire [31:0] icmp_destination_ip;

//ip_send in
wire ip_tx_enable = udp_tx_active;
wire [7:0] ip_tx_data_in = udp_data;
wire [15:0] ip_tx_length = udp_length;
wire [31:0] destination_ip = udp_destination_ip_sync;
//ip_send out
wire [7:0] ip_tx_data;
wire ip_tx_active;

//mac_send in
wire mac_tx_enable = arp_tx_active || ip_tx_active;
wire [7:0] mac_tx_data_in = tx_is_arp? arp_tx_data : ip_tx_data;
wire [47:0] destination_mac = tx_is_arp  ? arp_destination_mac  : udp_destination_mac_sync;

//mac_send out
wire [7:0] mac_tx_data;
wire mac_tx_active;

//rgmii_send in
wire [7:0] rgmii_tx_data_in = mac_tx_data;
wire rgmii_tx_enable = mac_tx_active;

//rgmii_send out
wire rgmii_tx_active;

//dhcp
wire [15:0]dhcp_udp_tx_length        =  udp_tx_length;
wire [7:0] dhcp_udp_tx_data          =  udp_tx_data;
wire [15:0]local_port				    =  tx_is_dhcp ? 16'd68 : 16'd1024;
 
wire [15:0]dhcp_udp_destination_port =  tx_is_dhcp ? dhcp_destination_port : udp_destination_port_sync; 
wire dhcp_rx_active;
wire mac_rx_active; 
  

always @(posedge tx_clock)
  if (rgmii_tx_active) begin
    tx_ready <= false;
    tx_start <= false;
  end
  else if (tx_ready)
    tx_start <= true;
  else begin
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
    end;
  end

//-----------------------------------------------------------------------------
//                               receive
//-----------------------------------------------------------------------------
rgmii_recv rgmii_recv_inst (
  //out
  .active(rgmii_rx_active),

  .reset(rx_reset), 
  .clock(rx_clock),  
  .data(rx_data),
  .PHY_RX(PHY_RX),     
  .PHY_DV(PHY_DV),
  .PHY_RX_CLOCK(PHY_RX_CLOCK),
  .PHY_TX_CLOCK(PHY_TX_CLOCK)
  );  

mac_recv mac_recv_inst(
  //in
  .rx_enable(mac_rx_enable),
  //out
  .active(mac_rx_active),
  .is_arp(rx_is_arp),
  .remote_mac(remote_mac), 
  .clock(rx_clock), 
  .data(rx_data),  
  .local_mac(local_mac),    
  .broadcast(broadcast)
  );  
  
  
ip_recv ip_recv_inst(
  //out
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
	//in
	.clock(rx_clock),
	.rx_enable(udp_rx_enable),
	.data(rx_data),
	.to_ip(to_ip),
   .local_ip(local_ip),
   .broadcast(broadcast),
	.remote_mac(remote_mac),
   .remote_ip(remote_ip),

	//out
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
  //in
  .rx_enable(arp_rx_enable), 
  .tx_enable(arp_tx_enable),   
  //out
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
  .remote_mac(remote_mac_sync)
);  

icmp icmp_inst (
  //in
  .rx_enable(icmp_rx_enable), 
  .tx_enable(icmp_tx_enable),  
  //out
  .tx_request(icmp_tx_request),
  .tx_active(icmp_tx_active),
  .tx_data(icmp_data),  
  .destination_mac(icmp_destination_mac),  
  .destination_ip(icmp_destination_ip),
  .length(icmp_length),
  .dst_unreachable(),

  .remote_mac(remote_mac_sync),
  .remote_ip(remote_ip_sync),
  .reset(tx_reset), 
  .rx_clock(rx_clock),  
  .rx_data(rx_data),
  .tx_clock(tx_clock)  
); 



wire dhcp_tx_request;
reg dhcp_enable;
reg dhcp_tx_enable;
wire [7:0]  dhcp_tx_data;
wire [15:0] dhcp_tx_length;
wire [47:0] dhcp_destination_mac;
wire [31:0] dhcp_destination_ip;
wire [15:0] dhcp_destination_port;
wire [31:0] lease;
wire [31:0] server_ip;					// IP address of the DHCP that provided the IP address 
wire [31:0] ip_accept;					// DHCP provided IP address
wire [47:0] remote_mac;
wire [31:0] remote_ip;
wire [15:0] remote_port;

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
  .udp_tx_enable(udp_tx_enable),
  .tx_enable(dhcp_tx_enable),
  .udp_tx_active(udp_tx_active), 
  .remote_mac(remote_mac_sync),				// MAC address of DHCP server
  .remote_ip(remote_ip_sync),				// IP address of DHCP server 
  .dhcp_seconds_timer(dhcp_seconds_timer),

  // tx_out
  .dhcp_tx_request(dhcp_tx_request), 
  .tx_data(dhcp_tx_data),
  .length(dhcp_tx_length),
  .ip_accept(ip_accept),				// IP address from DHCP server
  
  //constants
  .local_mac(local_mac),
  .dhcp_destination_mac(dhcp_destination_mac),
  .dhcp_destination_ip(dhcp_destination_ip),
  .dhcp_destination_port(dhcp_destination_port),  

  // result
  .dhcp_success(dhcp_success),
  .dhcp_failed(dhcp_failed)
  
  );

//-----------------------------------------------------------------------------
//                                rx to tx clock domain transfers
//-----------------------------------------------------------------------------
wire [47:0] remote_mac_sync;
wire [31:0] remote_ip_sync;
wire [15:0] udp_destination_port;
wire [15:0] udp_destination_port_sync;
wire [47:0] udp_destination_mac;
wire [47:0] udp_destination_mac_sync;
wire [31:0] udp_destination_ip;
wire [31:0] udp_destination_ip_sync;

cdc_sync #(48)cdc_sync_inst1 (.siga(remote_mac), .rstb(0), .clkb(tx_clock), .sigb(remote_mac_sync)); 
cdc_sync #(32)cdc_sync_inst2 (.siga(remote_ip), .rstb(0), .clkb(tx_clock), .sigb(remote_ip_sync)); 
cdc_sync #(32) cdc_sync_inst7 (.siga(udp_destination_ip), .rstb(0), .clkb(tx_clock), .sigb(udp_destination_ip_sync)); 
cdc_sync #(48) cdc_sync_inst8 (.siga(udp_destination_mac), .rstb(0), .clkb(tx_clock), .sigb(udp_destination_mac_sync)); 
cdc_sync #(16) cdc_sync_inst9 (.siga(udp_destination_port), .rstb(0), .clkb(tx_clock), .sigb(udp_destination_port_sync)); 

  
//-----------------------------------------------------------------------------
//                               send
//-----------------------------------------------------------------------------
udp_send udp_send_inst (
  //in
  .reset(tx_reset),
  .clock(tx_clock),
  .tx_enable(udp_tx_enable),
  .data_in(dhcp_udp_tx_data),
  .length_in(dhcp_udp_tx_length),
  .local_port(local_port),
  .destination_port(dhcp_udp_destination_port),
  //out
  .active(udp_tx_active),
  .data_out(udp_data),
  .length_out(udp_length),
  .port_ID(port_ID)
  );

  
ip_send ip_send_inst (
  //in
  .data_in(ip_tx_data_in),
  .tx_enable(ip_tx_enable),
  .is_icmp(0),
  .length(ip_tx_length),
  .destination_ip(destination_ip),
  //out
  .data_out(ip_tx_data),
  .active(ip_tx_active),

  .clock(tx_clock),
  .reset(tx_reset),
  .local_ip(local_ip)
  );  
  
mac_send mac_send_inst (
  //in
  .data_in(mac_tx_data_in),
  .tx_enable(mac_tx_enable),  
  .destination_mac(destination_mac),
  //out
  .data_out(mac_tx_data),
  .active(mac_tx_active),  

  .clock(tx_clock), 
  .local_mac(local_mac),
  .reset(tx_reset)
  );  
  
rgmii_send rgmii_send_inst (
  //in
  .data(rgmii_tx_data_in),  
  .tx_enable(rgmii_tx_enable),   
  .active(rgmii_tx_active),      
  .clock(tx_clock), 
  .PHY_TX(PHY_TX),
  .PHY_TX_EN(PHY_TX_EN),              
  .PHY_TX_CLOCK(PHY_TX_CLOCK),
  .PHY_RX_CLOCK(PHY_RX_CLOCK),  
  .PHY_CLK125(PHY_CLK125),                
  .clock_2_5MHz(clock_2_5MHz),
  .clock_12_5MHz(clock_12_5MHz)
  );  
  


endmodule
