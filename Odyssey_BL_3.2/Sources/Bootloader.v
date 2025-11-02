//

// Bootloader  V2.0 for Odyssey-2 project
//
// N7DDC, David Fainitski, April 2018
//



/*
 
	EPCS64
	
	Bytes 				= 8M
	Sectors				= 128
	Bytes per sector 	= 65k
	Pages per sector 	= 256
	Number of pages  	= 32768
	Bytes per page   	= 256
	
	Address Range (Byte Addresses in HEX)
	
	Sector	Start    End
	
	127      H'7F0000 H'7FFFFF
   126      H'7E0000 H'7EFFFF
	..........................
	97       H'610000 H'61FFFF
   96       H'600000 H'60FFFF
	  //
	95       H'5F0000 H'5FFFFF
   94       H'5E0000 H'5EFFFF
	..........................
	65       H'420000 H'42FFFF
   64       H'410000 H'41FFFF
	  //
	63       H'400000 H'40FFFF
   62       H'3F0000 H'3FFFFF
	..........................
	33       H'210000 H'21FFFF
   32       H'200000 H'20FFFF  ****
	  //
	31 		H'1F0000 H'1FFFFF
	30 		H'1E0000 H'1EFFFF
	29 		H'1D0000 H'1DFFFF
	28 		H'1C0000 H'1CFFFF
	27 		H'1B0000 H'1BFFFF
	26 		H'1A0000 H'1AFFFF
	25 		H'190000 H'19FFFF
	24 		H'180000 H'18FFFF
	23 		H'170000 H'17FFFF
	22 		H'160000 H'16FFFF
	21 		H'150000 H'15FFFF
	20 		H'140000 H'14FFFF
	19 		H'130000 H'13FFFF
	18 		H'120000 H'12FFFF
	17 		H'110000 H'11FFFF
	16 		H'100000 H'10FFFF
	15 		H'0F0000 H'0FFFFF
	14 		H'0E0000 H'0EFFFF
	13 		H'0D0000 H'0DFFFF
	12 		H'0C0000 H'0CFFFF
	11 		H'0B0000 H'0BFFFF
	10 		H'0A0000 H'0AFFFF
	 9 		H'090000 H'09FFFF
	 8 		H'080000 H'08FFFF
	 7 		H'070000 H'07FFFF
	 6 		H'060000 H'06FFFF
	 5 		H'050000 H'05FFFF
	 4 		H'040000 H'04FFFF
	 3 		H'030000 H'03FFFF
	 2 		H'020000 H'02FFFF
	 1 		H'010000 H'01FFFF
	 0 		H'000000 H'00FFFF		
		
Each Sector holds 256 Pages each of 256 bytes
		



Command Format:
	

MAC address request, to device
‘MAC’ + SN + 28 binary zeroes

Reply, from device:
‘MAC’ + SN + 22 binary zeroes + 6 bytes of MAC address



Write IP command, to device
‘WIP’ + SN + 24 binary zeroes + 4 bytes of IP address 

Reply, from device:
The same, after finishing



Erase 32 sectors (one slot) starting this address of page, to device
‘ERS’ + SN + Address1 + Address0 + 26 binary zeroes

Reply, from device:
The same, after finishing.



Data 256 bytes of page for writing starting this address of page
‘WPD’ + SN + Address1 + Address0 + 26 binary zeroes + 256 bytes of data

Reply from device, after finishing:
The same, except 256 bytes of data. 32 bytes total.

	    	      
       

*/

module Bootloader (
// Clock
input INA_CLK,              //122.88MHz from ADC

// PHY
output  	wire     [3:0]PHY_TX,
input   	wire     [3:0]PHY_RX,		   
input		wire     PHY_RX_DV,				// PHY has data flag
output	wire     PHY_TX_CLOCK,			// PHY Tx data clock
output	wire 	 	PHY_TX_EN,				// PHY Tx enable
input		wire	 	PHY_RX_CLOCK,      	// PHY Rx data clock
input		wire     PHY_CLK_125,			// 125MHz clock from PHY PLL
inout		wire     PHY_MDIO,				// data line to PHY MDIO
output	wire 	 	PHY_MDC,					// 2.5MHz clock to PHY MDIO
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
output test1,
output test2  

);

assign NCONFIG = 1'd0;

assign BAND = 1'b0;
assign PTT_out_mcu = 1'b0;
assign LNA = 1'b0;

assign test2 = C_80kHz;
//assign test2 = C2_5MHz;
assign test1 = INA_CLK;
//assign test2 = clock_ready;

parameter [23:0] fw_version = "3.0";

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
   if (blink_cnt == 11) blink_cnt <= 1'd0;
end

//--------------------------------------------------------
// 					Clocks
//--------------------------------------------------------

wire C125_0_deg, C125_90_deg;
wire C12_5MHz, C2_5MHz;
C125_PLL PLL_125(.inclk0(PHY_RX_CLOCK), .c0(C125_0_deg), .c1(C125_90_deg), .c2(C12_5MHz), .c3(C2_5MHz));


wire tx_clock = C125_0_deg;
wire rx_clock = C125_90_deg;
assign PHY_TX_CLOCK = C125_90_deg;

 
wire C_80kHz;
wire clock_ready;
SLOW_PLL PLL_SLOW (.inclk0(INA_CLK), .c0(C_80kHz), .locked(clock_ready));

//------------------------------------------------------------
//  Reset and initialisation
//------------------------------------------------------------	
 
wire eth_reset = ~(speed & duplex);
assign PHY_RESET_N = bl_mode;

//---------------------------------------------------------------------------------
// Connection to MCU
//---------------------------------------------------------------------------------	
wire [23:0] start_addr;
wire [31:0]local_IP;
reg write_ip = 0;
reg [15:0]local_Port = 16'd50000;
wire [47:0]local_MAC; 	
wire addr_ready;
wire bl_mode;
wire reconfig_req;

//assign led4 = _reconfig_req & (dim_cnt <= dimmer);

				
pic_control #(fw_version) pic_cntrl (.run(clock_ready), .clock(C_80kHz), ._write_ip(write_ip), .ip_to_write(ip_to_write), .u_rx(u_rx), .u_tx(u_tx),
                                     .IP(local_IP), .MAC(local_MAC), .start_addr(start_addr), .addr_ready(addr_ready), .bl_mode(bl_mode), .bl_req(~KEY_DOT & ~KEY_DASH),
												 .reconfig_req(reconfig_req), ._reset_req(dev_reset), ._random_bit(random_bit)  );
	
	
//----------------------------------------------------------------------------------
// Read/Write the  PHY MDIO registers (NOTE: Max clock frequency is 2.5MHz)
//----------------------------------------------------------------------------------
wire speed, duplex ; 

phy_cfg phy_config(._run(bl_mode), .clock(C2_5MHz), .speed(speed), .duplex(duplex), .mdio_pin(PHY_MDIO), .mdc_pin(PHY_MDC)); 
						 
//--------------------------------------------------------------------------------
// FLASH memory operation
//--------------------------------------------------------------------------------
reg erase_req = 0, write_req = 0;
wire erase_done, wr_done;
reg erase_done_old = 0, wr_done_old = 0;
reg [2047:0] wr_data;  // 256 bytes for writing a page

flash flash_inst(.clock(C12_5MHz), .erase_req(erase_req), .write_req(write_req), .wr_data(wr_data), .page_addr(address), .erase_done(erase_done),
                   .wr_done(wr_done), .DCLK(DCLK), .DATAOUT(DATAOUT), .DATAIN(DATAIN), .FLASH_NCE(FLASH_NCE));

	
//------------------------------------------------------------------------------
//  Remote Update
//------------------------------------------------------------------------------
wire _reconfig_req;
	cdc_sync #(1)
	r_re (.siga(reconfig_req), .rstb(0), .clkb(C12_5MHz), .sigb(_reconfig_req));
wire _addr_ready;
   cdc_sync #(1)
	a_re (.siga(addr_ready), .rstb(0), .clkb(C12_5MHz), .sigb(_addr_ready));

Reconfigure Recon_inst(.reset(1'd0), .clock(C12_5MHz), .BootAddress(start_addr),
					   .control(_reconfig_req), .CRC_error(), .done(), .addr_ready(_addr_ready)); 
	

//---------------------------------------------------------------------------------
// Read from PHY
//---------------------------------------------------------------------------------

reg [7:0] rx_data;
reg rx_en;

always @(posedge rx_clock) 
begin rx_data[3:0] <= PHY_RX; end

always @(negedge rx_clock)
begin rx_data[7:4] <= PHY_RX; rx_en <= PHY_RX_DV; end

reg [4:0] rx_state = 0;
reg [42*8-1 : 0] rx_buffer;	// the buffer for Ethernet header, 42 bytes	
reg [32*8-1 : 0] data_buffer;  // the buffer for UDP or ICMP data, 32 bytes
reg [10:0] rx_byte_cnt = 0;	
reg udp_data = 0, udp_data_old = 0;
reg arp_data = 0, arp_data_old = 0;
reg icmp_data = 0, icmp_data_old = 0;
reg [31:0] icmp_checksum, header_checksum, ip_checksum;
reg [15:0] udp_length, ip_length;
reg network_rx_active = 0;
reg [23:0] rx_command;
reg [7:0] sn = 0;
reg [15:0] address;

`define      eth_dest        rx_buffer[(42-0)*8-1  -: 48]    // Eternet destination
`define      eth_source      rx_buffer[(42-6)*8-1  -: 48]    // Eternet source
`define      eth_type        rx_buffer[(42-12)*8-1 -: 16]    // Eternet type	

`define      arp_h_addr      rx_buffer[(42-14)*8-1 -: 16]    // ARP hardware address
`define      arp_p_addr      rx_buffer[(42-16)*8-1 -: 16]    // ARP protocol address
`define      arp_h_size      rx_buffer[(42-18)*8-1 -:  8]    // ARP hardware size 
`define      arp_p_size      rx_buffer[(42-19)*8-1 -:  8]    // ARP protocol size 
`define      arp_oper        rx_buffer[(42-20)*8-1 -: 16]    // ARP operation 
`define      arp_src_mac     rx_buffer[(42-22)*8-1 -: 48]    // ARP source MAC address
`define      arp_src_ip      rx_buffer[(42-28)*8-1 -: 32]    // ARP source IP address 
`define      arp_dest_mac    rx_buffer[(42-32)*8-1 -: 48]    // ARP destination MAC address 
`define      arp_dest_ip     rx_buffer[(42-38)*8-1 -: 32]    // ARP destination IP address 

`define      ip_version      rx_buffer[(42-14)*8-1 -:  8]    // IP version
`define      ip_type         rx_buffer[(42-15)*8-1 -:  8]    // IP type
`define      ip_size         rx_buffer[(42-16)*8-1 -: 16]    // IP header and data length
`define      ip_ident        rx_buffer[(42-18)*8-1 -: 16]    // IP identification
`define      ip_flags        rx_buffer[(42-20)*8-1 -: 16]    // IP flags
`define      ip_time         rx_buffer[(42-22)*8-1 -:  8]    // IP time to live
`define      ip_protocol     rx_buffer[(42-23)*8-1 -:  8]    // IP protocol
`define      ip_h_crc        rx_buffer[(42-24)*8-1 -: 16]    // IP header checksum
`define      ip_src          rx_buffer[(42-26)*8-1 -: 32]    // IP source address
`define      ip_dest         rx_buffer[(42-30)*8-1 -: 32]    // IP destination address

`define      udp_src         rx_buffer[(42-34)*8-1 -: 16]    // UDP source port
`define      udp_dest        rx_buffer[(42-36)*8-1 -: 16]    // UDP destination port
`define      udp_size        rx_buffer[(42-38)*8-1 -: 16]    // UDP data length
`define      udp_crc         rx_buffer[(42-40)*8-1 -: 16]    // UDP data checksum

`define      icmp_type       rx_buffer[(42-34)*8-1 -:  8]    // ICMP type
`define      icmp_code       rx_buffer[(42-35)*8-1 -:  8]    // ICMP code
`define      icmp_crc        rx_buffer[(42-36)*8-1 -: 16]    // ICMP checksum
`define      icmp_ident      rx_buffer[(42-38)*8-1 -: 16]    // ICMP identifier
`define      icmp_snum       rx_buffer[(42-40)*8-1 -: 16]    // ICMP sequence number

`define CMD       data_buffer[(32-0)*8-1 -: 24] // Command from PC
`define SN        data_buffer[(32-3)*8-1 -: 8]
`define ADDRESS   data_buffer[(32-4)*8-1 -: 16]  // Page sddress
						 
always @(posedge rx_clock)
begin
   case (rx_state)
	0: if(!eth_reset & rx_data == 8'h55 & rx_byte_cnt <= 6) rx_byte_cnt <= rx_byte_cnt + 1'd1; // detection of 7 bytes of preamble h55
	   else if(!eth_reset & rx_data == 8'hd5 & rx_byte_cnt == 7)                         // detection of frame delimiter, 8th byte
		begin
		   rx_byte_cnt <= 0;
			rx_state <= 1;
		end
		else rx_byte_cnt <= 0;                   // reset counter if preamble is wrong
	//	
	1:	if(rx_byte_cnt <= 41)  // receive 42 bytes of Ethernet header
	   begin
		   network_rx_active <= 1;
		   rx_buffer[(42-rx_byte_cnt)*8-1 -: 8] <= rx_data; 
		   rx_byte_cnt <= rx_byte_cnt + 1'd1;
		end
		else
		if((`eth_dest == 48'hff_ff_ff_ff_ff_ff | `eth_dest == local_MAC) & `eth_type == 16'h0806 & `arp_oper == 16'h0001) // ARP packet detection
		begin
			arp_data <= ~arp_data;	
			rx_state <= 2; 
		end
		else if(`eth_dest == local_MAC & `ip_dest == local_IP & `eth_type == 16'h0800 & `ip_version == 8'h45) // IP packet detection
		begin
			if(`ip_protocol == 8'h01 & `icmp_type == 8'h08)  // ICMP packet detection
			begin
				if(rx_byte_cnt <= 73) // ICMP data receiving
				begin
					data_buffer[(32-(rx_byte_cnt-42))*8-1 -: 8] <= rx_data;
					rx_byte_cnt <= rx_byte_cnt + 1'd1;
				end
				else 
				begin
					icmp_data <= ~icmp_data;
					rx_state <= 2;
				end					
		   end
			else if(`ip_protocol == 8'h11 & `udp_dest == local_Port) // UDP packet detection
			begin
			   if((`udp_size == 8+32) | (`udp_size == 8+32+256)) // Correct UDP length
				begin
				   if(rx_byte_cnt <= 73) // Short or command UDP data receiving
				   begin
					   data_buffer[(32-(rx_byte_cnt-42))*8-1 -: 8] <= rx_data;
					   rx_byte_cnt <= rx_byte_cnt + 1'd1;
				   end
				   else if(`udp_size == 8+32+256) // long UDP packet 
					begin
					   if(rx_byte_cnt <= 329)
						begin
						   wr_data[(256-(rx_byte_cnt-74))*8-1] <= rx_data[0]; // LSB is first here */
							wr_data[(256-(rx_byte_cnt-74))*8-2] <= rx_data[1];
							wr_data[(256-(rx_byte_cnt-74))*8-3] <= rx_data[2];
							wr_data[(256-(rx_byte_cnt-74))*8-4] <= rx_data[3];
							wr_data[(256-(rx_byte_cnt-74))*8-5] <= rx_data[4];
							wr_data[(256-(rx_byte_cnt-74))*8-6] <= rx_data[5];
							wr_data[(256-(rx_byte_cnt-74))*8-7] <= rx_data[6];
							wr_data[(256-(rx_byte_cnt-74))*8-8] <= rx_data[7];
					      rx_byte_cnt <= rx_byte_cnt + 1'd1;
						end
						else rx_state <= 2;
					end
					else rx_state <= 2;	
				end
				else rx_state <= 2;
			end
			else rx_state <= 2;
		end
		else rx_state <= 2;
		
		
	2: begin  // fixation of MAC, IP, Port for answear
			if(`ip_protocol == 8'h11 & `udp_dest == local_Port) // UDP packet detection
			begin
			   rx_command <= `CMD;
				sn <= `SN;
				if(`CMD=="ERS" | `CMD=="WPD") address <= `ADDRESS;
			   else if(`CMD=="MAC")
			   begin
			      dest_MAC <= `eth_source; // fixation destination MAC, IP and Port
		         dest_IP <= `ip_src;
		         dest_Port <= `udp_src;
			   end
			   udp_data <= ~udp_data; //  start UDP processing	
			end
		   rx_state <= 3;	
      end
		//
	3: if(!rx_en)  // waiting for end of frame and return to start
	   begin
		   network_rx_active <= 0;
	      rx_byte_cnt <= 0;
			rx_state <= 1'd0;
      end
	
	default: rx_state <= 0;
	endcase
end	


				 
						 
//--------------------------------------------------------------------------------------
//		Packets processing
//--------------------------------------------------------------------------------------				 

reg [47:0] dest_MAC;
reg [31:0] dest_IP;
reg [31:0] ip_to_write;
reg [15:0] dest_Port;
reg dev_reset = 0;


reg [32*8-1 : 0] udp_buffer;  // the buffer for UDP data, 32 bytes

reg [7:0] state = 0;

always @(posedge rx_clock)
begin
   case (state)
	0: if(udp_data != udp_data_old) state <= 1;
	   else if(arp_data != arp_data_old) state <= 80;
		else if(icmp_data != icmp_data_old) state <= 90;
		else if(erase_done != erase_done_old) state <= 2;
		else if(wr_done != wr_done_old) state <= 3;
	
	1: begin // received packet from PHY
	      udp_data_old <= udp_data;
	      if(rx_command == "MAC") // MAC request packet 
		   begin
			   udp_buffer[(32-0)*8-1 : 0] <= {"MAC", sn, 176'b0, local_MAC};
			   udp_length <= 8'd8 + 8'd32;
			   ip_length <= 8'd28 + 8'd32;
			   state <= 70; // fast response
		   end
		   else if(rx_command == "WIP") // IP writitg request packet
		   begin
		      ip_to_write <= data_buffer[(32-28)*8-1 -: 32];
			   write_ip <= ~write_ip;
			   udp_buffer <= {"WIP", sn, 192'b0, local_IP};
			   udp_length <= 8'd8 + 8'd32;
			   ip_length <= 8'd28 + 8'd32;
			   state <= 70; // fast response
		   end
			else if(rx_command == "ERS") // Slot erase request packet
		   begin
			   erase_req <= ~erase_req;
				state <= 1'd0;
		   end
			else if(rx_command == "WPD") // Data write request packet
		   begin
			   write_req <= ~write_req;
				state <= 1'd0;
		   end
			else if(rx_command == "RES")
			begin
			   dev_reset <= 1;
				state <= 1'd0;
			end
			else state <= 1'd0; // no matching
			//
		end	
	2: begin // answear after erasing
         erase_done_old <= erase_done;
			udp_buffer[(32-0)*8-1 : 0] <= {"ERS", sn, address, 208'b0};
         udp_length <= 8'd8 + 8'd32;
			ip_length <= 8'd28 + 8'd32;
			state <= 70; 
      end
   3: begin
	      wr_done_old <= wr_done;
			erase_done_old <= erase_done;
			udp_buffer[(32-0)*8-1 : 0] <= {"WPD", sn, address, 208'b0};
         udp_length <= 8'd8 + 8'd32;
			ip_length <= 8'd28 + 8'd32;
			state <= 70;
      end	
			
  70: if(!tx_buffer_use) // tx header buffer forming for UDP answer
			begin
			   tx_buffer <= {dest_MAC, local_MAC, 16'h0800/*`eth_type*/, 8'h45/*`ip_version*/, 8'h45/*`ip_type*/, ip_length, 16'h0807/*`ip_ident*/,
			           16'h0000/*`ip_flags*/, 8'd128/*`ip_time*/, 8'h11/*`ip_protocol*/,
		              /*`ip_h_crc*/ 16'h0000, local_IP, dest_IP, local_Port, dest_Port, udp_length, 16'h0000 /*`udp_crc*/};
            state <= state + 1'd1;
			end  
		//	
	
  71: begin // IP header checksum calculating
	      ip_checksum <= tx_buffer[(42-14)*8-1 -: 16] + tx_buffer[(42-16)*8-1 -: 16]	+ tx_buffer[(42-18)*8-1 -: 16] + tx_buffer[(42-20)*8-1 -: 16] +
				             tx_buffer[(42-22)*8-1 -: 16] + tx_buffer[(42-26)*8-1 -: 16] + tx_buffer[(42-28)*8-1 -: 16] + tx_buffer[(42-30)*8-1 -: 16] +
								 tx_buffer[(42-32)*8-1 -: 16];
	      state <= state + 1'd1;
	   end
		
  72: begin // writing IP header checksum and start the answer
         tx_buffer[(42-24)*8-1 -: 16] <= ~(ip_checksum[15:0] + ip_checksum[31:16]);
			send_is_udp <= ~send_is_udp;
			
			state <= 0; 
      end 
	   //
  80: if(!tx_buffer_use) // ARP answer forming
		begin
			tx_buffer <= {`arp_src_mac, local_MAC, `eth_type, `arp_h_addr, `arp_p_addr, `arp_h_size, `arp_p_size, /*`arp_oper*/ 16'h0002,
			                 local_MAC, local_IP, `eth_source,`arp_src_ip};
		   send_is_arp <= ~send_is_arp;
		   arp_data_old <= arp_data;	
			state <= 0; 
		end	
		//
  90:	if(!tx_buffer_use) // ICMP answer forming
		begin
		   tx_buffer <= {`eth_source, local_MAC, `eth_type, `ip_version, `ip_type, `ip_size, `ip_ident, `ip_flags, `ip_time, `ip_protocol,
		     	/*`ip_h_crc*/ 16'h0000, local_IP, `ip_src, /*`icmp_type*/ 8'h00, /*`icmp_code*/ 8'h00, /*`icmp_crc*/ 16'h0000, `icmp_ident, `icmp_snum};
			state <=  state + 1'd1;
		end	
			
  91:	begin // ICMP checksum calculating
		   icmp_checksum <= tx_buffer[(42-34)*8-1 -: 16] + tx_buffer[(42-38)*8-1 -: 16] + tx_buffer[(42-40)*8-1 -: 16] + data_buffer[(32-0)*8-1 -: 16] +
		                   data_buffer[(32-2)*8-1 -: 16] + data_buffer[(32-4)*8-1 -: 16] + data_buffer[(32-6)*8-1 -: 16] +
								 data_buffer[(32-8)*8-1 -: 16] + data_buffer[(32-10)*8-1 -: 16] + data_buffer[(32-12)*8-1 -: 16] + data_buffer[(32-14)*8-1 -: 16] +
								 data_buffer[(32-16)*8-1 -: 16] + data_buffer[(32-18)*8-1 -: 16] + data_buffer[(32-20)*8-1 -: 16] + data_buffer[(32-22)*8-1 -: 16] +
								 data_buffer[(32-24)*8-1 -: 16] + data_buffer[(32-26)*8-1 -: 16] + data_buffer[(32-28)*8-1 -: 16] + data_buffer[(32-30)*8-1 -: 16];
			ip_checksum <= tx_buffer[(42-14)*8-1 -: 16] + tx_buffer[(42-16)*8-1 -: 16]	+ tx_buffer[(42-18)*8-1 -: 16] + tx_buffer[(42-20)*8-1 -: 16] +
				             tx_buffer[(42-22)*8-1 -: 16] + tx_buffer[(42-26)*8-1 -: 16] + tx_buffer[(42-28)*8-1 -: 16] + tx_buffer[(42-30)*8-1 -: 16] +
								 tx_buffer[(42-32)*8-1 -: 16];
		   state <= state + 1'd1; 
		end
		
  92: begin	
		   tx_buffer[(42-36)*8-1 -: 16] <= ~(icmp_checksum[15:0] + icmp_checksum[31:16]);
			tx_buffer[(42-24)*8-1 -: 16] <= ~(ip_checksum[15:0] + ip_checksum[31:16]);
			send_is_icmp <= ~send_is_icmp;
			icmp_data_old <= icmp_data;
			state <= 0; 
		end	
	
	default: state <= 0;
	endcase

end
	

	
//--------------------------------------------------------------------------------------
// Write to PHY
//--------------------------------------------------------------------------------------

reg [4:0] tx_state = 0;
reg [42*8-1 : 0] tx_buffer;	// the buffer for Ethernet header, 42 bytes
reg tx_buffer_use = 0;	
reg [7:0] tx_byte_cnt = 0;	
reg send_is_arp = 0, send_is_arp_old = 0;	
reg send_is_icmp = 0, send_is_icmp_old = 0;
reg send_is_udp = 0, send_is_udp_old = 0;
reg tx_arp = 0, tx_icmp = 0, tx_udp = 0;
reg network_tx_active = 0;					 

reg tx_en = 0;
reg [7:0] tx_data;
reg [31:0] CRC32_reg;
						 
ddio_out	ddio_out_inst (
	.datain_h({tx_en, tx_data[3:0]}),
	.datain_l({tx_en, tx_data[7:4]}),   
	.outclock(tx_clock),
	.dataout({PHY_TX_EN, PHY_TX})
	);  
	
						 
always @(posedge tx_clock)
begin
   case (tx_state)
	0: if(send_is_arp != send_is_arp_old)
      begin
		   send_is_arp_old <= send_is_arp;
		   tx_arp = 1;
		   tx_state <= 1'd1;
		end
	   else if(send_is_icmp != send_is_icmp_old)
	   begin
		   send_is_icmp_old <= send_is_icmp;
			tx_icmp = 1;
	   	tx_state <= 1'd1;
		end
	   else if(send_is_udp != send_is_udp_old)
	   begin
	      send_is_udp_old <= send_is_udp;
		   tx_udp = 1;
		   tx_state <= 1'd1;
		end	
	
	1: if(tx_byte_cnt <= 6)    // Preamble sending
		begin
		   network_tx_active <= 1;
			tx_en <= 1;
			tx_data <= 8'h55;
			tx_byte_cnt <= tx_byte_cnt + 1'd1;
		end
		else 
		begin
			tx_data <= 8'hd5;
			tx_byte_cnt <= 1'd0;
			tx_buffer_use <= 1;
			tx_state <= 5'd2;
		end	
		//
	2: if(tx_byte_cnt <= 40)  // TX buffer 0-40 bytes sending
		begin
			reset_CRC <= 0;
		   tx_data <= tx_buffer[(42-tx_byte_cnt)*8-1 -:8];
			tx_byte_cnt <= tx_byte_cnt + 1'd1;
	   end
		else   // 41
		begin
		   tx_data <= tx_buffer[7 -:8];
			tx_buffer_use <= 0;
			tx_byte_cnt <= 1'd0;
		   if(tx_arp) tx_state <= 5'd3; 
			else if(tx_icmp | tx_udp) tx_state <= 5'd4; 
		end
		//
	3:	if(tx_byte_cnt <= 16) // ARP 0-16 bytes zero filling sending
		begin
			tx_data <= 8'h00;
			tx_byte_cnt <= tx_byte_cnt + 1'd1;
		end
	   else
	   begin
		   tx_data <= 8'h00; // 17
		   tx_byte_cnt <= 1'd0;
			tx_state <= 5'd5;
	   end
	   //	
	4: if(tx_byte_cnt <= 30) // ICMP or UDP 0-30 bytes data sending
		begin
			if(tx_icmp) tx_data <= data_buffer[(32-tx_byte_cnt)*8-1 -: 8];
			else if(tx_udp) tx_data <= udp_buffer[(32-tx_byte_cnt)*8-1 -: 8];
			tx_byte_cnt <= tx_byte_cnt + 1'd1;
		end
		else 
		begin // 31
		   if(tx_icmp) tx_data <= data_buffer[7 -: 8];
			else if(tx_udp) tx_data <= udp_buffer[7 -: 8];
			tx_byte_cnt <= 1'd0;
         tx_state <= 5'd5;
		end
		//
	5: if(tx_byte_cnt == 0) // CRC sending first byte
	   begin
         CRC32_reg <= CRC32;
			tx_data <= CRC32[7 -:8];
			tx_byte_cnt <= tx_byte_cnt + 1'd1;
      end	
		else if(tx_byte_cnt <= 3) // CRC sending last 3 bytes
	   begin
		   tx_data <= CRC32_reg [(tx_byte_cnt*8)+7 -: 8];
			tx_byte_cnt <= tx_byte_cnt + 1'd1;
		end
		else if(tx_byte_cnt <= 14) // 11 + 1 bytes inter-gap sending
		begin
		   tx_data <= 8'h55;
			tx_en <= 0;
			tx_byte_cnt <= tx_byte_cnt + 1'd1;
		end	
		else	
		begin  // end of frame 	
			tx_byte_cnt <= 1'd0;
			tx_udp <= 0;
			tx_icmp <= 0;
			tx_arp <= 0;
			reset_CRC <= 1;
			network_tx_active <= 0;
			tx_state <= 1'd0;
		end
	

	default: tx_state <= 1'd0;	
	endcase	
		
		
end						 
						 						 
						 
//----------------------------
//   802.3 CRC32 Calculation
//----------------------------
wire [31:0] CRC32;
reg reset_CRC = 1;

crc32 crc32_inst(.clock(tx_clock), .clear(reset_CRC),  .data(tx_data), .result(CRC32)); 


//-----------------------------------------------------------
//  		LED Control  
//-----------------------------------------------------------

wire STATUS_LED; 
wire DEBUG_LED1;             
wire DEBUG_LED2;
wire DEBUG_LED3;
wire DEBUG_LED4;
wire DEBUG_LED5;
wire DEBUG_LED6;
wire DEBUG_LED7;
wire DEBUG_LED8;
wire DEBUG_LED9;
wire DEBUG_LED10;

parameter half_second = 2500000; // at 12.5MHz clock rate
parameter dimmer = 3;  // LED bright 0 - 100 %

reg [7:0] dim_cnt = 0;
always @(posedge C12_5MHz)  if (dim_cnt != 100) dim_cnt <= dim_cnt + 1'd1; else dim_cnt <= 0;

assign led1 = STATUS_LED & (dim_cnt <= dimmer);  // Heart Beat
assign led2 = DEBUG_LED1 & bl_mode & (dim_cnt <= dimmer);  // connection's status
assign led3 = DEBUG_LED2 & (dim_cnt <= dimmer);  // receive from PHY
assign led4 = DEBUG_LED3 & (dim_cnt <= dimmer);  // transmitt to PHY


// flash LED1 for ~ 0.2 second whenever rgmii_rx_active
Led_flash Flash_LED1(.clock(C12_5MHz), .signal(network_rx_active), .LED(DEBUG_LED2), .period(half_second)); 

// flash LED2 for ~ 0.2 second whenever the PHY transmits
Led_flash Flash_LED2(.clock(C12_5MHz), .signal(network_tx_active), .LED(DEBUG_LED3), .period(half_second)); 



//Flash STATUS_LED
reg [26:0]HB_counter;
always @(posedge PHY_CLK_125) HB_counter = HB_counter + 1'b1;
assign STATUS_LED = HB_counter[25];  // Blink

//------------------------------------------------------------
//   Multi-state LED Control    
//------------------------------------------------------------

parameter clock_speed = 12_500_000; 	// 12.5MHz clock 

// display state of PHY negotiations  - fast flash if no Ethernet connection, slow flash if 100T and on if 1000T
Led_control #(clock_speed) Control_LED0(.clock(C12_5MHz), .on(speed & duplex), .fast_flash(!duplex),
										.slow_flash(!speed & duplex), .LED(DEBUG_LED1));



									
endmodule

