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


//  Metis code copyright 2010, 2011, 2012, 2013 Alex Shovkoplyas, VE3NEA.


// 2021 Updated to support KSZ9021RN and KSZ9031RN on Hermes-Lite 2.0, KF7O.

//-----------------------------------------------------------------------------
// initialize the PHY device on startup
// by writing config data to its MDIO registers; 
// continuously read PHY status from the MDIO registers
//-----------------------------------------------------------------------------

module phy_cfg(
  //input
  input clock,       
  input conf_req,
  
  //hardware pins
  inout mdio_pin,
  output mdc_pin  
);

//mdio register values
reg [15:0] values [17:0];
//mdio register addresses 
reg [4:0] addresses [17:0];

reg [4:0] word_no;
reg wr_request = 0;

reg conf_req_old = 0;

//-----------------------------------------------------------------------------
//                            state machine
//-----------------------------------------------------------------------------

wire ready;

//state machine  
localparam RESET = 2'd0, INIT = 2'd1, WRITE = 2'd2, LOOP = 2'd3;  
reg [1:0] state = RESET;  

always @(posedge clock) 
begin
  //
  case (state)
  RESET: if(conf_req != conf_req_old)
  begin
     conf_req_old <= conf_req;
     state <= INIT;
  end
  //
  INIT: if(ready)  // reg initial
  begin
     values[17] <= 16'h0200; // Allow 1GB but don't advertise half duplex in 1000BASET
     values[16] <= 16'h0002;
     values[15] <= 16'h0004;
     values[14] <= 16'h4002;
     values[13] <= 16'h0077; // RGMII Control Signal Pad Skew   7-4 - RX_CTL   3-0 - TX_CTL, h7 - default, 0.06ns
     values[12] <= 16'h0002;
     values[11] <= 16'h0005;
     values[10] <= 16'h4002;
     values[9] <= 16'h7777; // RGMII RX Data Pad Skew 15-12 - RX3, 11-8 - RX2, 7-4 - RX1, 3-0 - RX0, h7 - default, 0.06ns
     values[8] <= 16'h0002;
     values[7] <= 16'h0006;
     values[6] <= 16'h4002;
     values[5] <= 16'h7777; // RGMII TX Data Pad Skew 15-12 - TX3, 11-8 - TX2, 7-4 - TX1, 3-0 - TX0, h7 - default, 0.06ns
     values[4] <= 16'h0002;
     values[3] <= 16'h0008;
     values[2] <= 16'h4002;
     values[1] <= {16'b0000_00_01111_01111}; // RGMII Clock Pad Skew TX 5bits RX 5bits, 15 - default, 0.06ns
	  values[0] <= 16'h1300; // Restart autonegotiation
	  addresses[17] <= 9;
     addresses[16] <= 5'h0d;
     addresses[15] <= 5'h0e;
     addresses[14] <= 5'h0d;
     addresses[13] <= 5'h0e;
     addresses[12] <= 5'h0d;
     addresses[11] <= 5'h0e;
     addresses[10] <= 5'h0d;
     addresses[9] <= 5'h0e;
     addresses[8] <= 5'h0d;
     addresses[7] <= 5'h0e;
     addresses[6] <= 5'h0d;
     addresses[5] <= 5'h0e;
     addresses[4] <= 5'h0d;
     addresses[3] <= 5'h0e;
     addresses[2] <= 5'h0d;
     addresses[1] <= 5'h0e;
	  addresses[0] <= 0;
     word_no <= 5'd17;
     wr_request <= 1'd1;
     state <= WRITE;
  end
   //
   WRITE: if(ready)
   begin
      if(word_no == 5'd0) 
         state <= LOOP;
      else 
         wr_request <= 1'b1;
      word_no <= word_no - 5'd1;
   end
   else
      wr_request <= 1'b0;
  //
  LOOP:
   state <= RESET;
  //
  endcase
end
     
//-----------------------------------------------------------------------------
//                        MDIO interface to PHY
//-----------------------------------------------------------------------------

mdio mdio_inst (
  .clock(clock), 
  .addr(addresses[word_no]), 
  .rd_request(),
  .wr_request(wr_request),
  .ready(ready),
  .rd_data(),
  .wr_data(values[word_no]),
  .mdio_pin(mdio_pin),
  .mdc_pin(mdc_pin)
  );  
  
endmodule
