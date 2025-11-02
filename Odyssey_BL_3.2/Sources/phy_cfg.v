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

//  25 Sept 2014 - Modified initial register values to correct for 0.12nS rather than 0.2nS steps.
//                 Also added write to register 106h to turn off Tx Data Pad Skews.  Both these
//						 changes are due to errors in the original data sheet which was corrected Feb 2014.


//-----------------------------------------------------------------------------
// initialize the PHY device on startup
// by writing config data to its MDIO registers; 
// continuously read PHY status from the MDIO registers
//-----------------------------------------------------------------------------

module phy_cfg(
  //input
  input _run,
  input clock,        //2.5 MHZ
  
  //output
  output reg speed = 0,
  output reg duplex = 0,
  
  //hardware pins
  inout mdio_pin,
  output mdc_pin  
);


wire run;
	cdc_sync #(1)
	r_un (.siga(_run), .rstb(0), .clkb(clock), .sigb(run));

//-----------------------------------------------------------------------------

wire ready;
wire [15:0] rd_data;
reg rd_request;
  


always @(posedge clock)  
begin
   if (run & ready)
      begin
         speed <= rd_data[6] & !rd_data[5];
         duplex <= rd_data[3];
         rd_request <= 1'b1;
      end
		
   else //!ready
   begin
      rd_request <= 0;
   end
	
end

               
//-----------------------------------------------------------------------------
//                        MDIO interface to PHY
//-----------------------------------------------------------------------------


mdio mdio_inst (
  .clock(clock), 
  .addr(5'h1F), 
  .rd_request(rd_request),
  .ready(ready),
  .rd_data(rd_data),
  .mdio_pin(mdio_pin),
  .mdc_pin(mdc_pin)
  );  
  
  
endmodule
