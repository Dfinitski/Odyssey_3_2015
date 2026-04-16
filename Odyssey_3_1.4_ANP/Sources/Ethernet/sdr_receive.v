//-----------------------------------------------------------------------------
//                          sdr receive
//-----------------------------------------------------------------------------

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


//  Metis code copyright 2010, 2011, 2012, 2013 Phil Harman VK6APH


module sdr_receive(
	// inputs
	input rx_clock,
	input [7:0] udp_rx_data,
	input udp_rx_active,
	input sending_sync,
	input broadcast,
	input discovery_ACK,					// set when sdr_send has seen discovery_reply request
	input [47:0] local_mac,
	input [15:0] to_port,

	// outputs
	output reg discovery_reply
);


reg [11:0] state;
reg [7:0]  byte_no;
reg [47:0] mac;

localparam 
	ST_IDLE			 = 12'd0,			// use 'one-hot' for state machine
	ST_COMMAND		 = 12'd1,
	ST_DISCOVERY	 = 12'd2,
	ST_TX				 = 12'd4,
	ST_WAIT			 = 12'd8;


// ****** NOTE: This state machine only runs when udp_rx_active ******	
	
always @(posedge rx_clock)
begin
  if (udp_rx_active && to_port == 1024) begin	// look for HPSDR udp packet to port 1024
    case (state)
	 
		ST_IDLE:	
			begin
			  byte_no <= 8'd0;
			  state <= ST_COMMAND;
			end 
			
		ST_COMMAND:
			begin
				if(byte_no==3 && udp_rx_data==8'd2) 	// Command
				   state <= ST_DISCOVERY;// allow Discovery to this address or broadcast 
            if(byte_no>3)
				   state <= ST_WAIT; 
            //
				byte_no <= byte_no + 8'd1;  // byte_no will be 4 when we leave this state
			end
				

		ST_DISCOVERY:  state <= ST_TX;   
																			
	
		// wait for the end of sending
		ST_TX:  if (!sending_sync) state <= ST_IDLE;
		
		ST_WAIT: if (!udp_rx_active) state <= ST_IDLE;						// command not for us so loop until it ends.

		default: if (!udp_rx_active) state <= ST_IDLE;
		
	  endcase	
	end
	  
	else state <= ST_IDLE;  // rx not active
	
end 

	
	
	
// wait for acknowledgement that sdr_send has seen the discovery reply request. 
// Needs separate state machine since udb Rx code only runs when udp_rx_active			
reg [2:0] DISC_state;	
reg [26:0]delay1;
always @ (posedge rx_clock)  
begin
	case (DISC_state)
	0: begin
			if (state == ST_DISCOVERY) begin
				discovery_reply <= 1'b1;
				delay1 <= 27'd125_000_000;
				DISC_state <= 	1;
			end 
		end 

	1: begin 								
			if (discovery_ACK | delay1 == 27'd0) begin  // time out ACK so we don't get stuck here. 
				discovery_reply <= 1'b0;
				DISC_state <= 0;
			end
			else delay1 <= delay1 - 1'd1;
		end 
	endcase
end	

endmodule
