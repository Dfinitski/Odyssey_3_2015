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


module ip_send (
  input reset,
  input clock,
  input tx_enable,
  output active,

  input [7:0] data_in,
  output [7:0] data_out,
  input is_icmp,
  input [15:0] length,
  input [31:0] local_ip,
  input [31:0] destination_ip
  );
  
  
  
//-----------------------------------------------------------------------------
//                              ip header
//-----------------------------------------------------------------------------
// 2026: header and checksum pipelined (4 register stages). The inputs are
// stable for TX_SETUP cycles before tx_enable (network.v), the shift register
// below reloads tx_bits every idle cycle, so the latency is invisible.
reg [7:0]  protocol_code = 8'd17;
reg [15:0] ip_packet_length = 16'd20;
reg [31:0] src_ip_r = 32'd0, dst_ip_r = 32'd0;
reg [16:0] s_src = 17'd0, s_dst = 17'd0;
reg [16:0] s_len = 17'd0;
reg [19:0] sum = 20'd0;
reg [16:0] checksum_and_carry = 17'd0;
reg [15:0] checksum = 16'd0;
always @(posedge clock) begin
  // stage 1
  protocol_code    <= is_icmp ? 8'd1 : 8'd17;
  ip_packet_length <= 16'd20 + length;
  src_ip_r         <= local_ip;
  dst_ip_r         <= destination_ip;
  // stage 2
  s_src <= {1'b0, src_ip_r[31:16]} + {1'b0, src_ip_r[15:0]};
  s_dst <= {1'b0, dst_ip_r[31:16]} + {1'b0, dst_ip_r[15:0]};
  s_len <= {1'b0, ip_packet_length} + {9'b0, protocol_code};
  // stage 3
  sum <= 20'h0C500 + {3'b0, s_len} + {3'b0, s_src} + {3'b0, s_dst};
  // stage 4, 5: one's complement folds
  checksum_and_carry <= {1'b0, sum[15:0]} + {13'b0, sum[19:16]};
  checksum <= ~(checksum_and_carry[15:0] + {15'b0, checksum_and_carry[16]});
end


//ip header
localparam HDR_LEN = 5'd22;
localparam HI_BIT = HDR_LEN * 8 - 1;
wire [HI_BIT:0] tx_bits = {16'h0800, 16'h4500, ip_packet_length, 40'h0000_0000_80, protocol_code, checksum, src_ip_r, dst_ip_r};
reg [4:0] byte_no = 5'd0;


//shift register
reg [HI_BIT:0] shift_reg;
assign data_out = shift_reg[HI_BIT -: 8];




//-----------------------------------------------------------------------------
//                            state machine
//-----------------------------------------------------------------------------
localparam false = 1'b0, true = 1'b1;
reg sending = false;

assign active = tx_enable | sending;

  
always @(posedge clock)  
  begin
  //tx data
  if (active) shift_reg <= {shift_reg[HI_BIT-8:0], data_in};
  else shift_reg <= tx_bits;
  
  //send while payload is coming
  if (tx_enable) begin byte_no <= HDR_LEN - 5'd1; sending <= true; end
  //purge shift register
  else if (byte_no != 0) byte_no <= byte_no - 5'd1;
  //done
  else sending <= false;
  end
  
  
endmodule
  