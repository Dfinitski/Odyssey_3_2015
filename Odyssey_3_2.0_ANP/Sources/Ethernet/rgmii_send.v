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
//  Odyssey-II/III: David Fainitski N7DDC, 2017
//  2026: GTX_CLK forwarded through a DDIO output (90 deg), tx_pll reset/lock


module rgmii_send (
  input [7:0] data,
  input tx_enable,
  output active,
  output tx_clock,
  output clock_2_5MHz,
  input  pll_areset,             // hold tx_pll in reset until CLK125_NDO is stable
  output pll_locked,

  //hardware pins
  output [3:0]PHY_TX,
  output PHY_TX_EN,
  output PHY_TX_CLOCK,
  input  PHY_CLK125
  );

//-----------------------------------------------------------------------------
//                              clocks
//-----------------------------------------------------------------------------
// c0 = 125 MHz 0 deg   : TX data (ddio_out)
// c1 = 125 MHz 90 deg  : GTX_CLK forwarding (2.0 ns -> clock edge centred in the data eye)
// c2 = 2.5 MHz         : network state machine

wire clock_125_MHz_0_deg, clock_125_MHz_90_deg;

tx_pll	tx_pll_inst (
	.areset (pll_areset),
	.inclk0 (PHY_CLK125),
	.c0 (clock_125_MHz_0_deg),
	.c1 (clock_125_MHz_90_deg),
	.c2 (clock_2_5MHz),
	.locked (pll_locked)
	);

assign tx_clock = clock_125_MHz_0_deg;

// KSZ9031 adds no delay on GTX_CLK: the MAC has to centre the clock.
// Same IOE path as TXD/TX_CTL -> the skew is set by the PLL phase only.
ddio_out1 gtx_clk_ddio (
	.datain_h (1'b1),
	.datain_l (1'b0),
	.outclock (clock_125_MHz_90_deg),
	.dataout  (PHY_TX_CLOCK)
	);


//-----------------------------------------------------------------------------
//                            shift reg
//-----------------------------------------------------------------------------
localparam PREAMBLE_BYTES = 64'h55555555555555D5;
localparam PREAMB_LEN = 4'd8;
localparam HI_BIT = 8*PREAMB_LEN - 1;
reg [HI_BIT:0] shift_reg;
reg [3:0] bytes_left;


//-----------------------------------------------------------------------------
//                           state machine
//-----------------------------------------------------------------------------
localparam ST_IDLE = 3'd0, ST_SEND = 3'd1, ST_GAP = 3'd2;
reg [2:0] state = ST_IDLE;


wire sending = tx_enable | (state == ST_SEND);
assign active = sending | (state == ST_GAP);


always @(posedge tx_clock)
  begin
  if (sending) shift_reg <= {shift_reg[HI_BIT-8:0], data};
  else shift_reg <= PREAMBLE_BYTES;

  case (state)
    ST_IDLE:
      //receiving the first payload byte
      if (tx_enable) state <= ST_SEND;

    ST_SEND:
      //receiving payload data
      if (tx_enable) bytes_left <= PREAMB_LEN - 4'd1;
      //purging shift register
      else if (bytes_left != 0) bytes_left <= bytes_left - 4'd1;
      //starting inter-frame gap
      else begin bytes_left <= 4'd12; state <= ST_GAP; end

    ST_GAP:
      if (bytes_left != 0) bytes_left <= bytes_left - 4'd1;
      else state <= ST_IDLE;
    endcase
  end


//-----------------------------------------------------------------------------
//                             output
//-----------------------------------------------------------------------------
// two pipeline registers (same clock - not a clock-domain crossing), then
// the DDR registers in the IOE
reg [7:0] tx_data_p1 = 8'd0, tx_data = 8'd0;
reg       tx_en_p1 = 1'b0,   tx_en = 1'b0;
always @(posedge tx_clock) begin
  tx_data_p1 <= shift_reg[HI_BIT -: 8];  tx_data <= tx_data_p1;
  tx_en_p1   <= sending;                 tx_en   <= tx_en_p1;
end

ddio_out	ddio_out_inst (
	.datain_h({tx_en, tx_data[3:0]}),
	.datain_l({tx_en, tx_data[7:4]}),
	.outclock(tx_clock),
	.dataout({PHY_TX_EN, PHY_TX})
	);

endmodule
