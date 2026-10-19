//-----------------------------------------------------------------------------
//                    Copyright (c) 2012 HPSDR Team
//-----------------------------------------------------------------------------
// demultiplex phy nibbles (RGMII, 1000BASE-T)
//
// 2026: cleaned up (wires assigned in always blocks, double assign of
// rx_clock, use before declaration); behaviour unchanged.
//  ddio_in has invert_input_clocks = ON:
//    dataout_l = rising-edge nibble  = RXD[3:0], RX_DV
//    dataout_h = falling-edge nibble = RXD[7:4], RX_DV ^ RX_ER
//-----------------------------------------------------------------------------

module rgmii_recv (
  input reset,

  //receive: data and active are valid at posedge of clock
  output rx_clock,
  output reg [7:0] rx_data,
  output active,
  output phy_connected,

  //hardware pins
  input  [3:0]PHY_RX,
  input  PHY_DV,
  input  PHY_RX_CLOCK,

  output test1,
  output test2
  );

assign rx_clock = PHY_RX_CLOCK;

//-----------------------------------------------------------------------------
//          de-multiplex nibbles presented at both clock edges
//-----------------------------------------------------------------------------
wire       rxdv_wire, error;
wire [7:0] data_wire;
reg        rxdv_wire1 = 1'b0, error1 = 1'b0;
reg  [7:0] data_wire1 = 8'd0;
reg        rx_en = 1'b0, rx_er = 1'b0;

ddio_in ddio_in_inst (
  .datain   ({PHY_RX, PHY_DV}       ),
  .inclock  (rx_clock                   ),
  .dataout_l({data_wire[3:0], rxdv_wire}),
  .dataout_h({data_wire[7:4], error}    )
);

always @(posedge rx_clock)
begin
  data_wire1 <= data_wire;
  rxdv_wire1 <= rxdv_wire;
  error1     <= error;
  rx_data    <= data_wire1;
  rx_en      <= rxdv_wire1;
  rx_er      <= rxdv_wire1 ^ error1;
end

//-----------------------------------------------------------------------------
//                   in-band status (RX_DV = 0): link speed / duplex
//-----------------------------------------------------------------------------
reg speed = 1'b0;
reg duplex = 1'b0;

always @(posedge rx_clock)
   if (!rx_en && !rx_er)
	begin
      speed  <= (rx_data[0] == 1'b1 && rx_data[2:1] == 2'b10);   // link up, 125 MHz
      duplex <= (rx_data[3] == 1'b1);
   end

assign test1 = speed;
assign test2 = duplex;

reg [31:0] timer = 32'd25000000;
assign phy_connected = ~(timer >= 32'd25000000); // 0.2 sec hold

always @(posedge rx_clock)
begin
   if (speed && duplex)
	   timer <= 32'd0;
	else if (timer < 32'd25000000)
	   timer <= timer + 1'd1;
end

//-----------------------------------------------------------------------------
//                          preamble detector
//-----------------------------------------------------------------------------
localparam MIN_PREAMBLE_LENGTH = 3'd5;
reg [2:0] preamble_cnt = MIN_PREAMBLE_LENGTH;
reg payload_coming = 1'b0;

always @(posedge rx_clock)
  //RX-DV low, nothing is being received
  if (!rx_en) begin payload_coming <= 1'b0; preamble_cnt <= MIN_PREAMBLE_LENGTH; end
  //RX-DV high, but payload is not being received yet
  else if (!payload_coming)
    //count preamble bytes
    if (rx_data == 8'h55) begin if (preamble_cnt != 0) preamble_cnt <= preamble_cnt - 3'd1; end
    //enough preamble bytes plus SFD, payload follows
    else if ((preamble_cnt == 0) && (rx_data == 8'hD5)) payload_coming <= 1'b1;
    //wrong byte received, reset preamble byte count
    else preamble_cnt <= MIN_PREAMBLE_LENGTH;

assign active = rx_en & payload_coming;

endmodule
