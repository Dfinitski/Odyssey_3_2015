//
//  HPSDR - High Performance Software Defined Radio
//  Metis code copyright 2010, 2011, 2012, 2013 Alex Shovkoplyas, VE3NEA. (GPL v2+)
//
//  read/write the registers of a PHY device using its MDIO interface
//
//  Odyssey BL 3.3: PHY address is an input (auto-detected by phy_cfg),
//  MDIO is released (Z, external pull-up) while idle.
//
//  set phy_addr, addr, wr_data and rd_request/wr_request on posedge of clock,
//  keep them valid until ready goes low, then release the request and wait
//  for ready = 1.  MDC = clock (80 kHz here; KSZ9031 tP >= 120 ns).
//  MDIO is changed on the falling edge of MDC, the PHY samples on rising.
//  Read data: the PHY changes MDIO after rising MDC; it is sampled here on
//  the following falling edge (half a period later).
//

module mdio(
  input  clock,
  input  [4:0] phy_addr,
  input  [4:0] addr,
  input  rd_request,
  input  wr_request,
  output ready,

  input  [15:0] wr_data,
  output reg [15:0] rd_data = 16'd0,

  inout  mdio_pin,
  output mdc_pin
);

wire [63:0] wr_bits = {32'hFFFFFFFF, 4'b0101, phy_addr, addr, 2'b10, wr_data};
wire [63:0] rd_bits = {32'hFFFFFFFF, 4'b0110, phy_addr, addr, 2'b11, 16'hFFFF};
reg   [5:0] bit_no = 6'd63;

localparam ST_IDLE = 3'd1, ST_READING = 3'd2, ST_WRITING = 3'd3;
reg [2:0] state = ST_IDLE;

reg mdio_high_z = 1'b1;
assign mdio_pin = mdio_high_z ? 1'bz : (state == ST_READING ? rd_bits[bit_no] : wr_bits[bit_no]);

always @(negedge clock)
  case (state)
    ST_IDLE:
      begin
      bit_no <= 6'd63;
      if (rd_request)      begin state <= ST_READING; mdio_high_z <= 1'b0; end
      else if (wr_request) begin state <= ST_WRITING; mdio_high_z <= 1'b0; end
      else mdio_high_z <= 1'b1;
      end

    ST_READING:
      begin
      if (bit_no == 6'd18) mdio_high_z <= 1'b1;     // turnaround: release
      rd_data <= {rd_data[14:0], mdio_pin};
      if (bit_no == 6'd1) state <= ST_IDLE;
      bit_no <= bit_no - 6'd1;
      end

    ST_WRITING:
      begin
      if (bit_no == 6'd0) begin state <= ST_IDLE; mdio_high_z <= 1'b1; end
      bit_no <= bit_no - 6'd1;
      end

    default:
      state <= ST_IDLE;
  endcase

assign mdc_pin = clock;
assign ready = (state == ST_IDLE);

endmodule
