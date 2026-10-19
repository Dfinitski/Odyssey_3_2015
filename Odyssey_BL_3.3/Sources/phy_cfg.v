//
//  HPSDR - High Performance Software Defined Radio
//  Metis code copyright 2010-2013 Alex Shovkoplyas, VE3NEA. (GPL v2+)
//  2021 KSZ9021RN/KSZ9031RN support, KF7O.
//
//  Odyssey BL 3.3 rework (KSZ9031RNX):
//   * actually runs: starts DELAY clocks after the PHY leaves reset
//     (3.2 had conf_req tied to 0, so the PHY was never configured)
//   * PHY address auto-detected: scans 0..7 reading reg 2 (PHY ID1 = 0x0022),
//     falls back to DEF_ADDR
//   * writes the Microchip-recommended 16 ms FLP burst timing
//     (datasheet DS00002117F 7.0 - "compatible with all link partners")
//   * RGMII pad skews (MMD 2 regs 4,5,6,8) as parameters (defaults = no change)
//   * advertises 1000BASE-T full duplex only, restarts auto-negotiation
//

module phy_cfg #(
  parameter [4:0]  DEF_ADDR  = 5'd3,
  parameter [15:0] DELAY     = 16'd8000,     // 100 ms at 80 kHz (after RESET_N high)
  parameter [15:0] SKEW_CTL  = 16'h0077,     // MMD2 reg4: RX_CTL[7:4] TX_CTL[3:0], 7 = 0 ps
  parameter [15:0] SKEW_RXD  = 16'h7777,     // MMD2 reg5: RXD3..RXD0
  parameter [15:0] SKEW_TXD  = 16'h7777,     // MMD2 reg6: TXD3..TXD0
  parameter [15:0] SKEW_CLK  = 16'h01EF      // MMD2 reg8: GTX_CLK[9:5] RX_CLK[4:0], 15 = 0 ps
)(
  input        clock,       // 80 kHz
  input        run,         // PHY out of reset (level)
  inout        mdio_pin,
  output       mdc_pin,
  output reg   done = 1'b0,
  output reg   found = 1'b0,
  output reg [4:0] phy_addr = DEF_ADDR
);

localparam N_WR = 5'd26;

// {reg, value}
function [20:0] cfg;
  input [4:0] i;
  case (i)
  // FLP burst timing 16 ms (MMD 0 reg 4 = 0x0006, reg 3 = 0x1A80)
  5'd0 : cfg = {5'h0D, 16'h0000};  5'd1 : cfg = {5'h0E, 16'h0004};
  5'd2 : cfg = {5'h0D, 16'h4000};  5'd3 : cfg = {5'h0E, 16'h0006};
  5'd4 : cfg = {5'h0D, 16'h0000};  5'd5 : cfg = {5'h0E, 16'h0003};
  5'd6 : cfg = {5'h0D, 16'h4000};  5'd7 : cfg = {5'h0E, 16'h1A80};
  // RGMII pad skews, MMD 2
  5'd8 : cfg = {5'h0D, 16'h0002};  5'd9 : cfg = {5'h0E, 16'h0004};
  5'd10: cfg = {5'h0D, 16'h4002};  5'd11: cfg = {5'h0E, SKEW_CTL};
  5'd12: cfg = {5'h0D, 16'h0002};  5'd13: cfg = {5'h0E, 16'h0005};
  5'd14: cfg = {5'h0D, 16'h4002};  5'd15: cfg = {5'h0E, SKEW_RXD};
  5'd16: cfg = {5'h0D, 16'h0002};  5'd17: cfg = {5'h0E, 16'h0006};
  5'd18: cfg = {5'h0D, 16'h4002};  5'd19: cfg = {5'h0E, SKEW_TXD};
  5'd20: cfg = {5'h0D, 16'h0002};  5'd21: cfg = {5'h0E, 16'h0008};
  5'd22: cfg = {5'h0D, 16'h4002};  5'd23: cfg = {5'h0E, SKEW_CLK};
  // 1000BASE-T control: advertise full duplex only
  5'd24: cfg = {5'h09, 16'h0200};
  // basic control: AN enable + restart AN, FD, speed bits = 1000
  5'd25: cfg = {5'h00, 16'h1340};
  default: cfg = {5'h1F, 16'h0000};
  endcase
endfunction

reg  [4:0]  reg_a = 5'd0;
reg  [15:0] wdat  = 16'd0;
reg         rd_req = 1'b0, wr_req = 1'b0;
wire        ready;
wire [15:0] rdat;

mdio mdio_inst (
  .clock(clock), .phy_addr(phy_addr), .addr(reg_a),
  .rd_request(rd_req), .wr_request(wr_req), .ready(ready),
  .wr_data(wdat), .rd_data(rdat),
  .mdio_pin(mdio_pin), .mdc_pin(mdc_pin)
);

localparam C_WAITRUN = 3'd0, C_DELAY = 3'd1, C_SCAN = 3'd2, C_SCAN_W = 3'd3,
           C_SCAN_D = 3'd4, C_WR = 3'd5, C_WR_W = 3'd6, C_WR_D = 3'd7;
reg [2:0]  st = C_WAITRUN;
reg [15:0] dly = 16'd0;
reg [4:0]  idx = 5'd0;
reg [2:0]  scan = 3'd0;

always @(posedge clock) begin
  if (!run) begin
    st <= C_WAITRUN; done <= 1'b0; dly <= 16'd0; rd_req <= 1'b0; wr_req <= 1'b0;
  end
  else case (st)
  C_WAITRUN: st <= C_DELAY;
  C_DELAY:   if (dly == DELAY) begin st <= C_SCAN; scan <= 3'd0; end
             else dly <= dly + 1'd1;
  // --- PHY address scan: read reg 2 ---
  C_SCAN:    if (ready) begin phy_addr <= {2'b00, scan}; reg_a <= 5'd2; rd_req <= 1'b1; st <= C_SCAN_W; end
  C_SCAN_W:  if (!ready) begin rd_req <= 1'b0; st <= C_SCAN_D; end
  C_SCAN_D:  if (ready) begin
               if (rdat == 16'h0022) begin found <= 1'b1; idx <= 5'd0; st <= C_WR; end
               else if (scan == 3'd7) begin phy_addr <= DEF_ADDR; idx <= 5'd0; st <= C_WR; end
               else begin scan <= scan + 1'd1; st <= C_SCAN; end
             end
  // --- register writes ---
  C_WR:      if (ready) begin {reg_a, wdat} <= cfg(idx); wr_req <= 1'b1; st <= C_WR_W; end
  C_WR_W:    if (!ready) begin wr_req <= 1'b0; st <= C_WR_D; end
  C_WR_D:    if (ready) begin
               if (idx == N_WR - 1'd1) done <= 1'b1;   // stay here
               else begin idx <= idx + 1'd1; st <= C_WR; end
             end
  endcase
end

endmodule
