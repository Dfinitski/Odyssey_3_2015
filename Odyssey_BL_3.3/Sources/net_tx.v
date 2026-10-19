//
//  Odyssey-2 Bootloader 3.3 - RGMII transmitter
//
//  Clock domain: tx_clk = PLL_125 c0 (from PHY CLK125_NDO), 125 MHz.
//
//  The frame to send is prepared by net_proc (PHY_RX_CLOCK domain) in
//  txb_hdr/txb_pay/txb_kind, which stay frozen until this module returns
//  done_t. start_t is a toggle (2-FF synchronised here); done_t is a toggle
//  synchronised back in net_proc.  Bytes 74..329 (RPD reply) come from the
//  dual-clock read-page RAM.
//
//  txb_kind : 0 = 60 bytes (ARP: 42 + 18 zero pad), 1 = 74 bytes, 2 = 330 bytes
//
//  Output: tx_d/tx_en registers -> ddio_out (IOE DDR registers). RGMII:
//  rising edge TXD[3:0] + TX_EN, falling edge TXD[7:4] + TX_EN^TX_ER (TX_ER=0).
//

module net_tx (
  input              clk,
  input  [42*8-1:0]  txb_hdr,     // quasi-static (rx domain), frozen while busy
  input  [32*8-1:0]  txb_pay,
  input  [1:0]       txb_kind,
  input              start_t,     // toggle from rx domain
  output reg         done_t = 1'b0,
  output             active,

  output     [7:0]   ram_raddr,   // read-page RAM (tx clock port)
  input      [7:0]   ram_q,

  output     [3:0]   PHY_TX,
  output             PHY_TX_EN
);

wire start_s;
cdc_sync #(1) st_sync (.siga(start_t), .rstb(1'b0), .clkb(clk), .sigb(start_s));
reg start_seen = 1'b0;

localparam T_IDLE = 3'd0, T_PRE = 3'd1, T_DATA = 3'd2, T_FCS = 3'd3, T_IFG = 3'd4;
reg [2:0]   st = T_IDLE;
reg [8:0]   cnt = 9'd0;
reg [8:0]   last = 9'd0;
reg [74*8-1:0] sr;
reg [31:0]  crc = 32'hFFFF_FFFF;
reg [7:0]   tx_d  = 8'd0;
reg         tx_en = 1'b0;

wire [7:0]  cur_byte = (cnt < 9'd74) ? sr[74*8-1 -: 8] : ram_q;
wire [31:0] crc_nxt;
crc32_d8 crc_i (.crc(crc), .d(cur_byte), .nxt(crc_nxt));

// RAM address one byte ahead (registered RAM read): during byte 73 we
// present address 0, data appears for byte 74.
wire [8:0] ra = cnt - 9'd73;
assign ram_raddr = ra[7:0];

assign active = (st != T_IDLE);

always @(posedge clk) begin
  case (st)
  T_IDLE: begin
      tx_en <= 1'b0;
      tx_d  <= 8'h00;
      cnt   <= 9'd0;
      if (start_s != start_seen) begin
        start_seen <= start_s;
        sr   <= {txb_hdr, txb_pay};
        last <= (txb_kind == 2'd0) ? 9'd59 : (txb_kind == 2'd1) ? 9'd73 : 9'd329;
        st   <= T_PRE;
      end
    end
  //
  T_PRE: begin                       // 7 x 0x55 + SFD 0xD5
      tx_en <= 1'b1;
      tx_d  <= (cnt == 9'd7) ? 8'hD5 : 8'h55;
      if (cnt == 9'd7) begin cnt <= 9'd0; crc <= 32'hFFFF_FFFF; st <= T_DATA; end
      else cnt <= cnt + 1'd1;
    end
  //
  T_DATA: begin
      tx_d <= cur_byte;
      crc  <= crc_nxt;
      if (cnt < 9'd74) sr <= {sr[73*8-1:0], 8'h00};
      if (cnt == last) begin cnt <= 9'd0; st <= T_FCS; end
      else cnt <= cnt + 1'd1;
    end
  //
  T_FCS: begin                       // ~CRC, LSB byte first
      case (cnt[1:0])
        2'd0: tx_d <= ~crc[7:0];
        2'd1: tx_d <= ~crc[15:8];
        2'd2: tx_d <= ~crc[23:16];
        2'd3: tx_d <= ~crc[31:24];
      endcase
      if (cnt == 9'd3) begin cnt <= 9'd0; st <= T_IFG; end
      else cnt <= cnt + 1'd1;
    end
  //
  T_IFG: begin                       // >= 12 byte times of idle
      tx_en <= 1'b0;
      tx_d  <= 8'h00;
      if (cnt == 9'd12) begin
        done_t <= ~done_t;
        st <= T_IDLE;
      end
      else cnt <= cnt + 1'd1;
    end
  //
  default: st <= T_IDLE;
  endcase
end

// DDR output in the IOE
ddio_out ddio_out_inst (
  .datain_h ({tx_en, tx_d[3:0]}),
  .datain_l ({tx_en, tx_d[7:4]}),
  .outclock (clk),
  .dataout  ({PHY_TX_EN, PHY_TX})
);

endmodule
