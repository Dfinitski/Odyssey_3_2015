//
//  Odyssey-2 Bootloader 3.3 - RGMII receiver / frame filter
//
//  Clock domain: PHY_RX_CLOCK (125 MHz, 1000BASE-T only).
//
//  * RGMII DDR capture in the IOE (ddio_in, inverted clock so that
//    dataout_l = rising-edge nibble = RXD[3:0]/RX_DV and
//    dataout_h = falling-edge nibble = RXD[7:4]/RX_DV^RX_ER).
//  * Preamble of any length (PHY may shorten it) up to SFD 0xD5.
//  * FCS check (CRC32 residue), RX_ER check, minimum length check.
//  * A frame is reported with a single-cycle 'acc' pulse only AFTER its
//    FCS has been verified - nothing is acted on before that.
//  * Page payload (bytes 74..329 of a 296-byte UDP datagram) is written into
//    the page RAM during reception only if 'page_wr_allow' was high at
//    byte 42 (flash idle). 'acc_page_ok' tells the processor that the RAM
//    really holds this frame's payload.
//
//  Frame byte numbering: byte 0 = first byte after SFD (destination MAC).
//  hdr  = bytes  0..41  (byte 0 in hdr[335:328])
//  cmd  = bytes 42..73  (byte 42 in cmd[255:248])
//

module net_rx #(
  parameter LINK_HOLD = 32'd125_000_000   // 1 s at 125 MHz
)(
  input             clk,
  input      [3:0]  rxd_pin,
  input             rxctl_pin,

  input      [47:0] local_mac,     // quasi-static (80 kHz domain)
  input      [31:0] local_ip,      // quasi-static (80 kHz domain)
  input      [15:0] local_port,
  input             page_wr_allow, // from net_proc (same clock)

  output reg        link_1000 = 1'b0,
  output reg        full_duplex = 1'b0,
  output            link_ok,
  output            active,

  output reg        acc = 1'b0,    // 1-clk pulse: good frame for us
  output reg        acc_arp = 1'b0,
  output reg        acc_icmp = 1'b0,
  output reg        acc_udp = 1'b0,
  output reg        acc_bcast = 1'b0, // UDP received via Ethernet broadcast (discovery)
  output reg        acc_long = 1'b0,  // UDP length 8+32+256
  output reg        acc_page_ok = 1'b0,
  output reg [42*8-1:0] hdr,
  output reg [32*8-1:0] cmd,

  output reg        pg_we = 1'b0,
  output reg [7:0]  pg_addr,
  output reg [7:0]  pg_data
);

//------------------------------------------------------------------
// DDR capture (IOE registers) + one fabric register stage
//------------------------------------------------------------------
wire [3:0] d_lo, d_hi;
wire       ctl_r, ctl_f;

ddio_in ddio_in_inst (
  .datain   ({rxd_pin, rxctl_pin}),
  .inclock  (clk),
  .dataout_l({d_lo, ctl_r}),
  .dataout_h({d_hi, ctl_f})
);

reg [7:0] rx_data = 8'd0;
reg       rx_dv   = 1'b0;
reg       rx_er   = 1'b0;

always @(posedge clk) begin
  rx_data <= {d_hi, d_lo};
  rx_dv   <= ctl_r;
  rx_er   <= ctl_r ^ ctl_f;
end

//------------------------------------------------------------------
// In-band link status (valid when RX_DV = 0 and RX_ER = 0)
//------------------------------------------------------------------
always @(posedge clk)
  if (!rx_dv && !rx_er) begin
    link_1000   <= rx_data[0] & (rx_data[2:1] == 2'b10);  // link up, 125 MHz
    full_duplex <= rx_data[3];
  end

reg [31:0] link_timer = LINK_HOLD;
always @(posedge clk)
  if (link_1000 && full_duplex)  link_timer <= 32'd0;
  else if (link_timer < LINK_HOLD) link_timer <= link_timer + 1'd1;

assign link_ok = (link_timer < LINK_HOLD);  // drops 1 s after gigabit FD lost

//------------------------------------------------------------------
// CRC
//------------------------------------------------------------------
reg  [31:0] crc = 32'hFFFF_FFFF;
wire [31:0] crc_nxt;
crc32_d8 crc_i (.crc(crc), .d(rx_data), .nxt(crc_nxt));

//------------------------------------------------------------------
// Field aliases (same layout as Bootloader 3.2)
//------------------------------------------------------------------
`define eth_dest     hdr[(42-0)*8-1  -: 48]
`define eth_type     hdr[(42-12)*8-1 -: 16]
`define arp_h_addr   hdr[(42-14)*8-1 -: 16]
`define arp_p_addr   hdr[(42-16)*8-1 -: 16]
`define arp_h_size   hdr[(42-18)*8-1 -:  8]
`define arp_p_size   hdr[(42-19)*8-1 -:  8]
`define arp_oper     hdr[(42-20)*8-1 -: 16]
`define arp_dest_ip  hdr[(42-38)*8-1 -: 32]
`define ip_version   hdr[(42-14)*8-1 -:  8]
`define ip_size      hdr[(42-16)*8-1 -: 16]
`define ip_flags     hdr[(42-20)*8-1 -: 16]
`define ip_protocol  hdr[(42-23)*8-1 -:  8]
`define ip_dest      hdr[(42-30)*8-1 -: 32]
`define udp_dest     hdr[(42-36)*8-1 -: 16]
`define udp_size     hdr[(42-38)*8-1 -: 16]
`define icmp_type    hdr[(42-34)*8-1 -:  8]
`define icmp_code    hdr[(42-35)*8-1 -:  8]

//------------------------------------------------------------------
// Receive state machine
//------------------------------------------------------------------
localparam S_IDLE = 2'd0, S_PRE = 2'd1, S_DATA = 2'd2, S_DROP = 2'd3;
reg [1:0]  st = S_IDLE;
reg [10:0] cnt = 11'd0;      // bytes after SFD, saturates
reg        err = 1'b0;

// classification, registered at byte 42 (all 42 header bytes are in hdr)
reg        f_arp = 1'b0, f_icmp = 1'b0, f_udp = 1'b0, f_bcast = 1'b0;
reg        f_long = 1'b0, f_page = 1'b0;
reg [10:0] f_need = 11'd2047;

wire is_bcast  = (`eth_dest == 48'hFFFF_FFFF_FFFF);
wire is_mine   = (`eth_dest == local_mac);
wire ip_ok     = (`eth_type == 16'h0800) && (`ip_version == 8'h45) &&
                 ((`ip_flags & 16'h3FFF) == 16'h0000);           // no fragments
wire ip_to_me  = ip_ok && (`ip_dest == local_ip);
wire udp_short = (`udp_size == 16'd40)  && (`ip_size == 16'd60);
wire udp_lng   = (`udp_size == 16'd296) && (`ip_size == 16'd316);
wire udp_port  = (`ip_protocol == 8'h11) && (`udp_dest == local_port);

assign active = (st == S_DATA);

always @(posedge clk) begin
  acc   <= 1'b0;
  pg_we <= 1'b0;

  case (st)
  //
  S_IDLE:
    if (link_ok && rx_dv && !rx_er) begin
      if (rx_data == 8'hD5) begin            // preamble fully stripped
        st <= S_DATA; cnt <= 11'd0; err <= 1'b0; crc <= 32'hFFFF_FFFF;
        f_arp <= 1'b0; f_icmp <= 1'b0; f_udp <= 1'b0; f_long <= 1'b0; f_page <= 1'b0;
      end
      else if (rx_data == 8'h55) st <= S_PRE;
      else                       st <= S_DROP;
    end
  //
  S_PRE:
    if (!rx_dv)                st <= S_IDLE;
    else if (rx_er)            st <= S_DROP;
    else if (rx_data == 8'hD5) begin
      st <= S_DATA; cnt <= 11'd0; err <= 1'b0; crc <= 32'hFFFF_FFFF;
      f_arp <= 1'b0; f_icmp <= 1'b0; f_udp <= 1'b0; f_long <= 1'b0; f_page <= 1'b0;
    end
    else if (rx_data != 8'h55) st <= S_DROP;
  //
  S_DATA:
    if (rx_dv) begin
      crc <= crc_nxt;
      if (rx_er) err <= 1'b1;
      if (cnt != 11'h7FF) cnt <= cnt + 1'd1;

      if (cnt < 11'd42)
        hdr <= {hdr[41*8-1:0], rx_data};

      if (cnt >= 11'd42 && cnt < 11'd74)
        cmd <= {cmd[31*8-1:0], rx_data};

      if (cnt == 11'd42) begin              // header complete -> classify
        f_arp   <= (is_bcast | is_mine) && (`eth_type == 16'h0806) &&
                   (`arp_h_addr == 16'h0001) && (`arp_p_addr == 16'h0800) &&
                   (`arp_h_size == 8'd6) && (`arp_p_size == 8'd4) &&
                   (`arp_oper == 16'h0001) && (`arp_dest_ip == local_ip);
        f_icmp  <= ip_to_me && (`ip_protocol == 8'h01) && (`icmp_type == 8'h08) &&
                   (`icmp_code == 8'h00) && (`ip_size == 16'd60);
        // Unicast to our IP (any dest MAC -> survives a stale ARP cache on the PC)
        // or Ethernet broadcast (discovery, only 'MAC' is honoured later).
        f_udp   <= ip_ok && udp_port && (udp_short | udp_lng) && (ip_to_me | is_bcast);
        f_bcast <= is_bcast && !ip_to_me;
        f_long  <= udp_lng;
        f_page  <= ip_to_me && udp_port && udp_lng && page_wr_allow;
        f_need  <= (`eth_type == 16'h0806) ? 11'd46 :     // 42 + FCS (pad not required)
                   udp_lng                  ? 11'd334 :    // 330 + FCS
                                              11'd78;      // 74 + FCS
      end

      if (f_page && cnt >= 11'd74 && cnt < 11'd330) begin
        pg_we   <= 1'b1;
        pg_addr <= cnt[7:0] - 8'd74;   // (cnt-74) mod 256
        pg_data <= rx_data;
      end
    end
    else begin                                // end of frame
      st <= S_IDLE;
      if ((crc == 32'hDEBB_20E3) && !err && (cnt >= f_need) && (cnt != 11'h7FF) &&
          (f_arp | f_icmp | f_udp)) begin
        acc         <= 1'b1;
        acc_arp     <= f_arp;
        acc_icmp    <= f_icmp;
        acc_udp     <= f_udp;
        acc_bcast   <= f_bcast;
        acc_long    <= f_long;
        acc_page_ok <= f_page;
      end
    end
  //
  S_DROP:
    if (!rx_dv) st <= S_IDLE;
  endcase
end

`undef eth_dest
`undef eth_type
`undef arp_h_addr
`undef arp_p_addr
`undef arp_h_size
`undef arp_p_size
`undef arp_oper
`undef arp_dest_ip
`undef ip_version
`undef ip_size
`undef ip_flags
`undef ip_protocol
`undef ip_dest
`undef udp_dest
`undef udp_size
`undef icmp_type
`undef icmp_code

endmodule
