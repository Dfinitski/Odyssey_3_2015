//
//  Odyssey-2 Bootloader 3.3 - packet processor / protocol engine
//
//  Clock domain: PHY_RX_CLOCK.
//
//  Protocol (UDP port 50000, payload 32 bytes, + 256 data bytes for WPD/RPD):
//
//   'MAC' SN 28x0                  -> 'MAC' SN ver[3] caps 14x0 IP[4] MAC[6]
//        (also accepted as Ethernet broadcast = discovery)
//   'WIP' SN 24x0 IP[4]            -> 'WIP' SN 24x0 newIP[4]
//   'ERS' SN A1 A0 26x0            -> same, after the 32 sectors are erased
//   'WPD' SN A1 A0 26x0 data[256]  -> 'WPD' SN A1 A0 26x0, after programming
//   'RPD' SN A1 A0 26x0            -> 'RPD' SN A1 A0 26x0 data[256]   (new in 3.3)
//   'RES' 29x0                     -> no reply, asks the MCU to reset
//
//  Rules (all new in 3.3, backward compatible with BootLoader 2.3 PC tool):
//   * Flash commands are ignored while the flash is busy (a retransmitted
//     request can never re-trigger or corrupt a running operation).
//   * ERS/WPD identical (cmd,SN,address) to the LAST COMPLETED one is answered
//     again without touching the flash (lost reply -> safe retransmission).
//     'MAC' clears that memory.
//   * ERS only on a slot boundary (page address multiple of 8192, slot 0..3).
//
//  caps byte in the MAC reply: bit0 RPD, bit1 safe retransmission,
//  bit2 broadcast discovery, bit3 FCS checked, bit4 broadcast WIP.
//
//  Broadcast WIP (3.3): 'WIP' SN 18x0 MAC[6] IP[4] sent to the Ethernet
//  broadcast is accepted only if MAC (bytes 22..27) is ours - changes the IP
//  of one board from any subnet. Unicast WIP ignores bytes 22..27.
//  New IP 0.0.0.0 and 255.255.255.255 are rejected.
//

module net_proc #(
  parameter [23:0] FW_VERSION = "3.3",
  parameter [7:0]  CAPS       = 8'h1F,
  parameter [31:0] GARP_DLY1  = 32'd62_500_000,   // 0.5 s after link/IP ready
  parameter [31:0] GARP_DLY2  = 32'd250_000_000   // then 2 s later once more
)(
  input             clk,
  input      [47:0] local_mac,
  input      [31:0] local_ip,
  input      [15:0] local_port,
  input             ip_ready,       // IP/MAC valid (synchronised addr_ready & bl_mode)
  input             link_ok,

  // from net_rx
  input             acc,
  input             acc_arp, acc_icmp, acc_udp, acc_bcast, acc_long, acc_page_ok,
  input  [42*8-1:0] hdr,
  input  [32*8-1:0] cmd,
  output            page_wr_allow,

  // to net_tx
  output reg [42*8-1:0] txb_hdr,
  output reg [32*8-1:0] txb_pay,
  output reg [1:0]  txb_kind = 2'd1,
  output reg        tx_start_t = 1'b0,
  input             tx_done_t,      // toggle from tx domain (raw)

  // to flash (12.5 MHz domain)
  output reg        fl_req_t = 1'b0,
  output reg [1:0]  fl_op = 2'd0,   // 1 erase slot, 2 write page, 3 read page
  output reg [15:0] fl_page = 16'd0,
  input             fl_done_t,      // toggle from flash domain (raw)

  // to MCU interface (80 kHz domain)
  output reg        write_ip_t = 1'b0,
  output reg [31:0] ip_to_write = 32'd0,
  output reg        dev_reset_t = 1'b0,

  output            busy_flash,
  output            busy_tx
);

//------------------------------------------------------------------
// Synchronisers for the returning toggles
//------------------------------------------------------------------
wire tx_done_s, fl_done_s;
cdc_sync #(1) txd_sync (.siga(tx_done_t), .rstb(1'b0), .clkb(clk), .sigb(tx_done_s));
cdc_sync #(1) fld_sync (.siga(fl_done_t), .rstb(1'b0), .clkb(clk), .sigb(fl_done_s));
reg tx_done_seen = 1'b0, fl_done_seen = 1'b0;

reg tx_busy = 1'b0;
reg fl_busy = 1'b0;
reg fl_evt  = 1'b0;
assign busy_flash = fl_busy;
assign busy_tx    = tx_busy;

// tx_busy is NOT needed here: net_tx reads only the read-page RAM
assign page_wr_allow = !fl_busy;

//------------------------------------------------------------------
// Field aliases
//------------------------------------------------------------------
`define eth_source   hdr[(42-6)*8-1  -: 48]
`define arp_src_mac  hdr[(42-22)*8-1 -: 48]
`define arp_src_ip   hdr[(42-28)*8-1 -: 32]
`define ip_tos       hdr[(42-15)*8-1 -:  8]
`define ip_size      hdr[(42-16)*8-1 -: 16]
`define ip_ident     hdr[(42-18)*8-1 -: 16]
`define ip_flags     hdr[(42-20)*8-1 -: 16]
`define ip_src       hdr[(42-26)*8-1 -: 32]
`define udp_src      hdr[(42-34)*8-1 -: 16]
`define icmp_ident   hdr[(42-38)*8-1 -: 16]
`define icmp_snum    hdr[(42-40)*8-1 -: 16]
`define CMD          cmd[(32-0)*8-1 -: 24]
`define SN           cmd[(32-3)*8-1 -:  8]
`define ADDRESS      cmd[(32-4)*8-1 -: 16]
`define NEW_IP       cmd[(32-28)*8-1 -: 32]
`define TGT_MAC      cmd[(32-22)*8-1 -: 48]

// tx buffer word helpers
`define TW(n)  txb_hdr[(42-(n))*8-1 -: 16]
`define PW(n)  txb_pay[(32-(n))*8-1 -: 16]

//------------------------------------------------------------------
// Latched request
//------------------------------------------------------------------
reg [47:0] peer_mac  = 48'd0;
reg [31:0] peer_ip   = 32'd0;
reg [15:0] peer_port = 16'd0;
reg [23:0] r_cmd = 24'd0;
reg [7:0]  r_sn  = 8'd0;
reg [15:0] r_addr = 16'd0;

// current flash job and last completed one
reg [23:0] cur_cmd = 24'd0;  reg [7:0] cur_sn = 8'd0;  reg [15:0] cur_addr = 16'd0;
reg [47:0] cur_mac = 48'd0;  reg [31:0] cur_ip = 32'd0; reg [15:0] cur_port = 16'd0;
reg        last_valid = 1'b0;
reg [23:0] last_cmd = 24'd0; reg [7:0] last_sn = 8'd0; reg [15:0] last_addr = 16'd0;

wire [15:0] cmd_addr = `ADDRESS;
wire dup = last_valid && (`CMD == last_cmd) && (`SN == last_sn) && (`ADDRESS == last_addr);

// Command decode registered one clock early (timing at 125 MHz): 'cmd' is
// complete at byte 73 and stays unchanged during the 4 FCS bytes, so these
// flags are valid when 'acc' arrives.
reg c_mac = 1'b0, c_wip = 1'b0, c_res = 1'b0, c_ers = 1'b0, c_wpd = 1'b0, c_rpd = 1'b0;
reg c_slot_ok = 1'b0, c_page_hi = 1'b0, c_dup = 1'b0, c_tgt_me = 1'b0, c_ip_ok = 1'b0;
always @(posedge clk) begin
  c_mac <= (`CMD == "MAC");  c_wip <= (`CMD == "WIP");  c_res <= (`CMD == "RES");
  c_ers <= (`CMD == "ERS");  c_wpd <= (`CMD == "WPD");  c_rpd <= (`CMD == "RPD");
  c_slot_ok <= (cmd_addr[12:0] == 13'd0) && (cmd_addr[15:13] <= 3'd3);
  c_page_hi <= cmd_addr[15];
  c_dup <= dup;   // a stale value can only make a duplicate re-execute (harmless)
  c_tgt_me <= (`TGT_MAC == local_mac);
  c_ip_ok  <= (`NEW_IP != 32'h0000_0000) && (`NEW_IP != 32'hFFFF_FFFF);
end

reg [15:0] ip_id = 16'h0001;

//------------------------------------------------------------------
// Gratuitous ARP (announce our MAC after link-up: the bootloader MAC is
// random at every start, so the PC's ARP cache is usually stale)
//------------------------------------------------------------------
reg        garp_clr = 1'b0;
reg [31:0] garp_tmr = 32'd0;
reg [1:0]  garp_cnt = 2'd0;
reg        garp_req = 1'b0;
always @(posedge clk) begin
  if (!link_ok || !ip_ready) begin
    garp_tmr <= 32'd0; garp_cnt <= 2'd0; garp_req <= 1'b0;
  end
  else if (garp_cnt != 2'd2 && !garp_req) begin
    if (garp_tmr == ((garp_cnt == 2'd0) ? GARP_DLY1 : GARP_DLY2)) begin
      garp_req <= 1'b1; garp_tmr <= 32'd0;
    end
    else garp_tmr <= garp_tmr + 1'd1;
  end
  if (garp_clr) begin garp_req <= 1'b0; garp_cnt <= garp_cnt + 1'd1; end
end

//------------------------------------------------------------------
// Main FSM
//------------------------------------------------------------------
localparam P_IDLE = 3'd0, P_WAIT = 3'd1, P_CS1 = 3'd2, P_CS2 = 3'd3, P_CS3 = 3'd4, P_SEND = 3'd5, P_CS4 = 3'd6, P_CS0 = 3'd7;
reg [2:0]  st = P_IDLE;
reg        is_icmp = 1'b0;
reg [20:0] ip_sum = 21'd0, ic_sum_a = 21'd0, ic_sum_b = 21'd0;
reg [18:0] ip_p0 = 19'd0, ip_p1 = 19'd0, ic_p0 = 19'd0, ic_p1 = 19'd0, ic_p2 = 19'd0, ic_p3 = 19'd0;
reg [21:0] ic_sum = 22'd0;
reg [16:0] ip_f1 = 17'd0, ic_f1 = 17'd0;

// reply payload builder
reg [23:0] rp_cmd; reg [7:0] rp_sn; reg [15:0] rp_addr;
reg [47:0] rp_mac; reg [31:0] rp_ip; reg [15:0] rp_port;

always @(posedge clk) begin
  garp_clr <= 1'b0;

  // returning toggles
  if (tx_done_s != tx_done_seen) begin tx_done_seen <= tx_done_s; tx_busy <= 1'b0; end
  if (fl_done_s != fl_done_seen) begin fl_done_seen <= fl_done_s; fl_evt  <= 1'b1; end

  case (st)
  //----------------------------------------------------------------
  P_IDLE:
    if (acc && !ip_ready) ;        // IP/MAC not valid yet (or PLL not locked): ignore
    else if (acc && acc_arp) begin
      if (!tx_busy) begin
        txb_hdr <= {`eth_source, local_mac, 16'h0806, 16'h0001, 16'h0800, 8'd6, 8'd4, 16'h0002,
                    local_mac, local_ip, `arp_src_mac, `arp_src_ip};
        txb_pay  <= 256'd0;
        txb_kind <= 2'd0;
        st <= P_SEND;
      end
    end
    else if (acc && acc_icmp) begin
      if (!tx_busy) begin
        txb_hdr <= {`eth_source, local_mac, 16'h0800, 8'h45, `ip_tos, `ip_size, `ip_ident,
                    (`ip_flags & 16'h4000), 8'd128, 8'h01, 16'h0000, local_ip, `ip_src,
                    8'h00, 8'h00, 16'h0000, `icmp_ident, `icmp_snum};
        txb_pay  <= cmd;          // 32 bytes of echo data
        txb_kind <= 2'd1;
        is_icmp  <= 1'b1;
        st <= P_CS0;
      end
    end
    else if (acc && acc_udp) begin
      peer_mac <= `eth_source; peer_ip <= `ip_src; peer_port <= `udp_src;
      r_cmd <= `CMD; r_sn <= `SN; r_addr <= `ADDRESS;

      if (c_mac && !acc_long) begin
        last_valid <= 1'b0;
        rp_cmd <= "MAC"; rp_sn <= `SN; rp_addr <= 16'd0;
        rp_mac <= `eth_source; rp_ip <= `ip_src; rp_port <= `udp_src;
        st <= P_WAIT;
      end
      else if (c_wip && !acc_long && c_ip_ok && !fl_busy && !fl_evt && (!acc_bcast || c_tgt_me)) begin
        // unicast, or broadcast addressed to our MAC
        ip_to_write <= `NEW_IP;
        write_ip_t  <= ~write_ip_t;     // pic_control samples ip_to_write 2+ slow clocks later
        rp_cmd <= "WIP"; rp_sn <= `SN; rp_addr <= 16'd0;
        rp_mac <= `eth_source; rp_ip <= `ip_src; rp_port <= `udp_src;
        st <= P_WAIT;
      end
      else if (!acc_bcast) begin    // everything below: unicast to our IP only
        if (c_res && !acc_long && !fl_busy && !fl_evt) begin
          dev_reset_t <= ~dev_reset_t;
        end
        else if (!fl_busy && !fl_evt &&
                 ((c_ers && !acc_long && c_slot_ok) ||
                  (c_wpd && acc_long && acc_page_ok && !c_page_hi) ||
                  (c_rpd && !acc_long && !tx_busy && !c_page_hi))) begin
          if (c_dup && !c_rpd) begin
            // lost reply: answer again, do not repeat the flash operation
            rp_cmd <= `CMD; rp_sn <= `SN; rp_addr <= `ADDRESS;
            rp_mac <= `eth_source; rp_ip <= `ip_src; rp_port <= `udp_src;
            st <= P_WAIT;
          end
          else begin
            fl_op    <= c_ers ? 2'd1 : c_wpd ? 2'd2 : 2'd3;
            fl_page  <= `ADDRESS;
            fl_req_t <= ~fl_req_t;
            fl_busy  <= 1'b1;
            cur_cmd <= `CMD; cur_sn <= `SN; cur_addr <= `ADDRESS;
            cur_mac <= `eth_source; cur_ip <= `ip_src; cur_port <= `udp_src;
            if (!c_rpd) last_valid <= 1'b0;  // flash content changes now
          end
        end
        // anything else (busy, bad address, unknown) is silently ignored
      end
    end
    else if (fl_evt) begin
      fl_evt  <= 1'b0;
      fl_busy <= 1'b0;
      if (cur_cmd != "RPD") begin
        last_valid <= 1'b1; last_cmd <= cur_cmd; last_sn <= cur_sn; last_addr <= cur_addr;
      end
      rp_cmd <= cur_cmd; rp_sn <= cur_sn; rp_addr <= cur_addr;
      rp_mac <= cur_mac; rp_ip <= cur_ip; rp_port <= cur_port;
      st <= P_WAIT;
    end
    else if (garp_req && !tx_busy) begin
      garp_clr <= 1'b1;
      txb_hdr <= {48'hFFFF_FFFF_FFFF, local_mac, 16'h0806, 16'h0001, 16'h0800, 8'd6, 8'd4, 16'h0001,
                  local_mac, local_ip, 48'h0000_0000_0000, local_ip};
      txb_pay  <= 256'd0;
      txb_kind <= 2'd0;
      st <= P_SEND;
    end
  //----------------------------------------------------------------
  P_WAIT:                      // UDP reply: wait for the transmitter
    if (!tx_busy) begin
      txb_hdr <= {rp_mac, local_mac, 16'h0800, 8'h45, 8'h00,
                  (rp_cmd == "RPD") ? 16'd316 : 16'd60, ip_id, 16'h0000, 8'd128, 8'h11,
                  16'h0000, local_ip, rp_ip, local_port, rp_port,
                  (rp_cmd == "RPD") ? 16'd296 : 16'd40, 16'h0000};
      ip_id <= ip_id + 1'd1;
      if (rp_cmd == "MAC")
        txb_pay <= {"MAC", rp_sn, FW_VERSION, CAPS, 112'd0, local_ip, local_mac};
      else if (rp_cmd == "WIP")
        txb_pay <= {"WIP", rp_sn, 192'd0, ip_to_write};
      else
        txb_pay <= {rp_cmd, rp_sn, rp_addr, 208'd0};
      txb_kind <= (rp_cmd == "RPD") ? 2'd2 : 2'd1;
      is_icmp  <= 1'b0;
      st <= P_CS0;
    end
  //----------------------------------------------------------------
  P_CS0: begin                 // checksums, stage 1: partial sums of <= 5 words
      ip_p0 <= `TW(14) + `TW(16) + `TW(18) + `TW(20) + `TW(22);
      ip_p1 <= `TW(26) + `TW(28) + `TW(30) + `TW(32);
      ic_p0 <= `TW(38) + `TW(40) + `PW(0)  + `PW(2);
      ic_p1 <= `PW(4)  + `PW(6)  + `PW(8)  + `PW(10) + `PW(12);
      ic_p2 <= `PW(14) + `PW(16) + `PW(18) + `PW(20) + `PW(22);
      ic_p3 <= `PW(24) + `PW(26) + `PW(28) + `PW(30);
      st <= P_CS1;
    end
  P_CS1: begin                 // stage 2
      ip_sum   <= ip_p0 + ip_p1;
      ic_sum_a <= ic_p0 + ic_p1;
      ic_sum_b <= ic_p2 + ic_p3;
      st <= P_CS2;
    end
  P_CS2: begin                 // first fold
      ip_f1  <= ip_sum[15:0] + ip_sum[20:16];
      ic_sum <= ic_sum_a + ic_sum_b;
      st <= P_CS3;
    end
  P_CS3: begin                 // second fold, write checksums
      `TW(24) <= ~(ip_f1[15:0] + ip_f1[16]);
      if (is_icmp) ic_f1 <= ic_sum[15:0] + ic_sum[21:16];
      st <= is_icmp ? P_CS4 : P_SEND;   // ICMP needs one more fold
    end
  P_SEND: begin
      tx_start_t <= ~tx_start_t;
      tx_busy    <= 1'b1;
      st <= P_IDLE;
    end
  P_CS4: begin                 // ICMP final fold
      `TW(36) <= ~(ic_f1[15:0] + ic_f1[16]);
      st <= P_SEND;
    end
  default: st <= P_IDLE;
  endcase
end

`undef eth_source
`undef arp_src_mac
`undef arp_src_ip
`undef ip_tos
`undef ip_size
`undef ip_ident
`undef ip_flags
`undef ip_src
`undef udp_src
`undef icmp_ident
`undef icmp_snum
`undef CMD
`undef SN
`undef ADDRESS
`undef NEW_IP
`undef TGT_MAC
`undef TW
`undef PW

endmodule
