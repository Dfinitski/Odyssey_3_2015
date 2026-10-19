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
//  April 2016, N2ADR: Added xid, dhcp_seconds_timer


module dhcp (
  //rx in
  input rx_clock,
  input [7:0] rx_data,
  input rx_enable,
  input dhcp_rx_active,
  //rx out
  output reg [31:0] lease = 32'd0,      // DHCP supplied lease time in seconds
  output reg [31:0] server_ip = 32'd0,   // IP address of DHCP server

  //tx in
  input reset,
  input tx_clock,
  input udp_tx_enable,
  input tx_enable,
  input udp_tx_active,
  input [47:0] remote_mac,
  input [31:0] remote_ip,
  input [3:0] dhcp_seconds_timer,
  input force_discover,          // 2026: level, clears is_renewal -> next request is a DISCOVER
  //tx out
  output reg dhcp_tx_request = 1'b0,
  output reg[7:0] tx_data,
  output reg [15:0] length,
  output reg[31:0] ip_accept = 32'd0,
  output reg dhcp_success = 1'b0,
  output reg dhcp_failed = 1'b0,

  //constants
  input [47:0] local_mac,
  output [47:0] dhcp_destination_mac,
  output [31:0] dhcp_destination_ip,
  output [15:0] dhcp_destination_port,
  output unicast_renewal         // 2026: source IP = ciaddr only for renewals


  );

localparam TX_IDLE = 8'd0, DHCPDISCOVER = 8'd1, DHCPSEND = 8'd2, DHCPOFFER = 8'd3, DHCPREQUEST = 8'd4, DHCPRENEW = 8'd5;
localparam RX_IDLE = 8'd0, RX_OFFER_ACK = 8'd1, RX_DONE = 8'd2;

localparam DIS_TX_LEN = 16'd244;    // length of discover message
localparam REQ_TX_LEN = 16'd256;    // length of request message for boot (starting) state
localparam RENEW_TX_LEN = 16'd244;    // length of request message for lease renewal

reg [8:0] byte_no;
reg tx_request;
reg send_discovery;
reg is_renewal = 1'b0;    // Are we renewing the lease?
wire send_request;
reg [31:0] offer_server = 32'd0;      // 2026: IP of the server that made the OFFER (rx domain, quasi-static)
wire force_discover_rx;
sync sync_fd(.clock(rx_clock), .sig_in(force_discover), .sig_out(force_discover_rx));
reg rx_send_request = 1'b0;        // in rx_clock domain (declared early, 2026)
reg disc_t = 1'b0, req_ack_t = 1'b0; // tx domain toggles (2026)
reg [8:0] state = TX_IDLE;

reg [47:0] destination_mac = 48'hFFFFFFFFFFFF;
reg [31:0] destination_ip = 32'hFFFFFFFF;
// 2026: only a lease renewal goes unicast to the server; DISCOVER and the
// REQUEST that answers an OFFER are broadcast (also after a failed renewal)
reg renewal_tx = 1'b0;            // tx domain: current message is a renewal
assign unicast_renewal = renewal_tx;
assign dhcp_destination_mac = renewal_tx ? destination_mac : 48'hFFFF_FFFF_FFFF;
assign dhcp_destination_ip  = renewal_tx ? destination_ip  : 32'hFFFF_FFFF;

reg [15:0] xid_sum = 16'd0;    // random-ish number, constant after the first DHCP Discover
reg xid_done = 0;
wire [7:0] xid0, xid1, xid2, xid3;
assign xid0 = local_mac[15:8];  // this is the final transaction ID
assign xid1 = local_mac[7:0];
assign xid2 = xid_sum[15:8];
assign xid3 = xid_sum[7:0];

//---------------------------------------------------------------
//                DHCP Send
//---------------------------------------------------------------

always @ (posedge tx_clock)
begin
  if (reset) begin                 // 2026: reset was not used before
    state <= TX_IDLE;
    dhcp_tx_request <= 1'b0;
  end
  else
  case (state)
  TX_IDLE:
    begin
    if ( ! xid_done)
      xid_sum <= xid_sum + 1'd1;
    byte_no <= 9'b1;
    dhcp_tx_request <= 1'b0;
    send_discovery <= 1'b0;
      if (tx_enable) begin
        if (is_renewal)
          state <= DHCPRENEW;
        else begin
          state <= DHCPDISCOVER;
          disc_t <= ~disc_t;             // 2026: tell rx a new DISCOVER is out
        end
      end
      else if (send_request) begin
        state <= DHCPREQUEST;
        req_ack_t <= ~req_ack_t;         // 2026: request taken
      end
    end

  DHCPDISCOVER:
    begin
    renewal_tx <= 1'b0;
    xid_done <= 1'b1;
    length <= DIS_TX_LEN;
    dhcp_tx_request <= 1'b1;
      if (udp_tx_enable) begin
        tx_data <= 8'h01;          // tx_data needs to be available before udp_tx_active
        send_discovery <= 1'b1;      // hence this is byte_no 0
        state <= DHCPSEND;
      end
    end

  DHCPREQUEST:
    begin
    renewal_tx <= 1'b0;
    length <= REQ_TX_LEN;
    dhcp_tx_request <= 1'b1;
      if (udp_tx_enable) begin      // tx_data needs to be available before udp_tx_active
        tx_data <= 8'h01;          // hence this is byte_no 0
        state <= DHCPSEND;
      end
    end

  DHCPRENEW:
    begin
    renewal_tx <= 1'b1;
    length <= RENEW_TX_LEN;
    dhcp_tx_request <= 1'b1;
      if (udp_tx_enable) begin      // tx_data needs to be available before udp_tx_active
        tx_data <= 8'h01;          // hence this is byte_no 0
        state <= DHCPSEND;
      end
    end

  DHCPSEND:
    begin
      if (byte_no < length) begin
        if (udp_tx_active) begin
          case (byte_no)
          1: tx_data <= 8'h01;
          2: tx_data <= 8'h06;
          3: tx_data <= 8'h0;
          4: tx_data <= xid0;
          5: tx_data <= xid1;
          6: tx_data <= xid2;
          7: tx_data <= xid3;
          8: tx_data <= 8'd0;
          9: tx_data <= {4'd0, dhcp_seconds_timer};
          10: tx_data <= 8'd0;    // 2 zeros
          12: tx_data <= is_renewal ? ip_accept[31:24] : 8'd0;    // ciaddr
          13: tx_data <= is_renewal ? ip_accept[23:16] : 8'd0;
          14: tx_data <= is_renewal ? ip_accept[15:8]  : 8'd0;
          15: tx_data <= is_renewal ? ip_accept[7:0]   : 8'd0;
          16: tx_data <= 8'd0;    // 12 zeros
          28: tx_data <= local_mac[47:40];    // chaddr
          29: tx_data <= local_mac[39:32];
          30: tx_data <= local_mac[31:24];
          31: tx_data <= local_mac[23:16];
          32: tx_data <= local_mac[15:8];
          33: tx_data <= local_mac[7:0];
          34: tx_data <= 8'd0;    // 202  zeros
         236: tx_data <= 8'h63;
         237: tx_data <= 8'h82;
         238: tx_data <= 8'h53;
         239: tx_data <= 8'h63;
         240: tx_data <= 8'h35;
         241: tx_data <= 8'h01;
         242: tx_data <= send_discovery ? 8'h01 : 8'h03;
         243: tx_data <= (send_discovery | renewal_tx) ? 8'hFF : 8'h32;  // send discovery ends here
         244: tx_data <= 8'h04;                // start of REQUEST
         245: tx_data <= ip_accept[31:24];    // Requested IP
         246: tx_data <= ip_accept[23:16];
         247: tx_data <= ip_accept[15:8];
         248: tx_data <= ip_accept[7:0];
         249: tx_data <= is_renewal ? 8'hFF : 8'h36;    // Renewal Request ends here
         250: tx_data <= 8'h04;    // Original Request requires the Server Identifier
         251: tx_data <= offer_server[31:24];
         252: tx_data <= offer_server[23:16];
         253: tx_data <= offer_server[15:8];
         254: tx_data <= offer_server[7:0];
         255: tx_data <= 8'hFF;
        endcase
        byte_no <= byte_no + 9'd1;
       end
       end
    else  state <= TX_IDLE;     // all data sent so return to start
     end
  endcase
end

//-----------------------------------------------------------------------------
//                               output
//-----------------------------------------------------------------------------
assign dhcp_destination_port = 16'd67;

//--------------------------------------------------------------------
//                Transfer data from rx to tx clock domains
//--------------------------------------------------------------------

sync sync_inst1(.clock(tx_clock), .sig_in(rx_send_request), .sig_out(send_request));



//---------------------------------------------------------------
//                DHCP Receive  (rewritten 2026)
//---------------------------------------------------------------
// Parses the whole reply (fixed BOOTP part + option list in any order) and
// acts only at the END option:
//   OFFER : first one after each DISCOVER -> REQUEST (requested IP = yiaddr,
//           server id = option 54, or the IP source if 54 is missing)
//   ACK   : ip_accept / lease / server -> dhcp_success
//   NAK   : dhcp_failed
// yiaddr is kept in a temporary register until then (a NAK or a second
// OFFER can no longer overwrite ip_accept).
localparam R_IDLE = 3'd0, R_FIXED = 3'd1, R_CODE = 3'd2, R_LEN = 3'd3, R_DATA = 3'd4, R_DONE = 3'd5;
reg [2:0]  rx_state = R_IDLE;
reg [8:0]  rx_byte_no = 9'd0;
reg [47:0] target_mac = 48'd0;
reg [31:0] temp_ip_accept = 32'd0;
reg [7:0]  opt_code = 8'd0, opt_len = 8'd0, opt_idx = 8'd0;
reg [7:0]  msg_type = 8'd0;
reg [31:0] srv_opt = 32'd0, lease_opt = 32'd0;
reg        srv_valid = 1'b0, lease_valid = 1'b0;
reg        offer_taken = 1'b0;

wire en_rx;                                   // dhcp_enable (2.5 MHz domain) synchronised
sync sync_en(.clock(rx_clock), .sig_in(rx_enable), .sig_out(en_rx));
wire disc_t_rx, req_ack_rx;
sync sync_disc(.clock(rx_clock), .sig_in(disc_t), .sig_out(disc_t_rx));
sync sync_rack(.clock(rx_clock), .sig_in(req_ack_t), .sig_out(req_ack_rx));
reg disc_seen = 1'b0, req_ack_seen = 1'b0;

always @ (posedge rx_clock)
begin
  if (force_discover_rx) is_renewal <= 1'b0;
  // a new DISCOVER was sent -> accept the next OFFER
  if (disc_t_rx != disc_seen) begin disc_seen <= disc_t_rx; offer_taken <= 1'b0; end
  // tx has taken the REQUEST
  if (req_ack_rx != req_ack_seen) begin req_ack_seen <= req_ack_rx; rx_send_request <= 1'b0; end

  if (!en_rx) begin
    dhcp_success <= 1'b0;
    dhcp_failed  <= 1'b0;
  end

  if (!(dhcp_rx_active && en_rx))
    rx_state <= R_IDLE;
  else
    case (rx_state)
      R_IDLE: begin
        rx_byte_no <= 9'd1;
        msg_type <= 8'd0; srv_valid <= 1'b0; lease_valid <= 1'b0;
        rx_state <= (rx_data == 8'h02) ? R_FIXED : R_DONE;     // op = BOOTREPLY
      end

      R_FIXED: begin
        rx_byte_no <= rx_byte_no + 9'd1;
        case (rx_byte_no)
          4: if (rx_data != xid0) rx_state <= R_DONE;
          5: if (rx_data != xid1) rx_state <= R_DONE;
          6: if (rx_data != xid2) rx_state <= R_DONE;
          7: if (rx_data != xid3) rx_state <= R_DONE;
          16,17,18,19: temp_ip_accept <= {temp_ip_accept[23:0], rx_data};   // yiaddr
          28,29,30,31,32,33: target_mac <= {target_mac[39:0], rx_data};      // chaddr
          34: if (target_mac != local_mac) rx_state <= R_DONE;
          236: if (rx_data != 8'h63) rx_state <= R_DONE;                    // magic cookie
          237: if (rx_data != 8'h82) rx_state <= R_DONE;
          238: if (rx_data != 8'h53) rx_state <= R_DONE;
          239: if (rx_data != 8'h63) rx_state <= R_DONE; else rx_state <= R_CODE;
          default: ;
        endcase
      end

      R_CODE:
        if (rx_data == 8'hFF) begin                                  // END: act
          rx_state <= R_DONE;
          if (msg_type == 8'd2 && !offer_taken) begin                // OFFER
            ip_accept       <= temp_ip_accept;
            offer_server    <= srv_valid ? srv_opt : remote_ip;
            rx_send_request <= 1'b1;
            offer_taken     <= 1'b1;
          end
          else if (msg_type == 8'd5) begin                           // ACK
            ip_accept       <= temp_ip_accept;
            lease           <= lease_valid ? lease_opt : 32'd0;
            server_ip       <= srv_valid ? srv_opt : remote_ip;
            destination_mac <= remote_mac;                           // next hop to the server
            destination_ip  <= srv_valid ? srv_opt : remote_ip;
            is_renewal      <= 1'b1;
            dhcp_success    <= 1'b1;
            dhcp_failed     <= 1'b0;
          end
          else if (msg_type == 8'd6) begin                           // NAK
            dhcp_failed     <= 1'b1;
            dhcp_success    <= 1'b0;
          end
        end
        else if (rx_data != 8'h00) begin opt_code <= rx_data; rx_state <= R_LEN; end   // 0 = pad

      R_LEN: begin
        opt_len <= rx_data;
        opt_idx <= 8'd0;
        rx_state <= (rx_data == 8'd0) ? R_CODE : R_DATA;
      end

      R_DATA: begin
        if (opt_code == 8'd53 && opt_idx == 8'd0) msg_type <= rx_data;
        if (opt_code == 8'd54 && opt_len == 8'd4) begin srv_opt   <= {srv_opt[23:0], rx_data};   if (opt_idx == 8'd3) srv_valid   <= 1'b1; end
        if (opt_code == 8'd51 && opt_len == 8'd4) begin lease_opt <= {lease_opt[23:0], rx_data}; if (opt_idx == 8'd3) lease_valid <= 1'b1; end
        opt_idx <= opt_idx + 1'd1;
        if (opt_idx == opt_len - 1'd1) rx_state <= R_CODE;
      end

      R_DONE: ;                                     // wait for the end of the packet
      default: rx_state <= R_IDLE;
    endcase
end

endmodule