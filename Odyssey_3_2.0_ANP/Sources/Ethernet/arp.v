//
//  HPSDR - High Performance Software Defined Radio
//  Metis code copyright 2010, 2011, 2012, 2013 Phil Harman VK6APH, Alex Shovkoplyas, VE3NEA. (GPL v2+)
//
//-----------------------------------------------------------------------------
// decode arp packet, generate reply
//
// Odyssey 2026:
//  * remote_mac is taken from the rx clock domain (was the tx-synchronised copy)
//  * no reply while local_ip is 0.0.0.0 (DHCP in progress)
//  * gratuitous ARP on request (garp_req, tx domain pulse): announces our
//    random-at-boot MAC so the PC's ARP cache is updated at once
//-----------------------------------------------------------------------------

module arp (
  input reset, //when PHY not connected
  input rx_clock,
  input rx_enable,
  input [7:0] rx_data,
  input tx_clock,
  input [47:0] local_mac,
  input [31:0] local_ip,
  input [47:0] remote_mac,          // rx clock domain
  input tx_enable,
  input garp_req,                   // tx clock domain, 1-clock pulse
  output [7:0] tx_data,
  output [47:0] destination_mac,
  output tx_request,
  output tx_active
);

//-----------------------------------------------------------------------------
//                                receive
//-----------------------------------------------------------------------------
localparam ST_IDLE = 3'd0, ST_RX = 3'd1, ST_TXREQ = 3'd2, ST_TX = 3'd3, ST_ERR = 3'd4;
reg [2:0] state = ST_IDLE;
localparam RX_LEN = 5'd28;
reg [4:0] byte_no;
reg [31:0] remote_ip;
reg [47:0] reply_mac;
wire sending_sync;

always @(posedge rx_clock)
  case (state)
    ST_IDLE:
      if (rx_enable)
        begin
        reply_mac <= remote_mac;
        byte_no <= RX_LEN - 5'd2;
        state <= ST_RX;
        end

    ST_RX:
      if (!rx_enable) state <= ST_IDLE;
      else
        begin
        case (byte_no)
          //arp operation, h0001 = request
          21: if (rx_data != 8'h00) state <= ST_ERR;
          20: if (rx_data != 8'h01) state <= ST_ERR;
          //save sender's ip
          10,11,12,13: remote_ip <= {remote_ip[23:0], rx_data};
          0,1,2,3:
            //compare target ip to our local_ip
            if (rx_data != local_ip[byte_no*8+7 -:8] || local_ip == 32'd0) state <= ST_ERR;
            else if (byte_no == 0) state <= ST_TXREQ;
          endcase
         byte_no <= byte_no - 5'd1;
         end

    ST_TXREQ: if (sending_sync) state <= ST_TX;
    ST_TX: if (!sending_sync) state <= ST_ERR;     // waits for rx_enable low
    ST_ERR: if (!rx_enable) state <= ST_IDLE;
  endcase

//-----------------------------------------------------------------------------
//                                  send
//-----------------------------------------------------------------------------
localparam TX_LEN = 5'd30;
localparam false = 1'b0, true = 1'b1;

wire reply_req;
sync sync_inst1(.clock(tx_clock), .sig_in(state == ST_TXREQ), .sig_out(reply_req));

reg garp_pending = false;
reg sending = false;
reg sending_reply = false;
reg sending_garp = false;
sync sync_inst2(.clock(rx_clock), .sig_in(sending_reply), .sig_out(sending_sync));

assign tx_request = reply_req | garp_pending;

// what is sent if the grant comes now: a pending reply has priority
wire garp_now = sending ? sending_garp : !reply_req;

wire [TX_LEN*8-1:0] reply_bits = {80'h0806_0001_0800_0604_0002, local_mac, local_ip, reply_mac, remote_ip};
wire [TX_LEN*8-1:0] garp_bits  = {80'h0806_0001_0800_0604_0001, local_mac, local_ip, 48'h0, local_ip};
wire [TX_LEN*8-1:0] tx_bits    = garp_now ? garp_bits : reply_bits;

assign destination_mac = garp_now ? 48'hFFFF_FFFF_FFFF : reply_mac;

reg [4:0] tx_byte_no;
assign tx_data = tx_bits[tx_byte_no*8+7 -:8];
assign tx_active = tx_enable | sending;

always @(posedge tx_clock)
  begin
  if (garp_req) garp_pending <= true;

  if (reset) begin sending <= false; sending_reply <= false; sending_garp <= false; end
  else if (tx_enable && !sending) begin
    sending <= true;
    sending_garp  <= garp_now;
    sending_reply <= !garp_now;
    if (garp_now) garp_pending <= false;
  end

  if (tx_active)
    if (tx_byte_no == 5'd0) begin sending <= false; sending_reply <= false; sending_garp <= false; end
    else tx_byte_no <= tx_byte_no - 5'd1;
  else
    tx_byte_no <= TX_LEN - 5'd1;
  end

endmodule
