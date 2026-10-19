//
//  HPSDR - High Performance Software Defined Radio
//  Metis code copyright 2010, 2011, 2012, 2013 Alex Shovkoplyas, VE3NEA. (GPL v2+)
//
//  Odyssey 2026 rework:
//   * no gated / multiplexed FIFO clock any more (the old icmp_fifo was
//     clocked by "rx_clk_en ? rx_clock : tx_clock") - a dual-clock M9K RAM
//   * replies only to echo requests sent to our own IP (not broadcast)
//   * correct ICMP header (type 0, code 0, checksum), checksum with 2 folds
//   * echo data up to 2048 bytes
//
//  rx side (rx_clock): ip_recv delivers the IP payload (= ICMP message, exact
//  length, no padding/FCS) while rx_enable is high.
//  tx side (tx_clock): same handshake as arp.v - tx_request (level), one
//  tx_enable pulse from the arbiter, then tx_active while bytes are sent.
//

module icmp (
  input reset,
  input rx_clock,
  input rx_enable,
  input [7:0] rx_data,
  input broadcast,
  input [31:0] to_ip,
  input [31:0] local_ip,
  input [47:0] remote_mac,      // rx clock domain
  input [31:0] remote_ip,       // rx clock domain

  input tx_clock,
  input tx_enable,
  output tx_request,
  output tx_active,
  output [7:0] tx_data,
  output reg [15:0] length = 16'd0,
  output reg [47:0] destination_mac = 48'd0,
  output reg [31:0] destination_ip = 32'd0,
  output dst_unreachable
);

assign dst_unreachable = 1'b0;

//-----------------------------------------------------------------------------
//                               receive
//-----------------------------------------------------------------------------
localparam ST_IDLE = 3'd0, ST_CODE = 3'd1, ST_CS = 3'd2, ST_PAYLOAD = 3'd3,
           ST_FOLD = 3'd4, ST_TXREQ = 3'd5, ST_TX = 3'd6, ST_DONE = 3'd7;
reg [2:0]  state = ST_IDLE;
reg        cs_byte = 1'b0;
reg [11:0] wcnt = 12'd0;
reg [31:0] sum = 32'd0;
reg [16:0] f1 = 17'd0;
reg [15:0] checksum = 16'd0;
wire       sending_sync;

reg        ram_we = 1'b0;
reg [10:0] ram_waddr = 11'd0;
reg [7:0]  ram_wdata = 8'd0;

always @(posedge rx_clock) begin
  ram_we <= 1'b0;
  case (state)
    ST_IDLE:
      if (rx_enable) begin                       // byte 0 = type
        destination_mac <= remote_mac;
        destination_ip  <= remote_ip;
        state <= (rx_data == 8'h08 && !broadcast && to_ip == local_ip && local_ip != 32'd0) ? ST_CODE : ST_DONE;
      end
    ST_CODE:
      if (!rx_enable) state <= ST_IDLE;
      else begin cs_byte <= 1'b0; state <= (rx_data == 8'h00) ? ST_CS : ST_DONE; end
    ST_CS:                                       // 2 checksum bytes, skipped
      if (!rx_enable) state <= ST_IDLE;
      else if (cs_byte) begin wcnt <= 12'd0; sum <= 32'd0; state <= ST_PAYLOAD; end
      else cs_byte <= 1'b1;
    ST_PAYLOAD:
      if (!rx_enable) state <= ST_FOLD;          // end of ICMP message
      else if (wcnt == 12'd2048) state <= ST_DONE; // too long, no reply
      else begin
        ram_we    <= 1'b1;
        ram_waddr <= wcnt[10:0];
        ram_wdata <= rx_data;
        sum  <= sum + (wcnt[0] ? {24'd0, rx_data} : {16'd0, rx_data, 8'd0});
        wcnt <= wcnt + 1'd1;
      end
    ST_FOLD: begin                               // 2-step fold of the 1's complement sum
        f1 <= sum[15:0] + sum[31:16];
        length <= 16'd4 + {4'd0, wcnt};
        state <= ST_TXREQ;
      end
    ST_TXREQ: begin
        checksum <= ~(f1[15:0] + f1[16]);
        if (sending_sync) state <= ST_TX;
      end
    ST_TX:   if (!sending_sync) state <= ST_DONE;   // ST_DONE waits for rx_enable low
    ST_DONE: if (!rx_enable) state <= ST_IDLE;
  endcase
end

//-----------------------------------------------------------------------------
//                       echo data storage (M9K, dual clock)
//-----------------------------------------------------------------------------
(* ramstyle = "M9K, no_rw_check" *) reg [7:0] ram [0:2047];
reg  [7:0]  ram_q = 8'd0;
wire [10:0] ram_raddr;
always @(posedge rx_clock) if (ram_we) ram[ram_waddr] <= ram_wdata;
always @(posedge tx_clock) ram_q <= ram[ram_raddr];

//-----------------------------------------------------------------------------
//                               send
//-----------------------------------------------------------------------------
sync sync_inst1(.clock(tx_clock), .sig_in(state == ST_TXREQ), .sig_out(tx_request));
reg sending = 1'b0;
sync sync_inst2(.clock(rx_clock), .sig_in(sending), .sig_out(sending_sync));

reg  [15:0] idx = 16'd0;
wire [15:0] idx_eff = sending ? idx : 16'd0;
wire [15:0] ra = idx_eff - 16'd3;            // RAM is read one byte ahead
assign ram_raddr = ra[10:0];

assign tx_active = tx_enable | sending;
assign tx_data = (idx_eff == 16'd0) ? 8'h00 :            // type 0 = echo reply
                 (idx_eff == 16'd1) ? 8'h00 :            // code
                 (idx_eff == 16'd2) ? checksum[15:8] :
                 (idx_eff == 16'd3) ? checksum[7:0]  :
                                      ram_q;

always @(posedge tx_clock)
  if (reset) begin sending <= 1'b0; idx <= 16'd0; end
  else if (!sending) begin
    idx <= 16'd1;
    if (tx_enable) sending <= 1'b1;            // byte 0 is out in this cycle
  end
  else if (idx == length - 16'd1) sending <= 1'b0;
  else idx <= idx + 1'd1;

endmodule
