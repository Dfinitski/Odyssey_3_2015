`timescale 1ns/1ps
module tb;

// ---------------- clocks ----------------
reg ina_clk = 0;  always #4.069 ina_clk = ~ina_clk;
reg phy125  = 0;  initial #1.3 forever #4.000 phy125 = ~phy125;
reg rxsrc   = 0;  initial #0.7 forever #4.0002 rxsrc = ~rxsrc;   // PHY recovered clock (async)
wire rxclk;  assign #2.0 rxclk = rxsrc;                         // RGMII-ID: clock +2 ns vs data

// ---------------- DUT ----------------
reg  [3:0] rxd = 4'hD; reg rxctl = 0;
wire [3:0] txd; wire txen, gtx;
wire mdio, mdc, phy_rst_n;
wire dclk, asdo, ncs, data0;
wire u_tx; reg u_rx = 1;
wire [7:0] uo; wire [2:0] band;
pullup(mdio);
pulldown(data0);

Bootloader dut (
  .INA_CLK(ina_clk),
  .PHY_TX(txd), .PHY_RX(rxd), .PHY_RX_DV(rxctl), .PHY_TX_CLOCK(gtx), .PHY_TX_EN(txen),
  .PHY_RX_CLOCK(rxclk), .PHY_CLK_125(phy125), .PHY_MDIO(mdio), .PHY_MDC(mdc), .PHY_RESET_N(phy_rst_n),
  .DCLK(dclk), .DATAOUT(asdo), .DATAIN(data0), .FLASH_NCE(ncs),
  .NCONFIG(), .KEY_DOT(1'b0), .KEY_DASH(1'b0), .PTT_in_UO(1'b0), .PTT_in(1'b0), .random_bit(1'b1),
  .UO(uo), .BAND(band), .TUNE(), .ANT_2(), .ANT_3(), .PTT_out(), .PTT_out_mcu(), .LNA(),
  .u_rx(u_rx), .u_tx(u_tx), .led1(), .led2(), .led3(), .led4(), .test1(), .test2()
);
defparam dut.proc_inst.GARP_DLY1 = 32'd2000;
defparam dut.proc_inst.GARP_DLY2 = 32'd3000;
defparam dut.phy_config.DELAY    = 16'd200;

epcs_model flash (.DCLK(dclk), .ASDI(asdo), .nCS(ncs), .DATA0(data0));

// ---------------- MCU (UART, 32 slow clocks per bit) ----------------
wire sclk = dut.C_80kHz;
task mcu_send(input [7:0] b);
  integer i;
  begin
    @(posedge sclk); u_rx = 0; repeat (32) @(posedge sclk);
    for (i = 0; i < 8; i = i + 1) begin u_rx = b[i]; repeat (32) @(posedge sclk); end
    u_rx = 1; repeat (40) @(posedge sclk);
  end
endtask

reg [7:0] mcu_rx [0:63]; integer mcu_n = 0;
initial begin : mcu_rx_proc
  integer i; reg [7:0] b;
  forever begin
    @(negedge u_tx);
    repeat (16) @(posedge sclk);              // middle of start bit
    for (i = 0; i < 8; i = i + 1) begin repeat (32) @(posedge sclk); b[i] = u_tx; end
    repeat (32) @(posedge sclk);
    mcu_rx[mcu_n] = b; mcu_n = mcu_n + 1;
    $display("%t MCU <- %02h", $realtime, b);
  end
end

initial begin : mcu_main
  wait (mcu_n == 9);                           // hello: 01 + version + mode
  #1000;
  mcu_send(8'h02);
  mcu_send(8'd192); mcu_send(8'd168); mcu_send(8'd2); mcu_send(8'd160);
  mcu_send(8'd1);                              // slot
  mcu_send(8'd1);                              // mode: bootloader
end

// ---------------- MDIO PHY model (address 5, ID1 = 0x0022) ----------------
reg mdio_drv = 0, mdio_val = 1;
assign mdio = mdio_drv ? mdio_val : 1'bz;
integer m_ones = 0, m_bit = -1; reg [13:0] m_hdr; reg [15:0] m_dat, m_out;
reg [4:0] m_phy, m_reg; reg m_rd;
integer mdio_writes = 0, mdio_reads = 0;
always @(posedge mdc) begin
  if (m_bit < 0) begin
    if (mdio === 1'b1) m_ones = m_ones + 1;
    else if (mdio === 1'b0 && m_ones >= 32) begin m_bit = 1; m_hdr = 14'b0; m_ones = 0; end
    else m_ones = 0;
  end
  else begin
    m_bit = m_bit + 1;
    if (m_bit <= 14) m_hdr = {m_hdr[12:0], mdio};
    if (m_bit == 14) begin
      m_rd  = (m_hdr[11:10] == 2'b10);
      m_phy = m_hdr[9:5]; m_reg = m_hdr[4:0];
      if (m_hdr[12] !== 1'b1) $display("MDIO ERROR: bad ST");
    end
    if (m_bit > 16 && m_bit <= 32 && !m_rd) m_dat = {m_dat[14:0], mdio};
    if (m_rd && m_bit == 15) begin
      mdio_reads = mdio_reads + 1;
      m_out = (m_phy == 5 && m_reg == 2) ? 16'h0022 : 16'h1234;
      if (m_phy == 5) begin mdio_drv <= #20 1'b1; mdio_val <= #20 1'b0; end
    end
    else if (m_rd && m_bit >= 16 && m_bit <= 31 && m_phy == 5) begin
      mdio_val <= #20 m_out[15]; m_out = {m_out[14:0], 1'b1};
    end
    if (m_bit == 32) begin
      if (!m_rd) begin mdio_writes = mdio_writes + 1;
        $display("%t MDIO WR phy=%0d reg=%02h val=%04h", $realtime, m_phy, m_reg, m_dat); end
      else if (m_phy == 5) $display("%t MDIO RD phy=%0d reg=%02h -> %04h", $realtime, m_phy, m_reg, rd_shadow(m_reg));
      mdio_drv <= #20 1'b0;
      m_bit = -1;
    end
  end
end
function [15:0] rd_shadow(input [4:0] r); rd_shadow = (r == 2) ? 16'h0022 : 16'h1234; endfunction

// ---------------- RGMII RX driver ----------------
reg [7:0] fb [0:2047]; integer flen;
task send_frame;
  integer i;
  begin
    @(posedge rxsrc);
    for (i = 0; i < flen; i = i + 1) begin
      rxd = fb[i][3:0]; rxctl = 1;
      @(negedge rxsrc); rxd = fb[i][7:4]; rxctl = 1;
      @(posedge rxsrc);
    end
    rxd = 4'hD; rxctl = 0;                    // in-band status: link up, 1000, FD
    @(negedge rxsrc); rxd = 4'hD;
    repeat (12) @(posedge rxsrc);
  end
endtask

// ---------------- RGMII TX monitor ----------------
integer txf, tx_count = 0, tx_len = 0; reg [7:0] tb_b [0:2047];
reg [3:0] lo; reg en_r, en_f; integer k;
initial txf = $fopen("tx_frames.txt", "w");
always @(posedge gtx) begin lo = txd; en_r = txen; end
always @(negedge gtx) begin
  en_f = txen;
  if (en_r) begin tb_b[tx_len] = {txd, lo}; tx_len = tx_len + 1; end
  else if (tx_len > 0) begin
    $fwrite(txf, "%0t ", $realtime);
    for (k = 0; k < tx_len; k = k + 1) $fwrite(txf, "%02h", tb_b[k]);
    $fwrite(txf, "\n"); $fflush(txf);
    tx_count = tx_count + 1; tx_len = 0;
  end
  if (en_r !== en_f) $display("TX ERROR: TX_CTL differs between edges");
end

// ---------------- stimulus ----------------
integer sf, r, n, i; reg [8*16-1:0] tok; reg [8*8192-1:0] hex; integer v;
initial begin
  $timeformat(-6, 2, " us", 10);
  sf = $fopen("stim.txt", "r");
  wait (dut.proc_inst.ip_ready && dut.phy_cfg_done);
  $display("%t network ready, PHY addr %0d found=%0d", $realtime, dut.phy_addr, dut.phy_found);
  while (!$feof(sf)) begin
    r = $fscanf(sf, "%s", tok);
    if (r != 1) ;
    else if (tok == "D") begin r = $fscanf(sf, "%d", n); #(n); end
    else if (tok == "W") begin
      r = $fscanf(sf, "%d", n);
      fork : waitblk
        begin wait (tx_count >= n); disable waitblk; end
        begin #(5_000_000); $display("%t TIMEOUT waiting for tx frame %0d", $realtime, n); disable waitblk; end
      join
    end
    else if (tok == "M") begin           // marker
      r = $fscanf(sf, "%d", n); $fwrite(txf, "M %0d %0d\n", n, tx_count); $fflush(txf);
    end
    else if (tok == "F") begin
      r = $fscanf(sf, "%d", flen);
      for (i = 0; i < flen; i = i + 1) begin r = $fscanf(sf, "%h", v); fb[i] = v; end
      send_frame;
    end
    else if (tok == "Q") begin
      #20000;
      $writememh("flash_slot1.hex", flash.mem, 24'h200000, 24'h2004FF);
      $display("EPCS: erases=%0d programs=%0d reads=%0d errors=%0d", flash.n_erase, flash.n_prog, flash.n_read, flash.errors);
      $display("MDIO: writes=%0d reads=%0d", mdio_writes, mdio_reads);
      $display("MCU bytes=%0d", mcu_n);
      for (i = 0; i < mcu_n; i = i + 1) $write("%02h ", mcu_rx[i]); $write("\n");
      $fclose(txf);
      $finish;
    end
  end
end

initial begin #200_000_000; $display("GLOBAL TIMEOUT"); $finish; end
endmodule
