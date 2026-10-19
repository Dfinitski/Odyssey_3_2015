// Behavioural EPCS64 subset: 06 WREN, 05 RDSR, D8 sector erase, 02 page program, 03 read.
// Checks tCSH >= 100 ns, SPI mode 0, WEL handling, page wrap, busy during WIP.
`timescale 1ns/1ps
module epcs_model (input DCLK, input ASDI, input nCS, output reg DATA0);
  parameter real T_ERASE = 20000.0;   // ns (shortened)
  parameter real T_PROG  = 3000.0;
  reg [7:0] mem [0:8388607];
  integer i;
  initial begin for (i = 0; i < 8388608; i = i + 1) mem[i] = 8'h00; DATA0 = 1'bz; end

  reg wip = 0, wel = 0;
  integer bitc; reg [7:0] cmd; reg [23:0] addr; reg [7:0] sh; reg [7:0] outb;
  integer nbytes; reg [7:0] pbuf [0:255]; integer pcnt;
  real t_rise; integer errors = 0, n_erase = 0, n_prog = 0, n_read = 0;
  reg [7:0] pstart;
  real wip_end = 0;
  always #50 if (wip && $realtime >= wip_end) wip = 0;

  always @(posedge nCS) t_rise = $realtime;
  always @(negedge nCS) begin
    if ($realtime - t_rise < 100.0 && t_rise > 0) begin
      $display("EPCS ERROR: tCSH violated %0.1f ns at %t", $realtime - t_rise, $realtime); errors = errors + 1;
    end
    bitc = 0; nbytes = 0; pcnt = 0;
  end

  always @(posedge DCLK) if (!nCS) begin
    sh = {sh[6:0], ASDI};
    bitc = bitc + 1;
    if (bitc % 8 == 0) begin
      if (nbytes == 0) cmd = sh;
      else if (nbytes <= 3) addr = {addr[15:0], sh};
      else if (cmd == 8'h02) begin pbuf[pcnt[7:0]] = sh; pcnt = pcnt + 1; end
      nbytes = nbytes + 1;
      if (nbytes == 1 && wip && cmd != 8'h05) begin
        $display("EPCS ERROR: cmd %h while busy at %t", cmd, $realtime); errors = errors + 1; end
      if (nbytes == 1 && cmd == 8'h05) outb = {6'b0, wel, wip};
      if (cmd == 8'h03 && nbytes >= 4) begin outb = mem[addr]; addr = addr + 1; end
    end
  end

  // output: next bit after falling edge
  always @(negedge DCLK) if (!nCS) begin
    if (cmd == 8'h05 && nbytes >= 1) begin
      DATA0 <= #8 outb[7]; outb = {outb[6:0], outb[7]};
    end
    else if (cmd == 8'h03 && nbytes >= 4) begin
      DATA0 <= #8 outb[7]; outb = {outb[6:0], 1'b0};
    end
  end

  always @(posedge nCS) begin
    DATA0 <= #5 1'bz;
    if (nbytes == 1 && cmd == 8'h06) wel = 1;
    else if (cmd == 8'hD8 && nbytes == 4) begin
      if (!wel) begin $display("EPCS ERROR: erase without WEL"); errors = errors + 1; end
      else begin
        wip = 1; wel = 0; n_erase = n_erase + 1;
        for (i = 0; i < 65536; i = i + 1) mem[{addr[23:16],16'h0000} + i] = 8'hFF;
        wip_end = $realtime + T_ERASE;
      end
    end
    else if (cmd == 8'h02 && nbytes >= 5) begin
      if (!wel) begin $display("EPCS ERROR: program without WEL"); errors = errors + 1; end
      else begin
        if (pcnt != 256) $display("EPCS note: program of %0d bytes", pcnt);
        wip = 1; wel = 0; n_prog = n_prog + 1;
        pstart = addr[7:0];
        for (i = 0; i < pcnt && i < 256; i = i + 1)
          mem[{addr[23:8], pstart + i[7:0]}] = mem[{addr[23:8], pstart + i[7:0]}] & pbuf[i];
        wip_end = $realtime + T_PROG;
      end
    end
    else if (cmd == 8'h03) n_read = n_read + 1;
  end
endmodule
