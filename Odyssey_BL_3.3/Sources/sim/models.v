// Simulation-only behavioural models of the Altera megafunctions used by
// the Odyssey bootloader. Cyclone IV semantics.
`timescale 1ns/1ps

// altddio_in, width 5, invert_input_clocks = ON
//   dataout_h : sampled on the FALLING edge of inclock
//   dataout_l : sampled on the RISING edge, re-timed to the falling edge
module ddio_in (input [4:0] datain, input inclock, output [4:0] dataout_h, output [4:0] dataout_l);
  reg [4:0] lr = 0, hq = 0, lq = 0;
  always @(posedge inclock) lr <= datain;
  always @(negedge inclock) begin hq <= datain; lq <= lr; end
  assign dataout_h = hq;
  assign dataout_l = lq;
endmodule

// altddio_out width 5: both registered on rising edge, h out while clock high
module ddio_out (input [4:0] datain_h, input [4:0] datain_l, input outclock, output [4:0] dataout);
  reg [4:0] h = 0, l = 0, l2 = 0;
  always @(posedge outclock) begin h <= datain_h; l <= datain_l; end
  always @(negedge outclock) l2 <= l;
  assign dataout = outclock ? h : l2;
endmodule

module ddio_out1 (input datain_h, input datain_l, input outclock, output dataout);
  reg h = 0, l = 0, l2 = 0;
  always @(posedge outclock) begin h <= datain_h; l <= datain_l; end
  always @(negedge outclock) l2 <= l;
  assign dataout = outclock ? h : l2;
endmodule

// PLL_125: c0 = in, c1 = in + 2.0 ns (90 deg), c2 = in / 10
module C125_PLL (input areset, input inclk0, output c0, output c1, output c2, output reg locked = 0);
  always @(posedge inclk0 or posedge areset) if (areset) locked <= 0; else locked <= #500 1'b1;
  reg r1 = 0, r2 = 0;
  integer n = 0;
  assign c0 = inclk0;
  always @(inclk0) r1 <= #2.0 inclk0;
  assign c1 = r1;
  always @(posedge inclk0) begin
    n = (n == 9) ? 0 : n + 1;
    if (n == 0) r2 <= 1'b1;
    if (n == 5) r2 <= 1'b0;
  end
  assign c2 = r2;
endmodule

// PLL_SLOW: "80 kHz" - accelerated for simulation (period SLOW_HALF*2 ns)
module SLOW_PLL (input inclk0, output c0, output locked);
  parameter SLOW_HALF = 25.0;     // 20 MHz in simulation
  reg c = 0, l = 0;
  initial begin #1000 l = 1; end
  always #(SLOW_HALF) c = ~c;
  assign c0 = c;
  assign locked = l;
endmodule

module altera_remote_update_core (
  input clock, input [23:0] data_in, input [2:0] param, input read_param,
  input [1:0] read_source, input reconfig, input reset, input reset_timer,
  output busy, output [28:0] data_out, input write_param);
  assign busy = 1'b0;
  assign data_out = 29'd0;
endmodule
