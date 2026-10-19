//
//  HPSDR - High Performance Software Defined Radio
//  Copyright P Harman VK6APH  2012  (GPL v2 or later)
//
//  Simple Clock Domain Crossing module that double buffers data.
//
//  Odyssey BL 3.3: registers are marked as synchronizers so Quartus
//  keeps them, places them close together and reports MTBF.
//  Use ONLY for single-bit levels / toggles, or for multi-bit buses that
//  are guaranteed quasi-static (stable long before they are sampled).
//

`timescale 1 ns/100 ps

module cdc_sync #(parameter SIZE=1)
  (input  wire [SIZE-1:0] siga,
   input  wire            rstb, clkb,
   output reg  [SIZE-1:0] sigb);

(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION \"FORCED IF ASYNCHRONOUS\"; -name DONT_MERGE_REGISTER ON; -name PRESERVE_REGISTER ON" *)
reg [SIZE-1:0] q1;

initial begin q1 = {SIZE{1'b0}}; sigb = {SIZE{1'b0}}; end

always @(posedge clkb)
begin
  if (rstb)
    {sigb,q1} <= {2*SIZE{1'b0}};
  else
    {sigb,q1} <= {q1,siga};
end

endmodule
