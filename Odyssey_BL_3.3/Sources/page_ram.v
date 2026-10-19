//
//  Odyssey-2 Bootloader 3.3
//  256 x 8 simple dual-port, dual-clock RAM (inferred as one M9K).
//  Write port and read port are in different clock domains; the protocol
//  guarantees that a page is never written and read at the same time.
//

module page_ram (
  input            wclk,
  input            we,
  input      [7:0] waddr,
  input      [7:0] wdata,
  input            rclk,
  input      [7:0] raddr,
  output reg [7:0] q
);

(* ramstyle = "M9K, no_rw_check" *) reg [7:0] mem [0:255];

always @(posedge wclk)
  if (we) mem[waddr] <= wdata;

always @(posedge rclk)
  q <= mem[raddr];

endmodule
