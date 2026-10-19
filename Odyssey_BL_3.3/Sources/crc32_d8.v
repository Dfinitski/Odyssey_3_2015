//
//  Odyssey-2 Bootloader
//  IEEE 802.3 CRC32, byte-parallel, reflected (LSB first), poly 0xEDB88320.
//
//  Usage:  init crc = 32'hFFFF_FFFF before the first byte after SFD,
//          crc <= nxt for every frame byte.
//          TX : FCS = ~crc, sent LSB byte first (~crc[7:0] first).
//          RX : after all bytes INCLUDING the 4 FCS bytes crc == 32'hDEBB_20E3
//               (residue) for a good frame.
//
//  Pure combinational, ~3 LUT levels in Cyclone IV - fine for 125 MHz.
//

module crc32_d8 (
  input  [31:0] crc,
  input  [7:0]  d,
  output [31:0] nxt
);

function [31:0] crc_byte;
  input [31:0] c;
  input [7:0]  b;
  integer i;
  reg [31:0] x;
  begin
    x = c;
    for (i = 0; i < 8; i = i + 1)
      x = (x[0] ^ b[i]) ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
    crc_byte = x;
  end
endfunction

assign nxt = crc_byte(crc, d);

endmodule
