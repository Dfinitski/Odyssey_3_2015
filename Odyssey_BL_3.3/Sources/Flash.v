//
// N7DDC, David Fainitski, project Odyssey-II, 04.2018
// Bootloader 3.3 rework:
//   * request/acknowledge by toggles, op code + page address quasi-static
//   * nCS high time >= 4 clocks (320 ns; EPCS tCSH >= 100 ns, W25Q64JV tSHSL >= 50 ns)
//   * page data comes from / goes to dual-clock M9K page RAMs
//   * new: READ PAGE (03h) for verification
//   * all pins driven from registers (placed in IOE by FAST_xxx_REGISTER)
//
// Flash on the board: Winbond W25Q64JV (EPCS64 compatible command set:
//   06 WREN, 05 RDSR1 (bit0 BUSY), D8 64 KB block erase, 02 page program, 03 read).
//   W25Q64JV: tSHSL >= 50 ns (here >= 320 ns), fR(03h) <= 50 MHz,
//   block erase typ 150 ms / max 2 s, page program typ 0.4 ms / max 3 ms.
// Clock: 12.5 MHz.  SPI mode 0, one bit = 3 clocks (240 ns, ~4.2 MHz).
// Bit order: the RBF byte is sent LSB first (EPCS AS-mode convention), and
// read back the same way, so RAM contents always hold plain RBF bytes.
//
//   op 1 : erase 32 sectors (one 2 MB slot) starting at page[15:8] sector
//   op 2 : program 256 bytes at page
//   op 3 : read 256 bytes at page
//

module flash (
  input             clock,
  input             req_t,          // toggle (raw, from rx domain)
  input      [1:0]  op,             // quasi-static
  input      [15:0] page,           // quasi-static
  output reg        done_t = 1'b0,
  output            busy,

  output reg [7:0]  wr_raddr = 8'd0, // write-page RAM read port (this clock)
  input      [7:0]  wr_q,
  output reg        rd_we = 1'b0,    // read-page RAM write port (this clock)
  output reg [7:0]  rd_waddr = 8'd0,
  output reg [7:0]  rd_wdata = 8'd0,

  output reg        DCLK = 1'b0,
  output reg        DATAOUT = 1'b0,
  input             DATAIN,
  output reg        FLASH_NCE = 1'b1
);

wire req_s;
cdc_sync #(1) req_sync (.siga(req_t), .rstb(1'b0), .clkb(clock), .sigb(req_s));
reg  req_seen = 1'b0;

// DATAIN input register (IOE)
reg din_r = 1'b0;
always @(posedge clock) din_r <= DATAIN;

function [7:0] rev8; input [7:0] b; rev8 = {b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7]}; endfunction

// ---------------- byte engine ----------------
// load tx byte + start -> 8 bits shifted MSB first, rx byte captured.
reg  [7:0] sh_tx = 8'd0, sh_rx = 8'd0;
reg  [2:0] bitn = 3'd0;
reg  [1:0] ph = 2'd0;

// ---------------- sequencer ----------------
localparam  F_IDLE   = 5'd0,
            F_WREN   = 5'd1,  F_WREN_E = 5'd2,
            F_CMD    = 5'd3,  F_A2 = 5'd4, F_A1 = 5'd5, F_A0 = 5'd6,
            F_DATA   = 5'd7,  F_DLOAD = 5'd8, F_DLOAD2 = 5'd9,
            F_RDATA  = 5'd10,
            F_END    = 5'd11,
            F_POLL   = 5'd12, F_POLL_R = 5'd13, F_POLL_E = 5'd14,
            F_NEXT   = 5'd15,
            F_BYTE   = 5'd16,   // byte engine
            F_GAP    = 5'd17,   // nCS high gap
            F_DONE   = 5'd18;

reg [4:0]  st = F_IDLE, ret = F_IDLE, gap_ret = F_IDLE;
reg [1:0]  r_op = 2'd0;
reg [23:0] addr = 24'd0;
reg [5:0]  sect = 6'd0;
reg [8:0]  bcnt = 9'd0;
reg [2:0]  gap = 3'd0;

assign busy = (st != F_IDLE);

always @(posedge clock) begin
  rd_we <= 1'b0;

  case (st)
  F_IDLE: begin
      DCLK <= 1'b0; DATAOUT <= 1'b0; FLASH_NCE <= 1'b1;
      if (req_s != req_seen) begin
        req_seen <= req_s;
        r_op <= op;
        sect <= 6'd0;
        if (op == 2'd1) begin addr <= {page[15:8], 16'h0000}; st <= F_WREN; end
        else if (op == 2'd2) begin addr <= {page, 8'h00}; st <= F_WREN; end
        else if (op == 2'd3) begin addr <= {page, 8'h00}; st <= F_CMD;  end
        else begin done_t <= ~done_t; end          // unknown op: just ack
      end
    end
  //---- write enable -----------------------------------------------
  F_WREN: begin
      FLASH_NCE <= 1'b0; sh_tx <= 8'h06; ret <= F_WREN_E; st <= F_BYTE;
    end
  F_WREN_E: begin
      FLASH_NCE <= 1'b1; gap_ret <= F_CMD; st <= F_GAP;
    end
  //---- command + 24-bit address ----------------------------------
  F_CMD: begin
      FLASH_NCE <= 1'b0;
      sh_tx <= (r_op == 2'd1) ? 8'hD8 : (r_op == 2'd2) ? 8'h02 : 8'h03;
      ret <= F_A2; st <= F_BYTE;
    end
  F_A2: begin sh_tx <= addr[23:16]; ret <= F_A1; st <= F_BYTE; end
  F_A1: begin sh_tx <= addr[15:8];  ret <= F_A0; st <= F_BYTE; end
  F_A0: begin
      sh_tx <= addr[7:0]; st <= F_BYTE; bcnt <= 9'd0;
      ret <= (r_op == 2'd1) ? F_END : (r_op == 2'd2) ? F_DLOAD : F_RDATA;
      wr_raddr <= 8'd0;
    end
  //---- program data (256 bytes from RAM) -------------------------
  F_DLOAD:  st <= F_DLOAD2;                    // RAM latency
  F_DLOAD2: begin
      sh_tx <= rev8(wr_q);                     // RBF byte, LSB first on the wire
      wr_raddr <= wr_raddr + 1'd1;
      bcnt <= bcnt + 1'd1;
      ret <= (bcnt == 9'd255) ? F_END : F_DLOAD;
      st <= F_BYTE;
    end
  //---- read data (256 bytes to RAM) ------------------------------
  F_RDATA: begin
      if (bcnt != 9'd0) begin                  // store previous byte
        rd_we    <= 1'b1;
        rd_waddr <= bcnt[7:0] - 8'd1;
        rd_wdata <= rev8(sh_rx);
      end
      if (bcnt == 9'd256) st <= F_END;
      else begin
        sh_tx <= 8'h00; bcnt <= bcnt + 1'd1; ret <= F_RDATA; st <= F_BYTE;
      end
    end
  //---- end of command: nCS high (starts erase / program) ---------
  F_END: begin
      FLASH_NCE <= 1'b1;
      gap_ret <= (r_op == 2'd3) ? F_DONE : F_POLL;
      st <= F_GAP;
    end
  //---- poll status register until WIP = 0 ------------------------
  F_POLL: begin
      FLASH_NCE <= 1'b0; sh_tx <= 8'h05; ret <= F_POLL_R; st <= F_BYTE;
    end
  F_POLL_R: begin
      sh_tx <= 8'h00; ret <= F_POLL_E; st <= F_BYTE;
    end
  F_POLL_E: begin
      FLASH_NCE <= 1'b1;
      gap_ret <= sh_rx[0] ? F_POLL : F_NEXT;   // WIP bit
      st <= F_GAP;
    end
  F_NEXT:
      if (r_op == 2'd1 && sect != 6'd31) begin  // next sector of the slot
        sect <= sect + 1'd1;
        addr[23:16] <= addr[23:16] + 1'd1;
        st <= F_WREN;
      end
      else st <= F_DONE;
  //
  F_DONE: begin
      done_t <= ~done_t;
      st <= F_IDLE;
    end
  //---- byte engine: 8 bits, 3 clocks per bit ----------------------
  F_BYTE:
      case (ph)
      2'd0: begin DATAOUT <= sh_tx[7]; ph <= 2'd1; end
      2'd1: begin DCLK <= 1'b1; sh_rx <= {sh_rx[6:0], din_r}; ph <= 2'd2; end
      2'd2: begin
              DCLK <= 1'b0; sh_tx <= {sh_tx[6:0], 1'b0}; ph <= 2'd0;
              bitn <= bitn + 1'd1;
              if (bitn == 3'd7) st <= ret;
            end
      default: ph <= 2'd0;
      endcase
  //---- nCS high gap: 3 clocks + the setting clock = 320 ns -------
  F_GAP:
      if (gap == 3'd3) begin gap <= 3'd0; st <= gap_ret; end
      else gap <= gap + 1'd1;
  //
  default: st <= F_IDLE;
  endcase
end

endmodule
