//
/*

Commands:
1 - send FW version 7 bytes and BL mode request
2 -  receive 4 bytes of IP, 1 Slot, 1 mode
3 -  send IP to save, 4 bytes
4 -  send reset command to PIC
5 - receive reconfigure command

*/

module pic_control (
   input run,
   input clock,  // 80 kHz
	//
	input _write_ip,
	input [31:0] ip_to_write,
	//
   input u_rx,
	output u_tx,
	//
	output reg [31:0] IP,
	output reg [47:0] MAC,
	output reg [23:0] start_addr,
	output reg addr_ready = 0,
	output reg bl_mode = 0,
	output reg reconfig_req = 0,
	input bl_req,
	input _reset_req,
	//
	input _random_bit

);

parameter [23:0] fw_version = "0.0";


wire write_ip;
	cdc_sync #(1)
	w_ip (.siga(_write_ip), .rstb(0), .clkb(clock), .sigb(write_ip));
	
wire random_bit;
	cdc_sync #(1)
	r_bt (.siga(_random_bit), .rstb(0), .clkb(clock), .sigb(random_bit));
	
wire reset_req;
	cdc_sync #(1)
	r_rs (.siga(_reset_req), .rstb(0), .clkb(clock), .sigb(reset_req));
	
	
//UART interface ---------------------------------------------------
wire rx_ready;
wire [7:0] rx_byte;
   uart_rx rx_uart(clock, u_rx, rx_ready, rx_byte);

reg tx_start = 0;
wire tx_active, tx_done; 
reg [7:0] tx_byte;
   uart_tx tx_uart(clock, tx_start, tx_byte, tx_active, u_tx, tx_done);
//-------------------------------------------------------------------


//Main state mashine
reg [5:0] state = 0;
reg [5:0] return_state;
reg [7:0] data [0:8];
reg [4:0] byte_cnt;
reg [23:0] delay_cnt = 0;
reg write_ip_old = 0;


always @(posedge clock) 
begin
   case (state)
	
	0: if(run)
	      state <= 6'd1;
	   	
	1:  begin  // send FW version and mode request
		   data[8] <= 1'd1;  
		   data[7] <= " ";
			data[6] <= " ";
			data[5] <= " ";
			data[4] <= " ";	
		   data[3] <= fw_version[23:16];
         data[2] <= fw_version[15:8];
		   data[1] <= fw_version[7:0];
			data[0] <= bl_req ? 8'd1 : 8'd0;
			byte_cnt <= 4'd9;
			return_state <= 6'd2; 
			state <= 6'd20;   
	   end	
			
	2: if(rx_ready & rx_byte==2)
	   begin
		   byte_cnt <= 4'd6;
			return_state <= 6'd3;
			state <= 6'd25;
		end
		
	3: begin
         IP[31:24] <= data[5];
			IP[23:16] <= data[4];
			IP[15:8]  <= data[3];
			IP[7:0]   <= data[2];
			start_addr[23-:8] <= data[1] * 8'd32;
		   start_addr[15:0] <= 16'd0;
			state <= 6'd5;
         // MAC preparing
		   MAC[47:16] <= 32'h00_04_A3_00;
			byte_cnt <= 1'd0;
	      state <= 6'd4; 
		end	
		
   4: if(byte_cnt<16)
      begin
	      MAC[byte_cnt] <= random_bit;
			byte_cnt <= byte_cnt + 1'd1;
      end
	   else
		begin
	      bl_mode <= data[0]>0 ? 1'd1 : 1'd0;
			addr_ready <= 1'd1;
			state <= 6'd5;
		end
			
   5: if(write_ip!=write_ip_old)  // waiting for write IP to PIC command 3
      begin
		   write_ip_old <= write_ip;
		   data[4] <= 8'd3;
         data[3] <= ip_to_write[31:24];			
		   data[2] <= ip_to_write[23:16];
         data[1] <= ip_to_write[15:8];
		   data[0] <= ip_to_write[7:0];
			IP <= ip_to_write;
			byte_cnt <= 4'd5;
			return_state <= 6'd5; 
			state <= 6'd20; 
		end
		else if(reset_req) // waiting for send reset command 4
		begin
		   data[0] <= 8'd4;
			byte_cnt <= 1'd1;
			return_state <= 6'd5; 
			state <= 6'd20;
		end
		else if(rx_ready & rx_byte==5)  // waiting for reconfig request command 5
			reconfig_req <= 1'd1;

		
  //    Sub routines
  
  // Receive data		
  25: if(rx_ready)
      begin
         data[byte_cnt-1'd1] <= rx_byte;
			byte_cnt <= byte_cnt-1'd1;
         state <= 6'd26;			
      end
		
  26: if(byte_cnt>0)
         state <= 6'd25;
		else
		   state <= return_state;
     
  
  // send data
  20: begin
		   tx_byte <= data[byte_cnt-1'd1];	
		   state <= 6'd21;	
		end
		
  21: begin
         tx_start <= 1'd1;
         if(tx_active)
			begin
			   byte_cnt <= byte_cnt - 1'd1;
				tx_start <= 1'd0;
            state <= 6'd22;
			end
		end	
		
  22: if(tx_done) 
      begin
         if(byte_cnt>0)
			   state <= 6'd20;
			else
			   state <= return_state;
		end	
		
 
  // Delay routine	
  30: if(delay_cnt != 0) delay_cnt <= delay_cnt - 1'd1; // delay
      else state <= return_state; 
	
			
	//
	default: state <= 6'd0;
	endcase

end

//
endmodule

