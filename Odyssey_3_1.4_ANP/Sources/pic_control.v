//
/*

Commands:
1 - send FW version 7 bytes and one 0 byte
2 -  receive 4 bytes of IP, 0 byte and 0 byte
6 - receive 2 bytes AIN1
7 - receive 2 bytes AIN2

*/

module pic_control (
   input run,
   input clock,  // 80 kHz
	//
   input u_rx,
	output u_tx,
	//
	output reg [31:0] IP,
	output reg [47:0] MAC,
	output reg init_done = 0,
	output reg [11:0] AIN1 = 0,
	output reg [11:0] AIN2 = 0,
	//
	input _random_bit

);

parameter [55:0] fw_version = "0.0 ANP";
	
wire random_bit;
	cdc_sync #(1)
	r_bt (.siga(_random_bit), .rstb(0), .clkb(clock), .sigb(random_bit));
	
	
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
reg [15:0] buffer;



always @(posedge clock) 
begin
   case (state)
	
	0: if(run) // working one time after power on
	      state <= 6'd1;
	   	
	1:  begin  // send FW version and mode request
		   data[8] <= 1'd1;  
		   data[7] <= fw_version[55:48];
			data[6] <= fw_version[47:40];
			data[5] <= fw_version[39:32];
			data[4] <= fw_version[31:24];	
		   data[3] <= fw_version[23:16];
         data[2] <= fw_version[15:8];
		   data[1] <= fw_version[7:0];
			data[0] <= 8'd0;
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
		   init_done <= 1'd1;
			state <= 6'd5;
		end
			
   5: if(rx_ready & rx_byte==6)  // waiting for telemetry from PIC
	   begin
			byte_cnt <= 4'd2;
			return_state <= 6'd6;
			state <= 6'd25;
		end
		else if(rx_ready & rx_byte==7) 
		begin
			byte_cnt <= 4'd2;
			return_state <= 6'd7;
			state <= 6'd25;
		end
		
   6: begin
	      AIN1[11:10] <=    data[1] & 8'b00000011; 
			AIN1[9:8] <= (data[0]>>6) & 8'b00000011;
			AIN1[7:0] <=   data[0]<<2 & 8'b11111100;
			state <= 6'd5;
	   end
		
	7: begin
	      AIN2[11:10] <=    data[1] & 8'b00000011; 
			AIN2[9:8] <= (data[0]>>6) & 8'b00000011;
			AIN2[7:0] <=   data[0]<<2 & 8'b11111100;
			state <= 6'd5;
	   end
		
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

