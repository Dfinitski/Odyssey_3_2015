












module Band_decoder (

   input clock,
   input	run,
	input ptt,
   input	[14:0] rx_tune_phase,
   input	[14:0]tx_tune_phase,
   output reg [2:0]band
);


// RX phase count
//	P = F(Mhz) * 533.(3) 
	
wire [14:0]phase = ptt ? tx_tune_phase : rx_tune_phase; 

always @(posedge clock)
begin
   if(!run)
	   band = 3'd0;
	else 
	begin
	   if(phase<800) band <= 0;       // <1.5
		else if(phase<1333)  band <= 1; // <2.5
		else if(phase<2666)  band <= 2; // <5
		else if(phase<4266)  band <= 3; // <8
		else if(phase<8000)  band <= 4; // <15
		else if(phase<11733) band <= 5; // <22
		else if(phase<16533) band <= 6; // <31
		else band <= 7;                 // >31
	end
end
	
endmodule

