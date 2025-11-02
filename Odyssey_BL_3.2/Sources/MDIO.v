

module mdio(
  //control
  input clock,
  input [4:0]addr,
  input rd_request,
  output ready,
  
  //data
  output reg [15:0] rd_data,  
  
  //hardware pins
  inout mdio_pin,
  output mdc_pin
  );


wire [63:0] rd_bits = {32'hFFFFFFFF, 9'b011000011, addr, 2'bxx, 16'hFFFF};
reg  [5:0] bit_no;
  

//state  
localparam ST_IDLE = 3'd1, ST_READING = 3'd2; 
reg[2:0] state = ST_IDLE; 


//3-state mdio pin
reg mdio_high_z;
assign mdio_pin = mdio_high_z ? 1'bz : rd_bits[bit_no];


//state machine  
always @(negedge clock)  
  case (state)
    ST_IDLE:
      begin
         mdio_high_z <= 0;
         bit_no <= 63; 
         if (rd_request) 
			   state <= ST_READING;
      end

    ST_READING:
      begin
         if (bit_no == 18) 
			   mdio_high_z <= 1;
         rd_data <= {rd_data[14:0], mdio_pin};
         if (bit_no == 1) 
			   state <= ST_IDLE;
         bit_no <= bit_no - 6'b1;
      end

    default
      state <= ST_IDLE;
  endcase  

  


//control 
assign mdc_pin = clock;
assign ready = (state == ST_IDLE);






endmodule
