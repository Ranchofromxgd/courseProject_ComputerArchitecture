module dm_test;
reg [31:0] address;
reg [31:0] data_in;
reg data_w, clock;
wire [31:0] data_out;
parameter TIME_PERIOD = 100;



dm_4k data_memory( .addr(address), .din(data_in), .DMWr(data_w), .clk(clock), .dout(data_out));


initial
begin
clock = 1'b0;
data_w = 1'b0;
address = 32'b0;
data_in = 32'haaaa_aaaa;

#50
data_w = 1'b1;

#100
data_in = 32'haaaa_0000;


#100
data_in = 32'hffff_abc0;
address = 32'h0000_0001;


#100
data_in = 32'hffff_ffff;
address = 32'h0000_0002;

#100
data_in = 32'h0000_ffff;
address = 32'h0000_0003;

#100
data_w = 1'b0;
address = 32'h0000_0002;
#30
address = 32'h0000_0001;
#30
address = 32'h0000_0000;
end   //end initial


always
begin
#(TIME_PERIOD/2) 
clock = ~clock; 
end


endmodule