module im_4k( addr, dout );
    
    input [29:0] addr;
    output [31:0] dout;
    
    reg [31:0] mem[1023:0];
    
    assign dout = mem[addr[9:0]];
    
endmodule    
