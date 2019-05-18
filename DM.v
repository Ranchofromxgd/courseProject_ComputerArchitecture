module dm_4k( addr, din, DMWr, clk, dout );
   
   input  [31:0] addr;
   input  [31:0] din;
   input         DMWr;
   input         clk;
   output [31:0] dout;
     
   reg [31:0] dmem[1023:0];
   
   always @(posedge clk) begin
      if (DMWr)
         dmem[addr] <= din;
   end   // end always
   
   assign dout = dmem[addr[9:0]];
    
endmodule    
