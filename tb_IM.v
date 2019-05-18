 module tb_IM();
    
   reg [31:0]address;
   wire[31:0]out;
    
   im_4k UUT_D_ALU_IM( .addr(address[31:2]), .dout(out) );
   
    
   initial begin
      $display("Readfile!");
      $readmemh( "./test.txt" , UUT_D_ALU_IM.mem ) ;
      address = 32'h000000000;
      while(1)
      begin
      #100
      address = address + 4;
      end
   end

   
endmodule