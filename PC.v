`timescale 1ns / 1ps

module PC( clk, rst, PCWr, NPC, PC );              
input         clk;   
input         rst; //reset pc to 32'h0000_3000  
input         PCWr;   
input  [31:2]  NPC;   
output reg [31:2]  PC;

always @(posedge clk) 
	begin
		if(rst == 1)
			PC <= 32'h00003000;
		if (rst == 0 && PCWr== 1)
			PC <= NPC;
	end

endmodule
