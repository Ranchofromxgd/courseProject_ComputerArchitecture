`timescale 1ns/1ps 
module tb_PC;
reg 		clk_test;   
reg         rst_test; //reset pc to 32'h0000_3000  
reg         PCWr_test;   
reg  [31:2]  NPC_test;   
wire [31:2]  PC_test;

initial
begin
rst_test = 0;
clk_test = 0;
PCWr_test = 0;
NPC_test = 0;
end

always #30 clk_test=~clk_test;

initial
begin
#60
rst_test = 1;
#60
rst_test = 0;
#60
NPC_test = 255;
PCWr_test = 1;
#60
PCWr_test = 0;
end

PC UUT_D_PC(.rst(rst_test),.clk(clk_test),.PCWr(PCWr_test),.NPC(NPC_test),.PC(PC_test));

endmodule




