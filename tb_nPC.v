`timescale 1ns/1ps 
module tb_NPC_PC ;
reg  [1:0]  NPCOp; // PLUS4(2'b00), BRANCH(2'b01), JUMP(2'b10)  
reg  [25:0] IMM; 
wire [31:2] NPC; 

wire [31:2] PC_out;
reg PCWr,rst,clk;



NPC UUT_D_NPC(.PC(PC_out), .NPCOp(NPCOp), .IMM(IMM), .NPC(NPC) );
PC UUT_D_PC(.clk(clk), .rst(rst), .PCWr(PCWr), .NPC(NPC), .PC(PC_out));


initial
begin
PCWr = 0;
rst = 0; 
clk = 0;
NPCOp = 2'b00;
IMM = 0;
end

always #100 clk=~clk;

initial 
begin
#2
//RESET
rst = 1;#100
rst = 0;
PCWr = 1;#300
//BRANCH
NPCOp = 2'b01;
IMM[3:0] = 8'b11111111;#100
//PLUS
NPCOp = 2'b00; #300
//JUMP
NPCOp = 2'b10;
IMM[15:0] = 16'hffff;#100
//PLUS
NPCOp = 2'b00; 
end
endmodule



