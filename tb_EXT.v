`timescale 1ns/1ps 
`include "ctrl_encode_def.v"

module tb_EXT ;

reg [15:0] Imm16;
reg [1:0]  EXTOp;

wire [31:0] Imm32;
EXT UUT_D_EXT( .Imm16(Imm16), .EXTOp(EXTOp), .Imm32(Imm32) );

initial
begin
EXTOp = 2'b00;
Imm16 = 16'h000f;
Imm16 = 16'hf000;
end


initial 
begin
#100
//signed
EXTOp = 2'b01;
Imm16 = 16'h000f;
Imm16 = 16'hf000;#100
//highpos
EXTOp = 2'b10;
Imm16 = 16'h000f;#100
Imm16 = 16'hf000;
end
endmodule



