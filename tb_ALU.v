`include "ctrl_encode_def.v"


module tb_ALU ;
reg  [31:0] A, B;
reg  [4:0]  ALUOp;
wire [31:0] C;
wire   Zero;

ALU UUT_D_ALU(.A(A), .B(B), .ALUOp(ALUOp), .C(C), .Zero(Zero));

initial 
begin
A = 32'h00000000;
B = 32'd00000000;
ALUOp  = 5'b00000;
end

initial 
begin
#100
ALUOp = `ALUOp_ADDU;
A = 32'h0000ffff;
B = 32'd00000001;#100

ALUOp = `ALUOp_SUBU;#100
ALUOp = `ALUOp_OR;#100
A = 32'h000000ff;
B = 32'h000000ff;
ALUOp = `ALUOp_SUBU;#100
A = 32'h00000000;
B = 32'h00000000;


end


endmodule

