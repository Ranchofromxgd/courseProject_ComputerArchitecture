module mux_test;

reg  [7:0] A, B, C, D;
reg  [31:0] E, F, G, H,I,J,K,L;
//Select signal
reg   S1;
reg  [1:0] S2;
reg  [2:0] S3;
//Output 
wire [7:0] y1;
wire [31:0] y3_4,y3_8;


mux2 #(8) UUT_D_MUX_2_8 (.d0(A), .d1(B),.s(S1), .y(y1));

mux4 #(32) UUT_D_MUX_4_32 (.d0(E), .d1(F), .d2(G), .d3(H),.s(S2), .y(y3_4));

mux8 #(32) UUT_D_MUX_8_32 (.d0(E), .d1(F), .d2(G), .d3(H),.d4(I), .d5(J), .d6(K), .d7(L),.s(S3), .y(y3_8));

initial
begin
A = 8'h1f;
B = 8'h2f;
C = 8'h3f;
D = 8'h4f;

E = 32'hffff1111;
F = 32'hffff2222;
G = 32'hffff3333;
H = 32'hffff4444;
I = 32'hffff5555;
J = 32'hffff6666;
K = 32'hffff7777;
L = 32'hffff8888;

S1 = 1'b0;
S2 = 2'b00;
S3 = 3'b000;
end

initial 
begin #100
S1 = 1'b1;#100
S1 = 1'b0;#100
S2 = 2'b01;#100
S2 = 2'b10;#100
S2 = 2'b11;#100
S3 = 3'b000;#100
S3 = 3'b010;#100
S3 = 3'b100;
end


endmodule