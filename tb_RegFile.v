`timescale 1ns/1ps 
module tb_RegFile ;
reg [4:0] A1_test,A2_test,WriteReg_test;
reg [31:0] WD_test;
reg RorW_test,clk_test;
wire [31:0]RD1_test,RD2_test;

initial
begin
A1_test = 0;
A2_test = 0;
WriteReg_test = 0;
WD_test = 0;
RorW_test = 0;
clk_test = 0;
end

always #30 clk_test=~clk_test;

initial
begin
#30
WD_test = 256;
WriteReg_test = 10;
RorW_test = 1;
#30
WD_test = 65535;
WriteReg_test = 11;
RorW_test = 1;
#30
A1_test = 10;
A2_test = 11;
#30
RorW_test = 0;
end


RegFile UUT_D_RegFile(.A1(A1_test),.A2(A2_test),.WriteReg(WriteReg_test),.WD(WD_test),.RorW(RorW_test),.clk(clk_test),.RD1(RD1_test),.RD2(RD2_test));

endmodule



