`timescale 1ns / 1ps

//A1:register1 to read,A2:register2 to read,WriteReg:register to write
//WD:the data we need to write
//RorW:choose if write enabled
//clk:clock signal ,reset:set all registers to 0
//RD1 ,RD2: the output data from register file

module RegFile (A1, A2, WriteReg, WD, RorW, clk,RD1, RD2);
	input [4:0]	A1, A2, WriteReg;
	input [31:0] WD;
	input	RorW, clk;
	output [31:0] RD1, RD2;	//
	reg [31:0] register [0:31]; //32 registers totally
	integer i;

	assign RD1 = (A1 == 0) ? 0 : register[A1];
	assign RD2 = (A2 == 0) ? 0 : register[A2];

	always @(posedge clk) begin
		begin

			if (WriteReg != 0 && RorW== 1)
				register[WriteReg] <= WD;
		end
	end
endmodule