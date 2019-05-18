`timescale 1ns / 1ps
module NPC( PC, NPCOp, IMM, NPC );
    
   input  wire [31:2] PC;
   input  [1:0]  NPCOp;
   input  [25:0] IMM;
   output [31:2] NPC;
   
   reg [31:2] NPC;
   
   always @(*) begin
      case (NPCOp)
          2'b00: NPC = PC + 1;
	    //Signed extension
          2'b01: NPC = PC + {{14{IMM[15]}}, IMM[15:0]};
          2'b10: NPC = {PC[31:28], IMM[25:0]};
          default: ;

      endcase
   end
   
endmodule
