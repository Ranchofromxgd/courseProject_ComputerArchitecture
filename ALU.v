`include "ctrl_encode_def.v"
module ALU (A, B, ALUOp, C, Zero);        
   input  [31:0] A, B;
   input  [4:0]  ALUOp;
   output reg [31:0] C;
   output        Zero;
  
   always @(*) begin
      case ( ALUOp )
		//ADD
         `ALUOp_ADDU: C = A + B;
		//SUB
         `ALUOp_SUBU: C = A - B;
		//OR
         `ALUOp_OR:   C = A | B;
         default:   ;
      endcase
   end 
   
   assign Zero = (A == B) ? 1 : 0;

endmodule
    

