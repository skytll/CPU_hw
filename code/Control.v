
module Control(OpCode, Funct,
	PCSrc, Branch, RegWrite, RegDst, 
	MemRead, MemWrite, MemtoReg, 
	ALUSrc1, ALUSrc2, ExtOp, LuOp, ALUOp);
	input [5:0] OpCode;
	input [5:0] Funct;
	output [1:0] PCSrc;
	output Branch;
	output RegWrite;
	output [1:0] RegDst;
	output MemRead;
	output MemWrite;
	output [1:0] MemtoReg;
	output ALUSrc1;
	output ALUSrc2;
	output ExtOp;
	output LuOp;
	output [3:0] ALUOp;
	
	// Your code below
	assign PCSrc = 
		(OpCode == 6'h02)? 2'b01:
		(OpCode == 6'h03)? 2'b01:
		((Funct == 6'h08) && (OpCode == 6'h0))? 2'b10:
		((Funct == 6'h09) && (OpCode == 6'h0))? 2'b10:
		2'b00;
	assign Branch = 
		(OpCode == 6'h04)? 1:
		0;
	assign RegWrite = 
		(OpCode == 6'h2b)? 0:
		(OpCode == 6'h04)? 0:
		(OpCode == 6'h02)? 0:
		((Funct == 6'h08) && (OpCode == 6'h0))? 0:
		1;
	assign RegDst = 
		(OpCode == 6'h03)? 2'b10:
		(OpCode == 6'h23)? 2'b00:
		(OpCode == 6'h2b)? 2'b00:
		(OpCode == 6'h0f)? 2'b00:
		(OpCode == 6'h08)? 2'b00:
		(OpCode == 6'h09)? 2'b00:
		(OpCode == 6'h0c)? 2'b00:
		(OpCode == 6'h0a)? 2'b00:
		(OpCode == 6'h0b)? 2'b00:
		(OpCode == 6'h04)? 2'b00:
		2'b01;
	assign MemRead = 
		(OpCode == 6'h23)? 1:
		0;
	assign MemWrite = 
		(OpCode == 6'h2b)? 1:
		0;
	assign MemtoReg = 
		(OpCode == 6'h23)? 2'b01:
		(OpCode == 6'h03)? 2'b10:
		((Funct == 6'h09) && (OpCode == 6'h0))? 2'b10:
		2'b00;
	assign ALUSrc1 = 
		((Funct == 6'h00) && (OpCode == 6'h0))? 1:
		((Funct == 6'h02) && (OpCode == 6'h0))? 1:
		((Funct == 6'h03) && (OpCode == 6'h0))? 1:
		0;
	assign ALUSrc2 = 
		(OpCode == 6'h23)? 1:
		(OpCode == 6'h2b)? 1:
		(OpCode == 6'h0f)? 1:
		(OpCode == 6'h08)? 1:
		(OpCode == 6'h09)? 1:
		(OpCode == 6'h0c)? 1:
		(OpCode == 6'h0a)? 1:
		(OpCode == 6'h0b)? 1:
		0;
	assign ExtOp = 
		(OpCode == 6'h23)? 1:
		(OpCode == 6'h2b)? 1:
		(OpCode == 6'h08)? 1:
		//(OpCode == 6'h0c)? 1:
		(OpCode == 6'h0a)? 1:
		(OpCode == 6'h04)? 1:
		0;
	assign LuOp = 
		(OpCode == 6'h0f)? 1:
		0;
	// Your code above
	
	assign ALUOp[2:0] = 
		(OpCode == 6'h00)? 3'b010: 
		(OpCode == 6'h04)? 3'b001: 
		(OpCode == 6'h0c)? 3'b100: 
		(OpCode == 6'h0a || OpCode == 6'h0b)? 3'b101: 
		3'b000;
		
	assign ALUOp[3] = OpCode[0];
	
endmodule