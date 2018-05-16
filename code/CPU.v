
module CPU(reset, clk);
	input reset, clk;
	
	reg [31:0] PC;
	wire [31:0] PC_next;
	always @(posedge reset or posedge clk)
		if (reset)
			PC <= 32'h00000000;
		else
			PC <= PC_next;
	
	wire [31:0] PC_plus_4;
	assign PC_plus_4 = PC + 32'd4;
	
	wire [31:0] Instruction;
	InstructionMemory instruction_memory1(.Address(PC), .Instruction(Instruction));
	
	wire [1:0] RegDst;
	wire [1:0] PCSrc;
	wire Branch;
	wire MemRead;
	wire [1:0] MemtoReg;
	wire [3:0] ALUOp;
	wire ExtOp;
	wire LuOp;
	wire MemWrite;
	wire ALUSrc1;
	wire ALUSrc2;
	wire RegWrite;
	//用来根据opcode,funct,多个控制信号进行控制的module
	Control control1(
		.OpCode(Instruction[31:26]), .Funct(Instruction[5:0]),
		.PCSrc(PCSrc), .Branch(Branch), .RegWrite(RegWrite), .RegDst(RegDst), 
		.MemRead(MemRead),	.MemWrite(MemWrite), .MemtoReg(MemtoReg),
		.ALUSrc1(ALUSrc1), .ALUSrc2(ALUSrc2), .ExtOp(ExtOp), .LuOp(LuOp),	.ALUOp(ALUOp));
	
	wire [31:0] Databus1, Databus2, Databus3;//数据总线
	wire [4:0] Write_register;//要写进哪个寄存器
	assign Write_register = (RegDst == 2'b00)? Instruction[20:16]: (RegDst == 2'b01)? Instruction[15:11]: 5'b11111;//由RegDst控制的MUX
	//寄存器堆模块
	RegisterFile register_file1(.reset(reset), .clk(clk), .RegWrite(RegWrite), 
		.Read_register1(Instruction[25:21]), .Read_register2(Instruction[20:16]), .Write_register(Write_register),
		.Write_data(Databus3), .Read_data1(Databus1), .Read_data2(Databus2));
	
	wire [31:0] Ext_out;

	assign Ext_out = {ExtOp? {16{Instruction[15]}}: 16'h0000, Instruction[15:0]};//Ext_op控制的MUX
	
	wire [31:0] LU_out;
	assign LU_out = LuOp? {Instruction[15:0], 16'h0000}: Ext_out;//Lu_op控制的MUX
	
	wire [4:0] ALUCtl;//寄存器堆之后的ALU的控制信号
	wire Sign;
	ALUControl alu_control1(.ALUOp(ALUOp), .Funct(Instruction[5:0]), .ALUCtl(ALUCtl), .Sign(Sign));
	//ALU控制器
	
	wire [31:0] ALU_in1;
	wire [31:0] ALU_in2;
	wire [31:0] ALU_out;
	wire Zero;
	assign ALU_in1 = ALUSrc1? {17'h00000, Instruction[10:6]}: Databus1;//AS1控制的MUX
	assign ALU_in2 = ALUSrc2? LU_out: Databus2;//AS2控制的MUX
	ALU alu1(.in1(ALU_in1), .in2(ALU_in2), .ALUCtl(ALUCtl), .Sign(Sign), .out(ALU_out), .zero(Zero));
	
	wire [31:0] Read_data;//mem的read输出
	DataMemory data_memory1(.reset(reset), .clk(clk), .Address(ALU_out), .Write_data(Databus2), .Read_data(Read_data), .MemRead(MemRead), .MemWrite(MemWrite));
	//被控制信号控制的存储器模块
	assign Databus3 = (MemtoReg == 2'b00)? ALU_out: (MemtoReg == 2'b01)? Read_data: PC_plus_4;
	//DB3是用来写入寄存器的
	wire [31:0] Jump_target;
	assign Jump_target = {PC_plus_4[31:28], Instruction[25:0], 2'b00};//PC+4+偏移量
	
	wire [31:0] Branch_target;//分支目标
	assign Branch_target = (Branch & Zero)? PC_plus_4 + {LU_out[29:0], 2'b00}: PC_plus_4;//PC+4+偏移量
	
	assign PC_next = (PCSrc == 2'b00)? Branch_target: (PCSrc == 2'b01)? Jump_target: Databus1;

endmodule
	