/*
 *  File: Project.v
 *  Author:	Kameron Noorbakhsh
 *	Modules:
 *			Clock			  - A module which periodically changes the output signal.
			TestBench  		  - A module that will test how all of the modules work together
			Adder             - A module that will add 4 to the input address
			ProgramCounter    - A module which will increment the address by one on the clocks posedge and negedge
			InstructionMemory - A module that reads instructions from a .txt file and will return an instruction based on the address passed through
			Control	     	  - A module that takes an OPcode and will do the correct operation accordingly
			ALUControl		  - A module that takes an OPcode as well as a funct code and determines the operation to be done
			ALU				  - A module that will perform a specific action to two inputs and outputs, the action is based on user input
			DataMemory		  - A module that either writes data to a memory based on an address, or outputs data of a memory based on an address
			SignExtender	  - A module that will convert a 16 bit wire to 32 bits
			Registers		  - A module that outputs data from a memory based off two inputed registers, will also write registers to memory if notified
			MUXone	 		  - A module that choose the output from two 32 bit inputs based off a selector
			MUXtwo	  		  - A module that choose the output from two 5 bit inputs based off a selector
 */
 
 /*
  *  Module:	Clock		- A module which periodically changes the output signal.
 *  Author:	Dr. Richard A. Goodrum, Ph.D.
 *	Ports:
 *		clock - O/P	wire	   A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI.
 */
module Clock( clock );		    // Establish the name of the module and ports.
/*
 *	LO represents the number of ticks during which the output will be low.
 *  HI represents the number of ticks during which the output will be high.
 */
	parameter LO = 10, HI = 10;
	output reg clock;			// Establish a storage location for the oscillating output signal.

	initial						// Loop structure which executes only once during start-up.
		clock = 0;				// clock is intended to start as a low value.

	always						// Loop structure which executes continuously after start-up.
		begin					// Only a code block within the loop.
			#LO	clock = ~clock;	// Toggle the value of clock after LO ticks.
			#HI	clock = ~clock;	// Toggle the value of clock after HI ticks.
		end						// Close the code block within the loop.

endmodule						// End of code for this module.

 /*
  *  Module:	TestBench		- A module that will test how all of the modules work together
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		address  - I/P & O/P wire     A 32 bit which holds the place of the next instruction to be executed
		inputPC  - I/P & O/P wire     A 32 bit that is the output of the adder then inputted into the Program Counter
		outputIM - O/P wire           A 32 bit that is the instruction to be executed out of the Instruction Memory
		clock    - I/P wire			  A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI.				 
 */
module TestBench;											// Establish the name of the module and ports.
/*	initial                         						// This section of code was used for testing through gtkwave to help debug
	begin
		$dumpfile("Program1.vcd");							// Created the dump file 
		$dumpvars(0,TestBench); 							// Dumped the program into the created file
	end */


	wire [31:0] PCout, MUX1out, MUX2out, MUX3out, MUX6out, MUX7out, MUX8out, MUX9out, IMout, add1OUT, add2OUT, signextendOUT, ALUresult, ReadData1, ReadData2, ReadData;		// Instantiated the 32 bit wires 
	wire [3:0] ALUOp;
	wire clock,  regWrite, zero, RegDst, Jump, Jump_al, Branch_eq, Branch_ne, Memread, MemtoReg, Memwrite, ALUSrc, RegWrite, LUIcontrol, shift, jr;
	wire [4:0] ALUx, readReg1, readReg2, writeReg, ALUcontrol, MUX4out, MUX5out, ALUcontOUT;
	
	ProgramCounter pc (PCout, MUX1out, clock);
	Adder adder1(add1OUT, PCout, 4);
	Adder adder2(add2OUT, add1OUT, (signextendOUT << 2));
	MUXone mux2(add1OUT, add2OUT, ((Branch_eq & zero) | (Branch_ne & ~zero)), MUX2out);
	MUXone mux3(MUX2out, {{(IMout[27:0] << 2), {PCout[3:0]}}}, Jump, MUX3out);
	MUXone mux1(MUX3out, ALUresult, jr, MUX1out);
	InstructionMemory im(IMout, PCout, clock);
	Control cont(IMout[31:26], RegDst, Jump, Jump_al, Branch_eq, Branch_ne, Memread, MemtoReg, ALUOp, Memwrite, ALUSrc, RegWrite, LUIcontrol);
	Registers regs(IMout[25:21], IMout[20:16], MUX4out, MUX6out, RegWrite, ReadData1, ReadData2, clock);
	MUXtwo mux4(MUX5out, 5'b11111, Jump_al, MUX4out);
	MUXtwo mux5(IMout[20:16], IMout[15:11], RegDst, MUX5out);
	MUXone mux6(MUX9out, add1OUT, Jump_al, MUX6out);
	SignExtender sign(IMout[15:0], signextendOUT);
	ALUControl ALUcont(ALUcontOUT, ALUOp, IMout[5:0], shift, jr);
	MUXone mux7(ReadData1, signextendOUT, shift, MUX7out);
	MUXone mux8 (ReadData2, signextendOUT, ALUSrc, MUX8out);
	ALU alu(MUX7out, MUX8out, ALUcontrol, ALUresult, zero, clock, regs.RF[2], regs.RF[4]);
	DataMemory mem(ALUresult, ReadData2, ReadData, Memwrite, Memread, clock);
	MUXone mux9(ALUresult, ReadData, MemtoReg, MUX9out);
	
	
	
	initial													// Loops upon start-up
		begin												// Only a code block within the loop
															// Establish an output that will show anytime any of these variables change
			pc.addex = 0;
//			$monitor($time); 
			$readmemb ("data.dat", mem.RF);			//12288
			$readmemb ("text.txt", im.Imem);
		end													// Close the block within this loop

endmodule												// End of module

 /*
  *  Module:	Control			- A module that takes an OPcode and will do the correct operation accordingly
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		opcode	 	- I/P 				A 6 bit that represents the operation to be done
		RegDst   	- O/P				A single bit that represents the register
		Jump	 	- O/P				A single bit that represents if the user is trying to jump
		Jump_al	 	- O/P				A single bit that represents if the user is trying to jump and link
		Branch_eq 	- O/P				A single bit that represents if the user wants to branch and equal
		Branch_ne	- O/P				A single bit that shows if the user wants to branch and not equal
		Memread		- O/P				A single bit that shows if the user wants to read from the memory
		MemtoReg	- O/P				A single bit that sees if the user wants to put memory in the register
		Memwrite	- O/P				A single bit that shows if the user wants to write to the memory
		ALUSrc		- O/P				A single bit that shows if the ALU provides source
		RegWrite	- O/P				A single bit that shows if the user wants to write to register
		ALUOp		- O/P				A 2 bit that when passed through ALU control shows the operation to be done
		LUIcontrol	- O/P				A single bit that shows if the user wants to load upper immediate
 */
module Control (opcode, RegDst, Jump, Jump_al, Branch_eq, Branch_ne, Memread, MemtoReg, ALUOp, Memwrite, ALUSrc, RegWrite, LUIcontrol);
	input [5:0] opcode;																										// Instantiate 6 bit
	output reg RegDst, Jump, Jump_al, Branch_eq, Branch_ne, Memread, MemtoReg, Memwrite, ALUSrc, RegWrite, LUIcontrol;		// Instantiate all 1 but outputs regs
	output reg [3:0] ALUOp;																									// Instantiate 2 bit
	
	always @(*) begin
		RegDst  	<= 1'b1;					// RegDst default is 1
		Jump		<= 1'b0;
		Jump_al 	<= 1'b0;
		Branch_eq   <= 1'b0;
		Branch_ne   <= 1'b0;					// Set all default values for the outputIm
		Memread 	<= 1'b0;
		MemtoReg    <= 1'b0;
		Memwrite    <= 1'b0;
		ALUSrc		<= 1'b0;
		RegWrite	<= 1'b1;					// RegWrite default is 1
		ALUOp[3:0]  <= 4'b0000;					// ALUop default is 10
		LUIcontrol 	<= 1'b0;
		
		case (opcode)
			6'b000000: begin					// For any R-type arithmetic function (add)
					ALUOp <= 4'b1111;
			end
			6'b000010: begin					// For normal jump instruction
					Jump <= 1'b1;				// Change jump to hi
					ALUOp <= 4'b0000;
			end
			6'b000011: begin					// For jump and link instruction
					Jump_al <= 1'b1;			// Change Jump_al to hi
					ALUOp <= 4'b0001;
			end
			6'b000100: begin					// For Branch and equal instruction
					ALUOp <= 4'b0010;
					Branch_eq <= 1'b1;			// Set Branch_eq to hi
					RegWrite  <= 1'b0;			// Set Regwrite to lo
			end
			6'b000101: begin					// For Branch and not equal instruction
					ALUOp <= 4'b0011;
					Branch_ne <= 1'b1;			// Set Branch_ne to hi
					RegWrite  <= 1'b0;			// Set RegWrite to lo
			end
			6'b001111: begin					// For Load Upper Immediate instruction
					LUIcontrol <= 1'b1;			// Set LUIcontrol to hi
					ALUOp <= 4'b0100;
			end	
			6'b100011: begin					// For Load Word instruction
					Memread		<= 1'b1;		// Set Memread to hi
					RegDst		<= 1'b0;		// Set RegDst to lo
					MemtoReg	<= 1'b1;		// Set MemtoReg to hi
					ALUOp 		<= 4'b0111;		
					ALUSrc		<= 1'b1;		// Set ALUSrc to hi
			end
			6'b101011: begin					// For Store Word instruction
					Memwrite	<= 1'b1;		// Set Memwrite to hi
					ALUOp 		<= 4'b1000;
					ALUSrc		<= 1'b1;		// Set ALUSrc to hi
					RegWrite	<= 1'b0;		// Set RegWrite to lo
			end
			6'b011100: begin
					ALUOp 		<= 4'b1010;		// For mul
			end
			default:   begin
					RegDst  	<= 1'b1;					// RegDst default is 1
					Jump		<= 1'b0;
					Jump_al 	<= 1'b0;
					Branch_eq   <= 1'b0;
					Branch_ne   <= 1'b0;					// Set all default values for the outputIm
					Memread 	<= 1'b0;
					MemtoReg    <= 1'b0;
					Memwrite    <= 1'b0;
					ALUSrc		<= 1'b0;
					RegWrite	<= 1'b1;					// RegWrite default is 1
					ALUOp 		<= 4'b1001;				// ALUop default is 10
					LUIcontrol 	<= 1'b0;
			end
		endcase
	end
endmodule

 /*
  *  Module:	ALUControl				- A module that takes an OPcode as well as a funct code and determines the operation to be done
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		ALUop	 	- I/P 				A 4 bit that represents the function assigned from control
		funct 		- I/P				A 6 bit that holds data other than 0 as long as the instruction is an R-type
		ALUx   		- O/P				A 5 bit output that is assigned a value for each instruction
 */
module ALUControl (ALUx, ALUop, funct, shift, jr);
	
	output reg jr;
	output reg shift; 

	output reg[4:0]  ALUx;			// instantiate output
	input[3:0] ALUop;				// instantiate inputs 
	input[5:0] funct;
	
	always @(ALUop, funct) begin
		jr <= 1'b0;
		shift <=1'b0;
		case (ALUop)
			4'b1111:	begin
				case(funct)
					6'b100000: begin
						ALUx <= 5'b00000;		// add
					end
					6'b100100: begin
						ALUx <= 5'b00001;		// and
					end
					6'b011010: begin
						ALUx <= 5'b00010;		// div
					end
					6'b011000: begin
						ALUx <= 5'b00011;		// mult
					end
					6'b100101: begin
						ALUx <= 5'b00100;		// or
					end
					6'b000000: begin
						ALUx <= 5'b00101;		// sll
						shift <= 1'b1;
					end
					6'b101010: begin
						ALUx <= 5'b00110;		// slt
						shift <= 1'b1;
					end
					6'b000011: begin
						ALUx <= 5'b00111;		// sra
						shift <= 1'b1;
					end
					6'b000010: begin
						ALUx <= 5'b01000;		// srl
						shift <= 1'b1;
					end
					6'b100010: begin
						ALUx <= 5'b01001;		// sub
					end
					6'b001000: begin
						ALUx <= 5'b01010;		// jr
						jr <= 1'b1;
					end
					6'b010000: begin
						ALUx <= 5'b01011;		// mfhi
					end
					6'b010010: begin
						ALUx <= 5'b01100;		// mflo
					end
					6'b100110: begin
						ALUx <= 5'b01101;		// syscall
					end
					default:	begin
						ALUx <= 5'b01110;
					end
				endcase
			end
			4'b0000: begin
				ALUx <= 5'b01111;				// jump
			end
			4'b0001: begin
				ALUx <= 5'b10000;				// jump and link
			end
			4'b0010: begin
				ALUx <= 5'b10001;				// branch and equal
			end
			4'b0011: begin
				ALUx <= 5'b10010;				// branch and not equal
			end
			4'b0100: begin
				ALUx <= 5'b10011;				// load upper immediate
			end
			4'b0111: begin
				ALUx <= 5'b10100;				// load word
			end
			4'b1000: begin
				ALUx <= 5'b10101;				// store word
			end
			4'b1010: begin
				ALUx <= 5'b10110;				// mul
			end
			default: begin
				ALUx <= 5'b11111;
			end
		endcase
	end
 endmodule

module HILO(clock);

	input clock;
	
	reg [31:0] HI;
	reg [31:0] LO;
endmodule
	
 /*
  *  Module:	ALU				  - A module that will perform a specific action to two inputs and outputs, the action is based on user input
 *  Author:	Dr. Richard Goodrum
 *	Ports:
 *		ALUcontrol  - I/P		A 4 bit with the instruction of what to do with inputs A and B
		A		    - I/P		A 32 bit with data to be modified and outputted
		B			- I/P		A 32 bit with data to be modified and outputted
		ALUresult	- O/P		A 32 bit reg that outputs the final result based on A and B
		zero		- O/P		A 1 bit reg that returns whether the final result is 0
 */
module ALU(A, B, ALUcontrol, ALUresult, zero, clock, v0, a0);
	input clock;
	input [31:0] v0;
	input [31:0] a0;
	HILO hilo(clock);
	input [4:0] ALUcontrol;						// Instantiate 4 bit control variable
	input [31:0] A, B;							// Instantiate 32 bit inputs
	output reg[31:0] ALUresult;					// Instantiate 32 bit output reg for final result
	output zero;								// Instantiate 1 bit output
	
	assign zero = (ALUresult == 0);				// If the final result is 0, zero is true
	
	always @(ALUcontrol, A, B) 					// When the ALU is called upon
	begin
		case (ALUcontrol)						// Begin case statement for ALUcontrol
			0: ALUresult <= A + B;				// ADD
			1: ALUresult <= A & B;				// AND
			2: begin							// DIV
				hilo.HI <= A % B;				
				hilo.LO <= A / B;
			end
			3:	hilo.LO <= A * B;				// MULT
			4:  ALUresult <= A | B;				// OR
			5: ALUresult <= A << B;				// SLL
			6: begin
				if ( A < B )
					ALUresult <= 1'b1;			// SLT
				else
					ALUresult <= 1'b0;
			end
			7: ALUresult <= A >>> B;			// SRA
			8: ALUresult <= A >> B;				// SRL
			9: ALUresult <= A - B;				// SUB
			10: ALUresult <= A;					// JR
			11: ALUresult <= hilo.HI;			// MFHI
			12: ALUresult <= hilo.LO;			// MFLO
			13: begin
				if (v0 == 1)
					$display ("%d", a0);
				if (v0 == 1)
					$display("%s", a0);
				if (v0 == 10)
					$finish;					// SYSCALL
			end
			15: ALUresult <= A;
			16: ALUresult <= A;
			17:	ALUresult <= A;
			18: ALUresult <= A;
			19: ALUresult <= A << 16;
			20: ALUresult <= A;
			21: ALUresult <= A;
			22: ALUresult <= A * B;
			default: ALUresult <= 0;			// Default ALUcontrol to 0 if an unsupported case is used
		endcase									// End case statement
	end
endmodule
 /*
  *  Module:	Adder		- A module that will add 4 to the input address
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		O - O/P wire			     A 32 bit that will hold the output of the inputs being added
		A - I/P 	     			 A 32 bit that will be added to the wire B
		B - I/P     	    		 A 32 bit that is always the decimal value 4	 
 */
module Adder(O, A, B);				// Establish the name of the module and ports.
	
	output[31:0] O;					// Establish a location for the output
	input[31:0]  A, B;				// Establish locations for both of the inputs
	
	wire O;							// Make the output into a wire to hold data
	assign O = A + B;				// Set O as the sum of the two inputs

endmodule							// End of module
 /*
  *  Module:	ProgramCounter	   - A module which will increment the address by one on the clocks posedge and negedge
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		address  - O/P reg     A 32 bit which holds the place of the next instruction to be executed
		inputPC  - I/P         A 32 bit that is the output of the adder then inputted into the Program Counter
		clock    - I/P         A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI. 
 */
module ProgramCounter (address, inputPC, clock);	// Establish the name of the module and ports
	output[31:0] address;							// Establish a location for the output address
	input[31:0] inputPC;							// Establish a location for the input inputPC
	input clock;									// Establish a location for the input clock
	reg[31:0] addex;
		
		assign address = addex;
		always @(posedge clock)
			addex = inputPC;
		
	
endmodule											// End of module

 /*
  *  Module:	InstructionMemory - A module that reads instructions from a .txt file and will return an instruction based on the address passed through
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		address  - I/P 		   A 32 bit which holds the place of the next instruction to be executed
		outputIM - O/P reg     A 32 bit that holds the instruction to be executed
		clock    - I/P         A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI. 
		memAddr  - reg		   A 32 bit that will hold the address
		Imem	 - reg memory  A 32 x 128 memory storage that will hold all of the instructions from a txt file
 */
module InstructionMemory (outputIM, address, clock);	// Begin module
	output[31:0] outputIM;								// Establish a location for the output that holds the instruction
	input[31:0] address;								// Establish a location for the input address
	input clock;										// Establish a location for the input clock
	
	
	//reg [31:0] outputIM;								// Make outputIM into a register to hold instructions
	
	reg [31:0] memAddr;									// Establish memAddr to hold the memory address
	reg [31:0] Imem[0:58];								// Establish a memory to hold all of the instructions
	
	assign outputIM = Imem[address[11:2]];
	
endmodule												// End of module
 /*
  *  Module:	DataMemory		  - A module that either writes data to a memory based on an address, or outputs data of a memory based on an address
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		address   - I/P  	   A 32 bit wire which holds the location of where to read/write in the memory
		writeData - I/P 		A 32 bit wire that holds the data to be written into the memory
		readData  - O/P reg		A 32 bit reg that will output the data held in the memory at the address
		memWrite  - I/P 		A 1 bit wire that indicates whether data will be written to the memory
		memRead	  - I/P 	 	A 1 bit wire that indicates whether data will be read from the memory
		clock     - I/P 	    A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI. 
 */
module DataMemory(address, writeData, readData, memWrite, memRead, clock);
	input[31:0] address, writeData;						// Instantiate 32 bit input wires
	output reg[31:0] readData;							// Instantiate 32 bit output registers
	input memWrite, memRead, clock;						// Instantiate 1 bit input wires
	
	reg[31:0] RF[0:1023];									// Make 32 x 128 bit memory unit
	

	
	always @(posedge clock) 
	begin												// At the rising edge of the clock
		if (memWrite == 1'b1) begin						// If the user wants to write to the memory
			RF[address[11:2]] <= writeData;					// Set the corresponding spot in memory to inputed data
		end
		
		if (memRead == 1'b1) begin						// If the user wants to read from the memory
			readData <= RF[address[11:2]];					// Set the output to the corresponding spot in memory
		end
	end
endmodule
 /*
  *  Module:	SignExtender	  - A module that will convert a 16 bit wire to 32 bits
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		in   - I/P 		   A 16 bit wire to be converted to 32 bits
		out	 - O/P 		   A 32 bit wire that will be outputted
 */
module SignExtender(in, out);
	input [15:0] in;			// Wire with the initial 16 bit data
	output [31:0] out;			// Output wire with the final 32 bit data
	
	assign out = {{16{in[15]}}, {in[15:0]}};	// Concatenate the 16 bit wire into the 32 bit wire 
	
endmodule
/*
  *  Module:	Registers		  - A module that outputs data from a memory based off two inputed registers, will also write registers to memory if notified
 *  Author:	Dr. Richard Goodrum
 *	Ports:
 *		readReg1  - I/P     A 5 bit which holds the data from the first inputted register
		readReg2  - I/P     A 5 bit which holds the data from the second inputted register
		writeReg  - I/P		A 5 bit which holds the location of where to right the output to in the memory
		writeData - I/P		A 32 bit with the data to be written to memory
		regWrite  - I/P		A 1 bit that indicates whether the user wants to write to memory or not
		outData1  - O/P		A 32 bit that outputs the data associated with register1 from the memory
		outData2  - O/P		A 32 bit that outputs the data associated with register2 from the memory
		clock     - I/P     A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI. 
 */
module Registers(readReg1, readReg2, writeReg, writeData, regWrite, outData1, outData2, clock);
	input [4:0] readReg1, readReg2, writeReg;		// Instantiate 5 bit inputs
	input[31:0] writeData;							// Instantiate a 32 bit input
	input regWrite, clock;							// Instantiate 1 bit inputs
	output[31:0] outData1, outData2;				// Instantiate 32 bit outputs
	
	reg[31:0]RF[0:31];								// Create a 32 x 32 memory
	
	
	assign outData1 = RF[readReg1];					// Make outData1 the data in memory RF at position readReg1
	assign outData2 = RF[readReg2];					// Make outData2 the data in memory RF at position readReg2
	
	always
	begin
		@(posedge clock)							// At rising edge of clock
		if (regWrite)								// If the user wants to write to the memory
			RF[writeReg] <= writeData;				// Set the memory at index writeReg to the data inputted
	end
endmodule
 /*
  *  Module:	MUXone	  - A module that choose the output from two 32 bit inputs based off a selector
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		A   - I/P 		   A 32 bit that may be the output
		B   - I/P 		   A 32 bit that may be the output
		sel - I/P		   A 1 bit that chooses A or B
		out	 - O/P 		   A 32 bit wire that will be outputted
 */
module MUXone(A, B, sel, out);
	output[31:0] out;
	input[31:0] A, B;
	input sel;
	
	reg out;
	
	always @(A or B or sel)
		if (sel == 1'b0)
			out = A;
		else
			out = B;
endmodule
 /*
  *  Module:	MUXtwo	  - A module that choose the output from two 5 bit inputs based off a selector
 *  Author:	Kameron Noorbakhsh
 *	Ports:
 *		A   - I/P 		   A 5 bit that may be the output
		B   - I/P 		   A 5 bit that may be the output
		sel - I/P		   A 1 bit that chooses A or B
		out	 - O/P 		   A 5 bit wire that will be outputted
 */
module MUXtwo(A, B, sel, out);
	output[4:0] out;
	input[4:0] A, B;
	input sel;
	
	reg out;
	
	always @(A or B or sel)
		if (sel == 1'b0)
			out = A;
		else
			out = B;
endmodule