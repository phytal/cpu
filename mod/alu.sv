//8-bit arithmetic and logical unit
module alu(
    input logic [63:0] operand1, // operand 1
    input logic [63:0] operand2, // operand 2
    input logic [4:0] op,  // Operation code and control signal
	input logic pass, // Pass signal
	output logic [63:0] res, // result
	output logic error // Error signal
);
logic [63:0] result;
logic [63:0] a;
logic [63:0] b;
real a_real;
real b_real;

always @(*) begin
	a = operand1;
	b = operand2;
	a_real = a;
	b_real = b;

	if (pass) begin
		result = (a == 1'bz)? b : a; // Pass signal 
	end
	else begin
		// Perform operation based on operation code
		case (op)
			// Integer Arithmetic Instructions
			6'b000000: // add rd, rs, rt
				result = a + b;
			6'b000001: // addi rd, L
				result = a + b;
			6'b000010: // sub rd, rs, rt
				result = a - b;
			6'b000011: // subi rd, L
				result = a - b;
			6'b000100: // mul rd, rs, rt
				result = a * b;
			6'b000101: // div rd, rs, rt
				if (b == 0)
					error = 1;
				else
					result = a / b;

			// Logic Instructions
			6'b000110: // and rd, rs, rt
				result = a & b;
			6'b000111: // or rd, rs, rt
				result = a | b;
			6'b001000: // xor rd, rs, rt
				result = a ^ b;
			6'b001001: // not rd, rs
				result = ~a;
			6'b001010: // shftr rd, rs, rt
				result = a >> b;
			6'b001011: // shftl rd, rs, rt
				result = a << b;
			6'b001100: // shftri rd, L
				result = a >> b;
			6'b001101: // shftli rd, L
				result = a << b;

			// Floating Point Instructions
			6'h19: // fadd rd, rs, rt
				result = a_real + b_real;
			6'h1a: // fsub rd, rs, rt
				result = a_real - b_real;
			6'h1b: // fmul rd, rs, rt
				result = a_real * b_real;
			6'h1c: // fdiv rd, rs, rt
				if (b_real == 0)
					error = 1;
				else
					result = a_real / b_real;
		endcase
	end
end

// Output the result
assign res = result;

endmodule