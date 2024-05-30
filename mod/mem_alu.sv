// Simple 64-bit ALU for memory operations
module mem_alu(
    input logic [63:0] operand1, // operand 1
    input logic [63:0] operand2, // operand 2
    input logic [1:0] op,  // Operation code and control signal
	output logic [63:0] res // result
);
logic [63:0] result;
logic [63:0] a;
logic [63:0] b;

always @(*) begin
	a = operand1;
	b = operand2;

	// Perform operation based on operation code
	case (op)
		// Integer Arithmetic Instructions
		2'b00: // add rd, rs, rt
			result = a + b;
		2'b01: // addi rd, L
			result = a + b;
		2'b10: // sub rd, rs, rt
			result = a - b;
		2'b11: // subi rd, L
			result = a - b;
	endcase
end

// Output the result
assign res = result;

endmodule