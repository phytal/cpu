//8-bit arithmetic and logical unit
module mem_exec(
	// input logic clk, // Clock signal
    input logic [63:0] operand1, // operand 1
    input logic [63:0] operand2, // operand 2
    input logic [4:0] op,  // Operation code
	input logic [11:0] label, // Label for immediate value
	input logic label_en, // Enable label
    // output logic [63:0] rd, // Register destination result
	output logic [63:0] res // result
	// output logic write_en
);
logic [63:0] result;
logic [63:0] a;
logic [63:0] b;

always @(*) begin
	a = operand1;

	// Check if label is enabled
	if(label_en) begin
		b[11:0] = label[11:0];
	end else begin
		b = operand2;
	end

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
		// 6'b001110: // fadd rd, rs, rt
		// 	FP_add adder(.clk(clk), .in_ready(in_ready), .a(a), .b(b), .sum(result), .done(done));
	endcase
end

// Output the result
assign res = result;

endmodule