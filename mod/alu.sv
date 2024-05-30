// 64-bit ALU
module alu(
	input logic [3:0] state, // State of the CPU
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
	a <= operand1;
	b <= operand2;
	a_real <= $bitstoreal(a);
	b_real <= $bitstoreal(b);

	if (pass) begin
		result = (a == 1'bz)? b : a; // Pass signal 
	end
	else begin
		// Perform operation based on operation code
		case (op)
			// Integer Arithmetic Instructions
			5'h0: // add rd, rs, rt
				result = a + b;
			5'h1: // addi rd, L
				result = a + b;
			5'h2: // sub rd, rs, rt
				result = a - b;
			5'h3: // subi rd, L
				result = a - b;
			5'h4: // mul rd, rs, rt
				result = a * b;
			5'h5: // div rd, rs, rt
				if (b == 0 && state == 3'b010) // Check if the state is in the CPU state
					error = 1;
				else
					result = a / b;

			// Logic Instructions
			5'h6: // and rd, rs, rt
				result = a & b;
			5'h7: // or rd, rs, rt
				result = a | b;
			5'h8: // xor rd, rs, rt
				result = a ^ b;
			5'h9: // not rd, rs
				result = ~a;
			5'ha: // shftr rd, rs, rt
				result = a >> b;
			5'hb: // shftri rd, L
				result = a >> b;
			5'hc: // shftl rd, rs, rt
				result = a << b;
			5'hd: // shftli rd, L
				result = a << b;

			// Floating Point Instructions
			5'h19: // fadd rd, rs, rt
				result = $realtobits(a_real + b_real);
			5'h1a: // fsub rd, rs, rt
				result = $realtobits(a_real - b_real);
			5'h1b: // fmul rd, rs, rt
				result = $realtobits(a_real * b_real);
			5'h1c: // fdiv rd, rs, rt
				if (b_real == 0 && state == 3'b010)
					error = 1;
				else
					result = $realtobits(a_real / b_real);

			5'h17: begin // MOVRL
				result = a;
				result[11:0] = b[11:0];
			end
		endcase
	end
end

// Output the result
assign res = result;

endmodule