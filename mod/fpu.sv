// module fpu (
//     input wire clk,
//     input wire reset,
//     input wire [63:0] operand1,
//     input wire [63:0] operand2,
//     input wire [4:0] op, 
//     output wire [63:0] result,
//     output wire ready,  // Flag indicating that the result is ready
//     output wire error  // Flag indicating floating-point exception
// );
// logic [63:0] res;
// logic [63:0] a;
// logic [63:0] b;
// logic ready_bit;

// always @(*) begin
// 	a = operand1;
//     b = operand2;

// 	// Perform operation based on operation code
// 	case (op) 
// 		// Integer Arithmetic Instructions
//         6'h19: begin // addf
//             fp_add add_inst (
//                 .clk(clk),
//                 .in_ready(1),
//                 .a(a),
//                 .b(b),
//                 .sum(res),
//                 .done(ready_bit)
//             );
//         end
// 		6'h1a: // subf
// 		6'h1b: begin // mulf
//             fp_mul mul_inst (
//                 .clk(clk),
//                 .in_ready(1),
//                 .a(a),
//                 .b(b),
//                 .mul(res),
//                 .done(ready_bit)
//             );
//         end
// 		6'h1c: // divf
// 	endcase
// end

// // Output the result
// assign res = result;
// assign ready = ready_bit;
// endmodule



// module fp_add (
//     input wire clk,
//     input wire in_ready,
//     input wire [63:0] a,
//     input wire [63:0] b,
//     output wire [63:0] sum, 
//     output wire done  // Indicates completion of addition
// );
//     // Internal signals
//     reg [51:0] aMantissa;
//     reg [51:0] bMantissa;
//     reg [10:0] aExponent;
//     reg [10:0] bExponent;
//     reg [52:0] beforeNormalizeMantissa;  // Increase by 1 for possible carry
//     reg [11:0] beforeNormalizeExponent;  // Increase by 1 for possible carry
//     reg [51:0] sumMantissa;
//     reg [10:0] sumExponent;
//     reg readyReg;
//     reg doneReg;

//     initial begin
//         doneReg = 0;
//         readyReg = 0;
//         sumMantissa = 0;
//         sumExponent = 0;
//     end

//     always @(posedge clk) begin
//         if (readyReg == 0 && in_ready) begin
//             // Set up mantissas and exponents
//             aMantissa = a[51:0];
//             bMantissa = b[51:0];
//             aExponent = a[62:52];
//             bExponent = b[62:52];
//             readyReg = in_ready;
//         end

//         if (readyReg) begin
//             // First compare
//             if (aExponent > bExponent) begin
//                 beforeNormalizeExponent = aExponent;
//                 beforeNormalizeMantissa = aMantissa;
//                 beforeNormalizeMantissa = aMantissa + (bMantissa << (aExponent - bExponent));
//             end else begin
//                 beforeNormalizeExponent = bExponent;
//                 beforeNormalizeMantissa = bMantissa;
//                 beforeNormalizeMantissa = bMantissa + (aMantissa << (bExponent - aExponent));
//             end
//             doneReg = 1;
//         end
//     end

//     // Output signals
//     assign sum = {sumExponent, sumMantissa};
//     assign done = doneReg;

// endmodule


// module fp_mul (
//     input wire clk,
//     input wire in_ready,
//     input wire [63:0] a,
//     input wire [63:0] b,
//     output wire [63:0] mul,  // Result of multiplication
//     output wire done  // Indicates completion of multiplication
// );
//     // Internal signals
//     reg [51:0] bMantissa;
//     reg [10:0] aExponent;
//     reg [10:0] bExponent;
//     reg [52:0] beforeNormalizeMantissa;  // Increase by 1 for possible carry
//     reg [11:0] beforeNormalizeExponent;  // Increase by 1 for possible carry
//     reg [11:0] sumExponent;
//     reg readyReg;
//     reg doneReg;

//     initial begin
//         doneReg = 0;
//         readyReg = 0;
//     end

//     always @(posedge clk) begin
//         if (readyReg == 0 && in_ready) begin
//             // Set up mantissas and exponents
//             aMantissa = a[51:0];
//             bMantissa = b[51:0];
//             aExponent = a[62:52];
//             bExponent = b[62:52];
//             readyReg = in_ready;
//         end

//         if (readyReg) begin
//             beforeNormalizeMantissa = aMantissa * bMantissa;
//             sumExponent = aExponent + bExponent;
            
//             // Normalize - Check if the last bit is 0 and keep shifting
//             while (beforeNormalizeMantissa[0] == 0) begin
//                 beforeNormalizeMantissa = beforeNormalizeMantissa << 1;
//                 sumExponent = sumExponent - 1;
//             end

//             doneReg = 1;
//         end
//     end

//     // Output signals
//     assign mul = {sumExponent, beforeNormalizeMantissa[51:0]};
//     assign done = doneReg;
// endmodule