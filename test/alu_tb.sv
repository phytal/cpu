`include "mod/alu.sv"
`timescale 1ns/1ns

module alu_tb;

    // Declare signals and variables
    logic [63:0] op1;
    logic [63:0] op2;
    logic [4:0] opcode;
    wire [63:0] result;
    logic [11:0] label;
    logic label_en;

    // Instantiate the ALU module
    alu uut (
        .operand1(op1),
        .operand2(op2),
        .op(opcode),
        .res(result)
    );

    // Clock generation
    reg clk;
    always #5 clk = ~clk;

    // Test stimulus
    initial begin
        clk = 0;
        op1 = 4'b0000;
        op2 = 4'b0000;
        opcode = 3'b000;

        // Test case 1: 1 + 1
        #10;
        op1 = 4'b0001;
        op2 = 4'b0001;
        opcode = 3'b000;
        

        // Test case 3: 4 - 3
        #10;
        op1 = 4'b0100;
        op2 = 4'b0011;
        opcode = 3'b010;

        // Add more test cases as needed

        // End simulation
        #10;
        $finish;
    end

    // Monitor for displaying results
    always @(result)
        $display("Result: %b", result);

endmodule