`include "mod/control_unit.sv"

`timescale 1ns / 1ps

module control_unit_tb;

    // Signals
    logic clk;
    logic reset;
    logic [31:0] instr;
    logic [4:0] opcode;
    logic [4:0] rd;
    logic [4:0] rs;
    logic [4:0] rt;
    logic [11:0] L;
    logic pc_src;
    logic result_src;
    logic mem_write;
    logic reg_write;
    logic addr_src;
    logic [1:0] alu_srcA;
    logic [1:0] alu_srcB;
    logic [4:0] alu_op;
    logic alu_pass;
    logic halt;
    logic in_signal;
    logic out_signal;

    // Instantiate the control unit module
    control_unit dut (
        .clk(clk),
        .reset(reset),
        .instr(instr),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .L(L),
        .pc_src(pc_src),
        .result_src(result_src),
        .mem_write(mem_write),
        .reg_write(reg_write),
        .addr_src(addr_src),
        .alu_srcA(alu_srcA),
        .alu_srcB(alu_srcB),
        .alu_op(alu_op),
        .alu_pass(alu_pass),
        .halt(halt),
        .in_signal(in_signal),
        .out_signal(out_signal)
    );

    // Clock generation
    always #10 clk = ~clk;

    // Initial values
    initial begin
        clk = 0;
        reset = 1;
        instr = 32'h00000000; // Initial instruction
        #50 reset = 0; // De-assert reset
    end

    // Display control signals
    always @(posedge clk) begin
        $display("opcode: %h, rd: %h, rs: %h, rt: %h, L: %d, pc_src: %b, result_src: %b, mem_write: %b, reg_write: %b, addr_src: %b, alu_srcA: %d, alu_srcB: %d, alu_op: %h, alu_pass: %b, halt: %b, in_signal: %b, out_signal: %b",
                 opcode, rd, rs, rt, L, pc_src, result_src, mem_write, reg_write, addr_src, alu_srcA, alu_srcB, alu_op, alu_pass, halt, in_signal, out_signal);
    end

    // Test cases
    initial begin
        // // Test case 1: R-Type instruction
        instr = 32'b00000000000000000000000000000000; // Example R-Type instruction
        #20;

        // // Test case 2: L-Type instruction
        instr = 32'b00001000000000000000000000000010; // Example L-Type instruction
        #20;

        // Test case 2: BRR
        instr = 32'b01111000000000000000000000000011; // Example L-Type instruction
        #20;

        // // buggy
        // instr = 32'b00000000000000000000000000000000; // Example L-Type instruction
        // #20;

        // // buggy
        // instr = 32'b00000000000000000000000000000000; // Example L-Type instruction
        // #20;

        // Add more test cases as needed...

        // End simulation
        $finish;
    end

endmodule
