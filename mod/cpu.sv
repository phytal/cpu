`ifndef cpu_module
`define cpu_module

`include "mod/ram.sv"
// `include "mod/instr_decode.sv"
`include "mod/instr_fetch.sv"
`include "mod/control_unit.sv"
`include "mod/pc.sv"
`include "mod/alu.sv"
`include "mod/register_file.sv"
`include "mod/fpu.sv"
`include "mod/io_device.sv"

/*
* CPU module
*
* This module is the CPU of the system. It is responsible for wiring together the different components of the system.
*
* @input clk: The clock signal
* @input reset: The reset signal
* @output halt: The halt signal
* @output in_signal: The input present signal. Set to 1 when reading from the input port
* @input in_data: The input data
* @output out_signal: The output present signal. Set to 1 when writing to the output port
* @output out_data: The output data
*/
module cpu (
    input clk,
    input reset,
    output logic halt,
    output logic error,
    output logic in_signal,
    input logic[63:0] in_data,
    output logic out_signal,
    output logic[63:0] out_data   
);  
    // logic [2:0] state; // 0: fetch, 1: decode, 2: execute, 3: writeback

    // // Define states
    // parameter FETCH = 3'b000;
    // parameter DECODE = 3'b001;
    // parameter EXECUTE = 3'b010;
    // parameter WRITEBACK = 3'b011;
    // Define states for the multicycle processor
    typedef enum logic [2:0] {
        STATE_IF = 3'b000, // Instruction Fetch
        STATE_ID = 3'b001, // Instruction Decode
        STATE_EX = 3'b010, // Execution
        STATE_MEM = 3'b011, // Memory Access
        STATE_WB = 3'b100  // Write-back
    } state_t;
    
    // State register and next state logic
    state_t state, next_state;

    always_ff @(posedge clk or posedge reset)
    begin
        if (reset)
            state <= STATE_IF; // Initial state
        else
            state <= next_state; // Update state based on next_state
        $display("Current state: %0d", state);
        $display("PC: %0h", pc);
        $display("Instruction: %0b", instr);
        $display("Registers: %0h, %0h, %0h, %0d", rd, rs, rt, L);
        $display("Register data: %0h, %0h, %0h", rd_data, rs_data, rt_data);
        $display("ALU operands: %0h, %0h", operand1, operand2);
        $display("ALU result: %0h", alu_result);
        $display("halt: %0d", _halt);
    end

    // Logic to determine next state based on current state
    always_comb begin
        case (state)
            STATE_IF: next_state = STATE_ID;
            STATE_ID: next_state = STATE_EX;
            STATE_EX: next_state = STATE_MEM;
            STATE_MEM: next_state = STATE_WB;
            STATE_WB: next_state = STATE_IF; // Go back to IF for next instruction
            default: next_state = STATE_IF; // Default to IF state
        endcase
    end

    // // State control signals
    // always_ff @(posedge clk, negedge reset) begin
    //     if (reset) begin
    //         state <= FETCH;
    //     end else begin
    //         case (state)
    //             FETCH: begin
    //                 state_fetch <= 1;
    //                 // Other state control signals set to 0
    //                 if (fetch_done) begin // Replace fetch_done with an appropriate condition
    //                     state <= DECODE;
    //                 end
    //             end
    //             DECODE: begin
    //                 state_decode <= 1;
    //                 // Other state control signals set to 0
    //                 if (decode_done) begin // Replace decode_done with an appropriate condition
    //                     state <= EXECUTE;
    //                 end
    //             end
    //             EXECUTE: begin
    //                 state_execute <= 1;
    //                 // Other state control signals set to 0
    //                 if (execute_done) begin // Replace execute_done with an appropriate condition
    //                     state <= WRITEBACK;
    //                 end
    //             end
    //             WRITEBACK: begin
    //                 state_writeback <= 1;
    //                 // Other state control signals set to 0
    //                 if (writeback_done) begin // Replace writeback_done with an appropriate condition
    //                     state <= FETCH;
    //                 end
    //             end
    //             default: state <= FETCH; // Default to FETCH state in case of unknown state
    //         endcase
    //     end
    // end

    // Control Unit
    logic [4:0] op;
    logic [4:0] rd;
    logic [4:0] rs;
    logic [4:0] rt;
    logic [11:0] L;

    logic pc_src;
    logic result_src;
    logic mem_write;
    logic reg_write;
    logic immediate_src;
    logic addr_src;
    logic [1:0] alu_srcA;
    logic [1:0] alu_srcB;
    logic [4:0] alu_op;
    logic in;
    logic out;
    logic alu_pass;
    logic _halt;

    control_unit control_unit (
        .clk(clk),
        .reset(reset),
        .instr(instr),
        .opcode(op),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .L(L),
        .pc_src(pc_src),
        .result_src(result_src),
        .mem_write(mem_write),
        .reg_write(reg_write),
        // .immediate_src(immediate_src)
        .addr_src(addr_src),
        .alu_srcA(alu_srcA),
        .alu_srcB(alu_srcB),
        .alu_op(alu_op),
        .alu_pass(alu_pass),
        .halt(_halt),
        .in_signal(in),
        .out_signal(out)
    );

    assign in_signal = in;
    assign out_signal = out;

    // PC
    logic [63:0] pc_plus_4;
    logic [63:0] pc;

    program_counter prog_c (
        .state(state),
        .clk(clk),
        .reset(reset),
        .pc_plus_4(pc_plus_4),
        .pc_branch(alu_result),
        .pc_src(pc_src),
        .pc_out(pc)
    );

    // Increment PC by 4
    assign pc_plus_4 = pc + 4;

    // RAM
    // logic [63:0] r_addr;
    logic [63:0] rw_addr;
    logic [63:0] rw_data_in;
    // logic rw_write_en;
    wire [31:0] r_data_out;
    wire r_error;
    wire [63:0] rw_data_out;
    wire rw_error;
    
    ram memory (
        .clk(clk),
        .reset(reset),
        .r_addr(pc),
        .rw_addr(alu_result),
        .rw_data_in(rw_data_in),
        .rw_write_en(mem_write),
        .r_data_out(instr),
        .r_error(r_error),
        .rw_data_out(rw_data_out),
        .rw_error(rw_error)
    );
    
    // assign r_addr = pc;

    // Instruction Fetch
    logic [31:0] instr;

    // fetch fetch (
    //     .clk(clk),
    //     .reset(reset),
    //     .instr_in(r_data_out),
    //     .instr_out(instr)
    // );

    wire decode_error;
    // wire halt_signal;

    // Register File
    logic [63:0] rd_data;
    logic [63:0] rs_data;
    logic [63:0] rt_data;
    logic [63:0] stack_pointer;

    register_file rf (
        .state(state),
        .clk(clk),
        .reset(reset),
        .read_reg1(rd),
        .read_reg2(rs),
        .read_reg3(rt),
        .write_reg(rd),
        .write_en(reg_write),
        .write_data(alu_result),
        .read_data1(rd_data),
        .read_data2(rs_data),
        .read_data3(rt_data),
        .stack_pointer(stack_pointer)
    );

    // ALU
    logic [63:0] operand1;
    logic [63:0] operand2;
    logic [63:0] alu_result;
    logic alu_error;

    always_comb begin
        // MUX for ALU operands
        case (alu_srcA)
            2'b00: operand1 = rs_data;
            2'b01: operand1 = rd_data;
            2'b10: operand1 = pc;
            2'b11: operand1 = stack_pointer;
        endcase

        case (alu_srcB)
            2'b00: operand2 = rt_data;
            2'b01: operand2 = L;
            2'b10: operand2 = pc;
        endcase
    end

    alu alu (
        .operand1(operand1),
        .operand2(operand2),
        .op(alu_op),
        // .immediate(L),
        // .immediate_en(immediate_src),
        .pass(alu_pass),
        .res(alu_result),
        .error(alu_error)
    );

    // FPU
    logic [63:0] fpu_result;
    logic ready;
    logic fp_error;

    // fpu fpu (
    //     .clk(clk),
    //     .reset(reset),
    //     .operand1(operand1),
    //     .operand2(operand2),
    //     .op(op),
    //     .res(fpu_result),
    //     .ready(ready),
    //     .error(fp_error)
    // );

    // IO Device
    logic [63:0] io_data;
    logic in_ready;
    logic out_ready;

    io_device io (
        .clk(clk),
        .reset(reset),
        .in_data(in_data),
        .in_signal(in),
        .out_signal(out),
        .out_data(io_data),
        .in_ready(in_ready),
        .out_ready(out_ready)
    );

    assign out_data = rs_data;

    assign halt = _halt;
    assign error = r_error | rw_error | decode_error | alu_error | fp_error;
endmodule

`endif