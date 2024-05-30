`ifndef cpu_module
`define cpu_module

`include "mod/ram.sv"
// `include "mod/instr_decode.sv"
// `include "mod/instr_fetch.sv"
`include "mod/control_unit.sv"
`include "mod/pc.sv"
`include "mod/alu.sv"
`include "mod/register_file.sv"
`include "mod/io_device.sv"
`include "mod/mem_alu.sv"

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
    // Define states for the multicycle processor
    typedef enum logic [2:0] {
        STATE_IF = 3'b000, // Instruction Fetch
        STATE_ID = 3'b001, // Instruction Decode
        STATE_EX = 3'b010, // Execution
        STATE_WB = 3'b011  // Write-back
    } state_t;
    
    // State register and next state logic
    state_t state, next_state;
    logic halt_signal = 1'b0;

    always_ff @(posedge clk or posedge reset)
    begin
        if (reset) begin
            state <= STATE_IF; // Initial state
            halt_signal <= 1'b0;
        end else
            state <= next_state; 
    end

    // Logic to determine next state based on current state
    always_comb begin
        case (state)
            STATE_IF: next_state = STATE_ID;
            STATE_ID: next_state = STATE_EX;
            STATE_EX: next_state = STATE_WB;
            STATE_WB: next_state = STATE_IF; // Go back to IF for next instruction
            default: next_state = STATE_IF; // Default to IF state
        endcase
    end

    // Control Unit
    logic [4:0] op;
    logic [4:0] rd;
    logic [4:0] rs;
    logic [4:0] rt;
    logic [63:0] L;

    logic pc_src;
    logic [1:0] result_src; // 0: ALU, 1: MEM, 2: IN PORT
    logic mem_write;
    logic reg_write;
    logic immediate_src;
    logic addr_src;
    logic [1:0] alu_srcA;
    logic [1:0] alu_srcB;
    logic [4:0] alu_op;
    logic [1:0] mem_alu_srcA;
    logic [1:0] mem_alu_srcB;
    logic [1:0] mem_alu_op;
    logic mem_src;
    logic in;
    logic out;
    logic alu_pass;
    logic halt_signal_2;
    logic brnz;
    logic brgt;
    logic decode_error;

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
        .addr_src(addr_src),
        .alu_srcA(alu_srcA),
        .alu_srcB(alu_srcB),
        .alu_op(alu_op),
        .alu_pass(alu_pass),
        .mem_alu_srcA(mem_alu_srcA),
        .mem_alu_srcB(mem_alu_srcB),
        .mem_alu_op(mem_alu_op),
        .mem_src(mem_src),
        .halt(halt_signal_2),
        .in_signal(in),
        .out_signal(out),
        .brnz(brnz),
        .brgt(brgt),
        .decode_error(decode_error)
    );

    // PC
    logic [63:0] pc_plus_4;
    logic [63:0] pc;

    program_counter program_counter (
        .state(state),
        .clk(clk),
        .reset(reset),
        .pc_plus_4(pc_plus_4),
        .pc_branch(write_data),
        .pc_src(_pc_src),
        .pc_out(pc)
    );

    // Increment PC by 4
    assign pc_plus_4 = pc + 4;

    // BRNZ and BRGT
    logic _pc_src;
    always_comb begin
        if (brnz == 1'b1) begin
            _pc_src = rs_data != 0;
        end
        else if (brgt == 1'b1) begin
            _pc_src = rs_data > rt_data;
        end
        else begin
            _pc_src = pc_src;
        end
    end

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
        .rw_addr(mem_alu_result),
        .rw_data_in(rw_data_in),
        .rw_write_en(mem_write),
        .r_data_out(instr),
        .r_error(r_error),
        .rw_data_out(rw_data_out),
        .rw_error(rw_error)
    );

    // MUX for memory data to write
    always_comb begin
        if (mem_src)
            rw_data_in = alu_result;
        else
            rw_data_in = pc_plus_4;
    end

    // MUX for data movement
    logic [63:0] data;
    always_comb begin
        if (addr_src)
            data = alu_result;
        else
            data = rw_data_out;
    end

    // MUX for register write data
    logic [63:0] write_data;
    always_comb begin
        if (result_src == 2'b01)
            write_data = data;
        else if (result_src == 2'b10)
            write_data = in_data;
        else // result_src == 2'b00
            write_data = alu_result;
    end

    // MEM ALU
    logic [63:0] mem_operand1;
    logic [63:0] mem_operand2;
    logic [63:0] mem_alu_result;

    always_comb begin
        // MUX for ALU operands
        case (mem_alu_srcA)
            2'b00: mem_operand1 = rs_data;
            2'b01: mem_operand1 = rd_data;
            2'b10: mem_operand1 = pc;
            2'b11: mem_operand1 = stack_pointer;
        endcase

        case (mem_alu_srcB)
            2'b00: mem_operand2 = rt_data;
            2'b01: mem_operand2 = L;
            2'b10: mem_operand2 = pc;
        endcase
    end


    mem_alu mem_alu (
        .operand1(mem_operand1),
        .operand2(mem_operand2),
        .op(mem_alu_op),
        .res(mem_alu_result)
    );

    // Instruction Fetch
    logic [31:0] instr;

    // fetch module is redundant because of ram unit
    // fetch fetch (
    //     .clk(clk),
    //     .reset(reset),
    //     .instr_in(r_data_out),
    //     .instr_out(instr)
    // );

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
        .write_data(write_data),
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
        .pass(alu_pass),
        .res(alu_result),
        .error(alu_error)
    );

    // IO Device
    logic [63:0] io_data;
    logic clocked_in_signal;
    logic clocked_out_signal;

    io_device io (
        .state(state),
        .clk(clk),
        .reset(reset),
        .in_signal(in),
        .out_signal(out),
        .clocked_in_signal(clocked_in_signal),
        .clocked_out_signal(clocked_out_signal)
    );

    
    assign in_signal = clocked_in_signal;
    assign out_signal = clocked_out_signal;

    assign out_data = rs_data;

    assign halt = halt_signal | halt_signal_2;
    assign error = r_error | rw_error | decode_error | alu_error;
endmodule

`endif