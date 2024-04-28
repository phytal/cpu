// this module should output the values inside the registers given an entire instruction
// `include "mod/register_file.sv"
// `include "mod/alu.sv"
// `include "mod/fpu.sv"

module decode (
    input [31:0] instr,
    input clk,
    input reset,
    // input logic [31:0] pc,
    // input logic [31:0] pc,
    output reg [4:0] op,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [11:0] L
);
    // Internal signals to hold instruction fields
    reg [4:0] _rs;
    reg [4:0] _rt;
    reg [4:0] _rd;
    reg [4:0] _op;
    reg [11:0] _L;

    assign op = _op;
    assign rd = _rd;
    assign rs = _rs;
    assign rt = _rt;
    assign L = _L;

    // Decode the instruction and perform the corresponding operation on positive edge of clock
    always @(posedge clk or posedge reset) begin
    // always @(*) begin
        // Extracting opcode
        _op = instr[31:27];

        // Extracting register specifiers
        _rd = instr[26:22];
        _rs = instr[21:17];
        _rt = instr[16:12];

        // Extracting literal
        _L = $signed(instr[11:0]);
    end
endmodule

// // this module should output the values inside the registers given an entire instruction
// // `include "mod/register_file.sv"
// // `include "mod/alu.sv"
// // `include "mod/fpu.sv"

// module decode (
//     input [31:0] instr,
//     // input clk,
//     // input reset,
//     // input logic [31:0] pc,
//     // input logic [31:0] pc,
//     output reg [4:0] op,
//     output reg [4:0] rd,
//     output reg [4:0] rs,
//     output reg [4:0] rt,
//     output reg [11:0] L
//     // output logic [31:0] new_pc,
//     // output logic [63:0] write_data,
//     // output logic ready,
//     // output logic error,
//     // output logic halt
// );
//     // Internal signals to hold instruction fields
//     reg [4:0] _rs;
//     reg [4:0] _rt;
//     reg [4:0] _rd;
//     reg [4:0] _op;
//     reg [11:0] _L;
//     // reg [63:0] _operand1;
//     // reg [63:0] _operand2;
//     // reg [31:0] updated_pc = pc;

//     // Internal signals to hold register file data
//     // logic [63:0] rd_data; // Data read from register file for rd
//     // logic [63:0] rs_data; // Data read from register file for rs
//     // logic [63:0] rt_data; // Data read from register file for rt

//     // logic [63:0] rw_data; // Data to be written to register file
//     // logic reg_write; // Write enable signal for register file

//     // register_file rf (
//     //     .clk(clk),
//     //     .reset(reset),
//     //     .read_reg1(rd),
//     //     .read_reg2(rs),
//     //     .read_reg3(rt),

//     //     .write_reg(rd),
//     //     .write_en(reg_write),
//     //     .write_data(rw_data),

//     //     .read_data1(rd_data),
//     //     .read_data2(rs_data),
//     //     .read_data3(rt_data)
//     // );

//     // alu alu (
//     //     .operand1(operand1),
//     //     .operand2(operand2),
//     //     .op(op),
//     //     .res(rw_data)
//     // );

//     // fpu fpu (
//     //     .operand1(operand1),
//     //     .operand2(operand2),
//     //     .op(op),
//     //     .res(rw_data),
//     //     .ready(ready),
//     //     .error(error),
//     // );

//     // mem_exec 

//     assign op = _op;
//     assign rd = _rd;
//     assign rs = _rs;
//     assign rt = _rt;
//     assign L = _L;

//     // Decode the instruction and perform the corresponding operation on positive edge of clock
//     // always @(posedge clk or posedge reset) begin
//     always @(*) begin
//         // Extracting opcode
//         _op = instr[31:27];

//         // Extracting register specifiers
//         _rd = instr[26:22];
//         _rs = instr[21:17];
//         _rt = instr[16:12];

//         // Extracting literal
//         _L = $signed(instr[11:0]);

//     //     case(op)
//     //         // add rd, rs, rt | sub rd, rs, rt | mul rd, rs, rt | div rd, rs, rt | and rd, rs, rt | or rd, rs, rt 
//     //         // xor rd, rs, rt | not rd, rs | shftr rd, rs, rt | shftl rd, rs, rt | cmp rd, rs, rt
//     //         6'h0, 6'h2, 6'h4, 6'h5, 6'h6, 6'h7, 6'h8, 6'h9, 6'ha, 6'hc: begin                
//     //             operand1 = rs_data;
//     //             operand2 = rt_data;
//     //             reg_write = 1'b1;
//     //         end
//     //         // addi rd, L | subi rd, L | shftri rd, L | shftli rd, L
//     //         6'h1, 6'h3, 6'hb, 6'hd: begin
//     //             operand1 = rd_data;
//     //             operand2 = $signed(L);
//     //             reg_write = 1'b1;
//     //         end

//     //         // Control Instructions
//     //         6'h0e: // br rd
//     //             result = rd_data;
//     //         6'h0f: // brr rd
//     //             result = updated_pc + rd_data;
//     //         6'h10: // brr L
//     //             result = updated_pc + $signed(L);
//     //         6'h11: // brnz rd, rs
//     //             result = (rs_data == 0) ? (updated_pc + 4) : rd_data;
//     //         6'h12: // call rd, rs, rt
//     //             write_data = updated_pc + 4;
//     //             result = 
//     //             updated_pc = rd_data;
//     //         6'h13: // return
//     //             result = Mem[r31 - 8];
//     //         6'h14: // brgt rd, rs, rt
//     //             result = (rs <= rt) ? (pc + 4) : rd;

//     //         // Data Movement Instructions
//     //         6'h15: // mov rd, (rs)(L)
//     //             result = Mem[rs + L];
//     //         6'h16: // mov rd, rs
//     //             result = rs;
//     //         6'h17: // mov rd, L
//     //             result[63:52] = L;
//     //         6'h18: // mov (rd)(L), rs
//     //             result = rd_data + $signed(L);
//     //             write_data = rs_data;

//     //         default: begin
//     //             // Default case if op does not match any defined opcode
//     //             // Handle unrecognized opcodes or add error handling
//     //             error = 1'b1;
//     //         end
//     //     endcase
//     end


//     // Set ready signal to indicate that the module has completed execution
//     // assign ready = 1;
//     // assign new_pc = updated_pc;



// endmodule