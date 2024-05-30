module control_unit (
    input logic clk,
    input logic reset,
    input [31:0] instr,
    output logic [4:0] opcode,
    output logic [4:0] rd,
    output logic [4:0] rs,
    output logic [4:0] rt,
    output logic [63:0] L,
    output logic pc_src,
    output logic [1:0] result_src,
    output logic mem_write,
    output logic reg_write,
    output logic addr_src, // for data movement
    output logic [1:0] alu_srcA, // 0: rs, 1: rd, 2: pc, 3: sp
    output logic [1:0] alu_srcB, // 0: rt, 1: immediate, 2: pc
    output logic [4:0] alu_op, 
    output logic alu_pass,
    output logic [1:0] mem_alu_srcA,
    output logic [1:0] mem_alu_srcB,
    output logic [1:0] mem_alu_op,
    output logic mem_src,
    output logic halt,
    output logic in_signal,
    output logic out_signal,
    output logic brnz,
    output logic brgt,
    output logic decode_error
);
    // Internal signals to hold instruction fields
    logic [4:0] _rs;
    logic [4:0] _rt;
    logic [4:0] _rd;
    logic [4:0] _opcode;
    logic [63:0] _L;
    logic _pc_src;
    logic [1:0] _result_src;
    logic _mem_write;
    logic _reg_write;
    logic _addr_src;
    logic [1:0] _alu_srcA;
    logic [1:0] _alu_srcB;
    logic [4:0] _alu_op;
    logic _alu_pass;
    logic [1:0] _mem_alu_srcA;
    logic [1:0] _mem_alu_srcB;
    logic [1:0] _mem_alu_op;
    logic _mem_src;
    logic _halt;
    logic _in_signal;
    logic _out_signal;
    logic _brnz;
    logic _brgt;
    logic _decode_error;

    assign opcode = _opcode;
    assign rd = _rd;
    assign rs = _rs;
    assign rt = _rt;
    assign L = _L;
    assign pc_src = _pc_src;
    assign result_src = _result_src;
    assign mem_write = _mem_write;
    assign reg_write = _reg_write;
    assign addr_src = _addr_src;
    assign alu_srcA = _alu_srcA;
    assign alu_srcB = _alu_srcB;
    assign alu_op = _alu_op;
    assign alu_pass = _alu_pass;
    assign mem_alu_srcA = _mem_alu_srcA;
    assign mem_alu_srcB = _mem_alu_srcB;
    assign mem_alu_op = _mem_alu_op;
    assign mem_src = _mem_src;
    assign halt = _halt;
    assign in_signal = _in_signal;
    assign out_signal = _out_signal;
    assign brnz = _brnz;
    assign brgt = _brgt;
    assign decode_error = _decode_error;

    // // Define control signals based on opcode
    always @(*) begin
        // Resetting control signals
        _pc_src <= 1'b0;
        _mem_write <= 1'b0;
        _reg_write <= 1'b0;
        _addr_src <= 1'b0;
        _alu_srcA <= 2'b00;
        _alu_srcB <= 2'b00;
        _alu_op <= 5'b00000;
        _alu_pass <= 1'b0;
        _halt <= 1'b0;
        _in_signal <= 1'b0;
        _out_signal <= 1'b0;
        _brnz <= 1'b0;
        _brgt <= 1'b0;
        _result_src <= 2'b00;
        _mem_src <= 1'b0;
        _addr_src <= 1'b0;

        // Extracting opcode
        _opcode <= instr[31:27];

        // Extracting register specifiers
        _rd <= instr[26:22];
        _rs <= instr[21:17];
        _rt <= instr[16:12];

        // Extracting literal
        _L[11:0] <= instr[11:0];
        _L[63:12] <= {52{instr[11]}};
        

// ====================================================================================================
        case (_opcode)
            // Integer and Logic Arithmetic Instructions
            5'h00, 5'h02, 5'h04, 5'h05, 5'h06, 5'h07, 5'h08, 5'h09, 5'h0a, 5'h0c: begin  // R-Type
                _reg_write <= 1'b1;
                _alu_srcA <= 2'b00;
                _alu_srcB <= 2'b00;
                _alu_op <= _opcode;
            end
            5'h01, 5'h03, 5'h0b, 5'h0d: begin  // L-Type
                _reg_write <= 1'b1;
                _alu_srcA <= 2'b01;
                _alu_srcB <= 2'b01;
                _alu_op <= _opcode; 
            end
// ====================================================================================================
            // Control Instructions
            5'h0E: begin  // BR, pc = rd
                _alu_srcA <= 2'b01;
                _alu_pass <= 1'b1;
                _pc_src <= 1'b1;
            end
            5'h0F: begin  // BRR, pc = rd + pc
                _alu_srcA <= 2'b01;
                _alu_srcB <= 2'b10;
                _alu_pass <= 1'b0;
                _alu_op <= 5'b00000;
                _pc_src <= 1'b1;
            end
            5'h10: begin  // BRR L pc = pc + L
                _alu_srcA <= 2'b10;
                _alu_srcB <= 2'b01;
                _alu_pass <= 1'b0;
                _alu_op <= 5'b00000;
                _pc_src <= 1'b1;
            end
            5'h11: begin  // BRNZ pc = (rs == 0) ? (pc + 4) : rd
                _brnz <= 1'b1;
                _alu_srcA <= 2'b1;
                _alu_pass <= 1'b1;
            end
            5'h12: begin  // CALL mem[sp-8] = pc, pc = rd
                _alu_srcA <= 2'b01;
                _alu_pass <= 1'b1;
                _pc_src <= 1'b1;
                _mem_write <= 1'b1;
                _addr_src <= 1'b1;
                _mem_alu_srcA <= 2'b11;
                _mem_alu_srcB <= 2'b01;
                _mem_alu_op <= 2'b11;
                _mem_src <= 1'b1;
                
            end
            5'h13: begin  // RETURN pc = mem[sp-8]
                _addr_src <= 1'b1;
                _result_src <= 2'b01;
                _mem_alu_srcA <= 2'b11;
                _mem_alu_srcB <= 2'b01;
                _mem_alu_op <= 2'b11;
                _pc_src <= 1'b1;
            end
            5'h14: begin  // BRGT pc = rs <= rt? pc + 4 : rd
                _brgt <= 1'b1;
                _alu_srcA <= 2'b1;
                _alu_pass <= 1'b1;
            end
            5'h1F: begin  // HALT
                _halt <= 1'b1;
            end
// ====================================================================================================
            // Data Movement Instructions
            5'h15: begin  // MOV rd ← Mem[rs + L]
                _reg_write <= 1'b1;
                _mem_alu_srcA <= 2'b00;
                _mem_alu_srcB <= 2'b01;
                _mem_alu_op <= 5'b00000;
                _result_src <= 2'b01;
            end
            5'h16: begin  // MOV rd ← rs
                _reg_write <= 1'b1;
                _alu_srcA <= 2'b00;
                _alu_pass <= 1'b1;
            end
            5'h17: begin  // MOV rd [52:63] ← L
                _reg_write <= 1'b1;
                _alu_srcA <= 2'b01;
                _alu_srcB <= 2'b01;
                _alu_op <= _opcode;
            end
            5'h18: begin  // MOV Mem[rd + L] ← rs
                _mem_write <= 1'b1;
                _mem_alu_srcA <= 2'b01;
                _mem_alu_srcB <= 2'b01;
                _mem_alu_op <= 5'b00000;
                _alu_srcA <= 2'b00;
                _alu_pass <= 1'b1;
                _addr_src <= 1'b1;
                _result_src <= 2'b01;
            end
// ====================================================================================================
            5'h19, 5'h1A, 5'h1B, 5'h1C: begin  // ADDF, SUBF, MULF, DIVF
                _pc_src <= 1'b0;
                _mem_write <= 1'b0;
                _reg_write <= 1'b1;
                _addr_src <= 1'b0;
                _alu_srcA <= 2'b00;
                _alu_srcB <= 2'b00;
                _alu_op <= _opcode;
            end
            5'h1D: begin  // IN
               _in_signal <= 1'b1;
               _reg_write <= 1'b1;
               _result_src <= 2'b10;
            end
            5'h1E: begin  // OUT
                _out_signal <= 1'b1;
                _alu_srcA <= 2'b00;
                _alu_pass <= 1'b1;
            end
            default: begin
                // _decode_error <= 1'b1;
            end

        endcase
    end
endmodule
