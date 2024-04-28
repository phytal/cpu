module control_unit (
    input logic clk,
    input logic reset,
    input [31:0] instr,
    // input logic [4:0] opcode,
    // input logic [4:0] rd,
    // input logic [4:0] rs,
    // input logic [4:0] rt,
    output logic [4:0] opcode,
    output logic [4:0] rd,
    output logic [4:0] rs,
    output logic [4:0] rt,
    output logic [11:0] L,
    output logic pc_src,
    output logic result_src,
    output logic mem_write,
    output logic reg_write,
    // output logic immediate_src,
    output logic addr_src, // for data movement
    output logic [1:0] alu_srcA, // 0: rs, 1: rd, 2: pc, 3: sp
    output logic [1:0] alu_srcB, // 0: rt, 1: immediate, 2: pc
    output logic [4:0] alu_op, 
    output logic alu_pass,
    output logic halt,
    output logic in_signal,
    output logic out_signal
);
    // Internal signals to hold instruction fields
    logic [4:0] _rs;
    logic [4:0] _rt;
    logic [4:0] _rd;
    logic [4:0] _opcode;
    logic [11:0] _L;
    logic _pc_src;
    logic _result_src;
    logic _mem_write;
    logic _reg_write;
    // logic _immediate_src;
    logic _addr_src;
    logic [1:0] _alu_srcA;
    logic [1:0] _alu_srcB;
    logic [4:0] _alu_op;
    logic _alu_pass;
    logic _halt;
    logic _in_signal;
    logic _out_signal;

    assign opcode = _opcode;
    assign rd = _rd;
    assign rs = _rs;
    assign rt = _rt;
    assign L = _L;
    assign pc_src = _pc_src;
    assign result_src = _result_src;
    assign mem_write = _mem_write;
    assign reg_write = _reg_write;
    // assign immediate_src = _immediate_src;
    assign addr_src = _addr_src;
    assign alu_srcA = _alu_srcA;
    assign alu_srcB = _alu_srcB;
    assign alu_op = _alu_op;
    assign alu_pass = _alu_pass;
    assign halt = _halt;
    assign in_signal = _in_signal;
    assign out_signal = _out_signal;

    // // Define control signals based on opcode
    // always_comb begin
    // Decode the instruction and perform the corresponding operation on positive edge of clock
    // always_ff @(posedge clk or posedge reset) begin
    // always_ff @(posedge clk or posedge reset) begin
    always @(*) begin
        // Resetting control signals
        _pc_src <= 1'b0;
        _mem_write <= 1'b0;
        _reg_write <= 1'b0;
        // immediate_src <= 1'b0;
        _addr_src <= 1'b0;
        _alu_srcA <= 2'b00;
        _alu_srcB <= 2'b00;
        _alu_op <= 5'b00000;
        _alu_pass <= 1'b0;
        _halt <= 1'b0;
        _in_signal <= 1'b0;
        _out_signal <= 1'b0;

        // Extracting opcode
        _opcode <= instr[31:27];

        // Extracting register specifiers
        _rd <= instr[26:22];
        _rs <= instr[21:17];
        _rt <= instr[16:12];

        // Extracting literal
        _L <= $signed(instr[11:0]);
// ====================================================================================================
        case (_opcode)
            // Integer and Logic Arithmetic Instructions
            5'h00, 5'h02, 5'h04, 5'h05, 5'h06, 5'h07, 5'h08, 5'h09, 5'h0a, 5'h0c: begin  // R-Type
                $display("R-Type");
                _pc_src <= 1'b0;
                _mem_write <= 1'b0;
                _reg_write <= 1'b1;
                // immediate_src <= 1'b0;
                _addr_src <= 1'b0;
                _alu_srcA <= 2'b00;
                _alu_srcB <= 2'b00;
                _alu_op <= _opcode;
            end
            5'h01, 5'h03, 5'h0b, 5'h0d: begin  // L-Type
                $display("L-Type");
                _pc_src <= 1'b0;
                _mem_write <= 1'b0;
                _reg_write <= 1'b1;
                // immediate_src <= 1'b1;
                _addr_src <= 1'b0;
                _alu_srcA <= 2'b00;
                _alu_srcB <= 2'b01;
                _alu_op <= _opcode; 
            end
// ====================================================================================================
            // Control Instructions
            5'h0E: begin  // BR, pc = rd
                // immediate_src = 1'b0;
                _alu_srcA <= 2'b01;
                _alu_srcB <= 2'b00;
                _alu_pass <= 1'b1;
                _pc_src <= 1'b1;
            end
            5'h0F: begin  // BRR, pc = rd + pc
                $display("BRR");
                // immediate_src = 1'b0;
                _alu_srcA <= 2'b01;
                _alu_srcB <= 2'b10;
                _alu_pass <= 1'b0;
                _alu_op <= 5'b00000;
                _pc_src <= 1'b1;
            end
            5'h10: begin  // BRR L pc = pc + L
                // immediate_src = 1'b1;
                _alu_srcA <= 2'b10;
                _alu_srcB <= 2'b01;
                _alu_pass <= 1'b0;
                _alu_op <= 5'b00000;
                _pc_src <= 1'b1;
            end
            5'h11: begin  // BRNZ pc = (rs == 0) ? (pc + 4) : rd
                // immediate_src <= 1'b0;
                // if (rs_data == 0) begin
                //     alu_srcA <= 2'b10;
                //     alu_srcB <= 2'b01;
                //     alu_pass <= 1'b0;
                // end else begin
                //     alu_srcA <= 2'b01;
                //     alu_pass <= 1'b1;
                // end
            end
            5'h12: begin  // CALL mem[sp-8] = pc, pc = rd
                // immediate_src <= 1'b0;
                
            end
            5'h13: begin  // RETURN pc = mem[sp-8]
                // immediate_src <= 1'b0;
            end
            5'h14: begin  // BRGT pc = rs <= rt? pc + 4 : rd
                // immediate_src <= 1'b0;

            end
            5'h1F: begin  // HALT
                _halt <= 1'b1;
            end
// ====================================================================================================
            // Data Movement Instructions
            5'h15: begin  // MOV rd ← Mem[rs + L]
                _reg_write <= 1'b1;
                _alu_srcA <= 2'b00;
                _alu_srcB <= 2'b01;
                _alu_op <= 5'b00000;
                _alu_pass <= 1'b0;


            end
            5'h16: begin  // MOV rd ← rs
                _reg_write <= 1'b1;
                _alu_srcA <= 2'b00;
                _alu_pass <= 1'b1;

            end
            5'h17: begin  // MOV rd [52:63] ← L
                _reg_write <= 1'b1;
                _alu_srcB <= 2'b01;
                _alu_pass <= 1'b1;

            end
            5'h18: begin  // MOV Mem[rd +L] ← rs
                // Control signals for Data Movement Instructions Control Unit
            end
// ====================================================================================================
            5'h19: begin  // ADDF
                // Control signals for Floating Point Instructions Control Unit
            end
            5'h1A: begin  // SUBF
                // Control signals for Floating Point Instructions Control Unit
            end
            5'h1B: begin  // MULF
                // Control signals for Floating Point Instructions Control Unit
            end
            5'h1C: begin  // DIVF
                // Control signals for Floating Point Instructions Control Unit
            end
            5'h1D: begin  // IN
               _in_signal <= 1'b1;
               _alu_pass <= 1'b1;
               _alu_srcA <= 2'b00;
            end
            5'h1E: begin  // OUT
                _out_signal <= 1'b1;
            end
            default: begin
                // Default case
            end

        endcase
    end
endmodule
