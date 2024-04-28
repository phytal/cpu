`include "alu.sv"
`include "memory_load.sv"
`include "instr_decode.sv"
`include "instr_fetch.sv"
`include "load_store.sv"
`inlcude "register_file.sv"

initial
begin
    clk = 1'b0;
    forever begin
        # 5 clk = !clk; // forever, toggle the clock every 5 ns
    end
end

module processor (
    input wire clk,
    input wire reset,
    output reg [63:0] pc,
    // Other module ports...
);

always @(posedge clk or posedge reset)
begin
    if (reset)
        pc <= 64'h0; // Initialize PC to 0 on reset
    else
        pc <= next_pc; // Update PC based on control logic (e.g., branch, jump)
end

// Other processor logic...

endmodule

module simulator;
    // Clock signal
    reg clk = 0;
    always #5 clk = ~clk;

    // Reset signal
    reg reset = 0;
    initial begin
        reset = 1;
        #10 reset = 0;
    end

    // Module instances
    instr_fetch IF (.clk(clk), .reset(reset));
    instr_decode ID (.clk(clk), .reset(reset));
    alu ALU (.clk(clk), .reset(reset));
    memory_load ML (.clk(clk), .reset(reset));
    load_store LS (.clk(clk), .reset(reset));
    reg_file RF (.clk(clk), .reset(reset));

    // Connect modules
    assign ID.instr = IF.instr;
    assign ML.memory_address = ID.rs;

    assign ALU.op = ID.op;
    assign DM.instruction = ALU.instruction;
    assign LS.instruction = DM.instruction;

    // Display instruction
    always @(posedge clk) begin
        $display("Instruction: %h", IF.instruction);
    end

    // Simulation end
    initial #1000 $finish;
endmodule