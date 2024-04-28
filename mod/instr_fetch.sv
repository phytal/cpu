module fetch (
    input clk,
    input reset,
    input [31:0] instr_in,
    output [31:0] instr_out
);
    reg [31:0] instr_reg;

    always @(posedge clk or posedge reset) begin
        if (reset)
            instr_reg <= 32'h0;
        else
            instr_reg <= instr_in;
    end

    assign instr = instr_out;

endmodule
