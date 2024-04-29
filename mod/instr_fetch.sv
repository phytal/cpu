module fetch (
    input logic [2:0] state,
    input clk,
    input reset,
    input [31:0] instr_in,
    output [31:0] instr_out
);
    reg [31:0] instr_reg;

    always @(posedge clk or posedge reset) begin
        if (reset)
            instr_reg <= 32'h0;
        else if (state == 3'b000)
            instr_reg <= instr_in;
    end

    assign instr = instr_out;

endmodule
