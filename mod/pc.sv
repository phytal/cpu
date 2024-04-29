module program_counter (
    input logic [2:0] state,
    input logic clk,
    input logic reset,
    input logic [63:0] pc_plus_4,
    input logic [63:0] pc_branch,
    input logic pc_src,
    output logic [63:0] pc_out
);
    logic [63:0] pc;
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 0;
        else if (state == 3'b000) begin
            if (pc_src == 0)
                pc <= pc_plus_4;
            else if (pc_src == 1)
                pc <= pc_branch;
        end
    end
    assign pc_out = pc;
endmodule