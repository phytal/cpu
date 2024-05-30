module register_file (
    input logic [2:0] state,
    input clk,
    input reset,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input [4:0] read_reg3,
    input [4:0] write_reg,
    input write_en,
    input [63:0] write_data,
    
    output reg [63:0] read_data1,
    output reg [63:0] read_data2,
    output reg [63:0] read_data3,
    output reg [63:0] stack_pointer
);

reg [63:0] registers [31:0]; // 32 registers, each 64 bits wide

always @(posedge clk or posedge reset)
begin
    if (reset) begin
        for (int i = 0; i < 32; i = i + 1)
            registers[i] <= 64'h0; // Initialize all registers to zero
    end
    else if (state == 3'b011 && write_en) // Write to register file
        registers[write_reg] <= write_data; // Write data to specified register
    
end

// Read data from register file based on read register addresses
assign read_data1 = registers[read_reg1];
assign read_data2 = registers[read_reg2];
assign read_data3 = registers[read_reg3];
assign stack_pointer = registers[5'b11111]; // Stack pointer is register 31

endmodule