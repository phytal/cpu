module memory_load_unit (
    input wire clk,
    input wire reset,
    input wire [63:0] memory_address,
    input wire [4:0] read_reg1,
    input wire [4:0] read_reg2,
    output reg [63:0] memory_data,
    output reg data_ready
);

// Instantiate the register file module
register_file reg_file (
    .clk(clk),
    .reset(reset),
    .read_reg1(read_reg1),
    .read_reg2(read_reg2),
    .write_reg(5'b00000), // We don't write to the register file in this module
    .reg_write(1'b0),     // Disable register write in this module
    .write_data(64'h0),   // We don't write any data to the register file in this module
    .read_data1(),
    .read_data2()
);

always @(posedge clk or posedge reset)
begin
    if (reset) begin
        memory_data <= 64'h0; // Initialize memory data
        data_ready <= 1'b0;   // Reset data ready signal
    end
    else begin
        // Read operation based on memory address
        if (memory_address >= 0 && memory_address <= 31) begin
            // Valid memory address range (0-31) corresponds to register file indices
            reg_file.read_reg1 <= memory_address[4:0]; // Set read_reg1 to lower 5 bits of memory_address
            reg_file.read_reg2 <= 5'b00000;            // Set read_reg2 to zero (not used for memory load)
            memory_data <= reg_file.read_data1;         // Read data from register file
            data_ready <= 1'b1;                         // Set data ready signal
        end
        else begin
            // Invalid memory address range
            memory_data <= 64'h0;  // Reset memory data
            data_ready <= 1'b0;    // Clear data ready signal
        end
    end
end

endmodule
