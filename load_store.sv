module load_store_unit (
    input wire clk,
    input wire reset,
    input wire [4:0] read_reg1,
    input wire [4:0] read_reg2,
    input wire [4:0] write_reg,
    input wire reg_write,
    input wire [63:0] write_data,
    input wire read_write_select, // 0: Read operation, 1: Write operation
    output reg [63:0] read_data,
    output reg data_ready
);

// Instantiate the register file module
register_file reg_file (
    .clk(clk),
    .reset(reset),
    .read_reg1(read_reg1),
    .read_reg2(read_reg2),
    .write_reg(write_reg),
    .reg_write(reg_write),
    .write_data(write_data),
    .read_data1(),
    .read_data2()
);

always @(posedge clk or posedge reset)
begin
    if (reset) begin
        read_data <= 64'h0; // Initialize read data
        data_ready <= 1'b0; // Reset data ready signal
    end
    else begin
        if (read_write_select == 1'b0) begin
            // Read operation
            if (reg_write && (write_reg == read_reg1)) begin
                read_data <= write_data; // Directly output the written data if reading the same register being written
            end
            else begin
                read_data <= (write_reg == 5'b00000) ? 64'h0 : reg_file.read_data1; // Read data from register file based on read_reg1
            end
            data_ready <= 1'b1; // Set data ready signal
        end
        else begin
            // Write operation
            if (reg_write && (write_reg != 5'b00000)) begin
                // Write data to register file
                reg_file.write_data <= write_data;
            end
            read_data <= 64'h0; // Reset read data for write operation
            data_ready <= 1'b0; // Clear data ready signal
        end
    end
end

endmodule
