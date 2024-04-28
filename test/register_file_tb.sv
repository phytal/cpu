`include "mod/register_file.sv"

module register_file_tb();

    // Parameters
    integer CLK_PERIOD = 10; // Clock period in time units
    integer RESET_TIME = 5;  // Reset duration in time units
    
    // Signals
    reg clk;
    reg reset;
    reg [4:0] read_reg1, read_reg2, read_reg3, write_reg;
    reg write_en;
    reg [63:0] write_data;
    wire [63:0] read_data1, read_data2, read_data3, stack_pointer;

    // Instantiate the register file module
    register_file dut (
        .clk(clk),
        .reset(reset),
        .read_reg1(read_reg1),
        .read_reg2(read_reg2),
        .read_reg3(read_reg3),
        .write_reg(write_reg),
        .write_en(write_en),
        .write_data(write_data),
        .read_data1(read_data1),
        .read_data2(read_data2),
        .read_data3(read_data3),
        .stack_pointer(stack_pointer)
    );

    // Clock generation
    always #5 clk = ~clk;

    // Reset generation
    initial begin
        reset = 1;
        #RESET_TIME reset = 0;
        #RESET_TIME;
        reset = 1;
    end

    // Test case 1: Write to register 3, read from register 3
    initial begin
        $display("Test case 1: Write to register 3, read from register 3");
        write_reg = 5'b00011;
        write_data = 64'h123456789ABCDEF0;
        write_en = 1;
        #20; // Wait for a few clock cycles
        write_en = 0;
        read_reg1 = 5'b00011;
        #10;
        $display("Read data from register 3: %h", read_data1);
        $finish;
    end

    // Add more test cases as needed...

endmodule
