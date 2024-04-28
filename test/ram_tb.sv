`include "mod/ram.sv"

module ram_tb;
    // parameters
    parameter ADDR_WIDTH = 64;
    parameter DATA_WIDTH = 64;
    
    // Signals
    logic clk;
    logic mem_rst;
    logic [ADDR_WIDTH-1:0] address;
    logic [DATA_WIDTH-1:0] data_in;
    logic mem_we;
    wire [DATA_WIDTH-1:0] data_out;

    // RAM module
    ram memory (
        .clk(clk),
        .reset(mem_rst),
        .r_addr(),
        .rw_addr(address),
        .rw_data_in(data_in),
        .rw_write_en(mem_we),
        .r_data_out(),
        .r_error(),
        .rw_data_out(data_out),
        .rw_error()
    );
    
    // clock 
    always #5 clk = ~clk;
    
    task init_and_reset();
        // initialize signals
        address = 0;
        data_in = 0;
        mem_we = 0;
        clk = 0;
        #10;
        
        // reset memory
        mem_rst = 1;
        #10;
        mem_rst = 0;
    endtask

    // actual testbench
    initial begin
        // hyperparameters
        static int REPS = 65536;
        static int SUCCESS = 0;

        init_and_reset();    

        // write to RAM
        for (int i = 0; i < REPS; i++) begin
            address = i * 8;
            data_in = i * 333 + 123;
            mem_we = 1;
            #10;
        end
        
        // read from RAM
        for (int i = 0; i < REPS; i++) begin
            address = i * 8;
            mem_we = 0;
            #10;
            if (data_out != i * 333 + 123) begin
                $display("Data at address %d: %d", address, data_out);
            end else begin
                SUCCESS++;
            end
        end
        
        // display success rate
        $display("Success rate: %d/%-d", SUCCESS, REPS);

        // done
        $finish;
    end
endmodule