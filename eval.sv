`ifndef eval_bench
`define eval_bench

`include "mod/cpu.sv"
`include "macros/reset_and_load_macro.sv"

module evaluator();

    // create clk and reset top signals
    logic clk;
    logic reset;

    // cpu module
    logic halt; 
    logic in_signal;
    logic [63:0] in_data;
    logic out_signal;
    logic [63:0] out_data;

    cpu cpu_inst (
        .clk(clk),
        .reset(reset),
        .halt(halt),
        .in_signal(in_signal),
        .in_data(in_data),
        .out_signal(out_signal),
        .out_data(out_data)
    );

    // clock generator
    always #5 clk = ~clk;

    initial begin
        // ======================================================
        // reset the processor and load the tko file into the memory
        `RESET_AND_LOAD_FILE("tko/basic_add.tko", clk, reset, cpu_inst.memory.bytes);
    
        // // display the memory contents
        // for (int i = 0; i < 40; i++) begin
        //     $display("Memory[%d]: %d", i, cpu_inst.memory.bytes[i]);
        // end

       // Monitor outputs
        while (!halt) begin
        // for (int i = 0; i < 100; i++) begin
            // Display output data when out_signal is active
            if (out_signal) begin
                $display("OUTPUT DATA: %h", out_data);
            end

            // Add more monitoring as needed

            // Wait for a clock cycle
            #1;
        end
        // #100;

        // Simulation finished
        $display("Simulation finished. Halt signal received.");
        // $display("output %d\n", out_data);
        // $display("halted %h\n", halt);


        $finish;
    end

endmodule

`endif