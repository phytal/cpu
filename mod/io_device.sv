module io_device (
    input clk,
    input reset,
    input [63:0] in_data,      // Input data from CPU
    input in_signal,           // Input valid signal from CPU
    input out_signal,          // Output valid signal to CPU    
    output logic in_ready,      // Input ready signal to CPU
    output logic [63:0] out_data, // Output data to CPU
    output logic out_ready       // Output valid signal from CPU
);

// Define registers for input and output data
reg [63:0] input_register;
reg [63:0] output_register;
reg out_valid_reg;

// Clocked process to handle input and output data
always @(posedge clk) begin
    if (reset) begin
        input_register <= 64'h0;
        output_register <= 64'h0;
        out_valid_reg <= 1'b0;
    end else begin
        // Input handling
        if (in_signal) begin
            input_register <= in_data;
            in_ready <= 1'b1; // Signal CPU that input is ready
        end
        
        // Output handling
        if (out_signal) begin
            out_data <= output_register;
            out_valid_reg <= 1'b1; // Signal CPU that output is valid
        end
    end
end

// Assign output valid signal
assign out_ready = out_valid_reg;

endmodule
