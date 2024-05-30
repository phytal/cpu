module io_device (
    input [2:0] state, // State of the CPU
    input clk,
    input reset,
    input in_signal,           // Input valid signal from CPU
    input out_signal,          // Output valid signal to CPU    
    output logic clocked_in_signal, // Signal CPU that input is read
    output logic clocked_out_signal // Signal CPU that output is sent
);

// Define registers for input and output data
logic _clocked_in_signal;
logic _clocked_out_signal;

// Clocked process to handle input and output data
always @(posedge clk) begin
    if (reset) begin
        _clocked_in_signal <= 1'b0;
        _clocked_out_signal <= 1'b0;
    end else begin
        // Input handling
        if (in_signal && state == 3'b010) begin
            _clocked_in_signal <= 1'b1; // Signal CPU that input is ready
        end else begin
            _clocked_in_signal <= 1'b0;
        end
        
        // Output handling
        if (out_signal && state == 3'b010) begin
            _clocked_out_signal <= 1'b1; // Signal CPU that output is valid
        end else begin
            _clocked_out_signal <= 1'b0;
        end
    end
end

// Assign output valid signal
assign clocked_in_signal = _clocked_in_signal;
assign clocked_out_signal = _clocked_out_signal;

endmodule
