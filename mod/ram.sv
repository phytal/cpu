`ifndef ram_module
`define ram_module

/**
* @brief 512 KB RAM module with 1 read only port and 1 read-write port
* addresses are 64-bit wide and data is 64-bit wide
* bad addresses output an error signal
* 
* @input clk: clock signal
* @input reset: reset signal
* @input r_addr: read address
* @input rw_addr: read-write address
* @input rw_data_in: read-write data input
* @input rw_write_en: write enable signal
* @output r_data_out: read data output
* @output r_error: read error signal
* @output rw_data_out: read-write data output
* @output rw_error: read-write error signal
*/
module ram (
    input logic [2:0] state,
    input clk,
    input reset,
    input [63:0] r_addr,
    input [63:0] rw_addr,
    input [63:0] rw_data_in,
    input rw_write_en,
    output logic [31:0] r_data_out,
    output logic r_error,
    output logic [63:0] rw_data_out,
    output logic rw_error
);
    // the actual memory byte vector
    parameter MEM_SIZE = 524288;
    logic [7:0] bytes [MEM_SIZE];

    assign r_data_out = {bytes[r_addr + 3], bytes[r_addr + 2], bytes[r_addr + 1], bytes[r_addr]};
    assign r_error = (r_addr > (MEM_SIZE - 4));
    assign rw_data_out = {bytes[rw_addr + 7], bytes[rw_addr + 6], bytes[rw_addr + 5], bytes[rw_addr + 4], bytes[rw_addr + 3], bytes[rw_addr + 2], bytes[rw_addr + 1], bytes[rw_addr]};
    assign rw_error = (rw_addr > (MEM_SIZE - 8));

    // update the output latches when edge of clock or reset hits
    always @(posedge clk or posedge reset) begin
        // if reset is high, clear memory and all outputs
        if (reset) begin
            for (int i = 0; i < MEM_SIZE; i++) begin
                bytes[i] <= 0;
            end
        end 

        // otherwise we are actually doing something with the memory
        else if (state == 3'b011 & rw_write_en & rw_addr <= (MEM_SIZE - 8)) begin
            if (rw_write_en) begin
                bytes[rw_addr] <= rw_data_in[7:0];
                bytes[rw_addr + 1] <= rw_data_in[15:8];
                bytes[rw_addr + 2] <= rw_data_in[23:16];
                bytes[rw_addr + 3] <= rw_data_in[31:24];
                bytes[rw_addr + 4] <= rw_data_in[39:32];
                bytes[rw_addr + 5] <= rw_data_in[47:40];
                bytes[rw_addr + 6] <= rw_data_in[55:48];
                bytes[rw_addr + 7] <= rw_data_in[63:56];
            end
        end
    end
endmodule

`endif