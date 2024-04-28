`define RESET_AND_LOAD_FILE(filepath, clk, reset, bytes) \
begin \
    int __file_handle; \
    int __code; \
    \
    clk = 0; \
    #10 \
    \
    reset = 1; \
    #10; \
    reset = 0; \
    \
    __file_handle = $fopen(filepath, "rb"); \
    \
    for (int __i = 0; __i < $size(bytes); __i++) begin \
        bytes[__i] = 0; \
    end \
    if (__file_handle) begin \
        __code = $fread(bytes, __file_handle); \
        $display("Code: %d", __code); \
        $fclose(__file_handle); \
    end else begin \
        $display("Error opening file"); \
    end \
end
