# Project 4 Skeleton

## File structure
- `macros/` contains defined macros to help with code reuse, since icarus verilog doesn't support describing signals by reference 
- `mod/` contains your modules for the cpu
- `test/` contains the testbenches for your modules for the cpu. if named as `[modname]_tb.sv`, then you can use `make test_[modname]` to run the testbench
- `sim/` should contain the compiled vvp code
- `out/` should contain the output of the testbenches
- `tko/` contains the tko files for the testbenches
- `vcd/` contains the vcd dump files for the testbenches

## Running the testbenches
To run the testbenches, you can use the `make` command. For example, to run the testbench for the `alu` module, you can use `make test_alu`. This will compile the testbench, place the vvp in `sim/` and run the simulation. The output will be stored in the `out/` directory.

## Loading tko files
Use the macro in your testbench to load the tko file. This will reset the cpu and load the tko file into the memory. The macro is defined in `macros/reset_and_load_tko.sv`.

## Key things for submissions
You may change everything except 
- the module definition of the cpu
- the name of the ram module in cpu (keep it named "memory")
- the byte array within the ram module (keep it named "bytes" and keep it the same size)
- the memory should be accesible by cpu_inst.memory.bytes (see eval.sv)

Make sure that your cpu module is located at `mod/cpu.sv`.

Your code will be compiled with something along the lines of `iverilog -o eval.vvp eval.sv` and run with `vvp eval.vvp`. We will write our own eval module to test your cpu module. Make sure that we can use the `` `include `` directive to include your cpu module in our eval module.

We will use iverilog version 11.0