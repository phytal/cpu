# compile and run a module
mod_%:
	iverilog -g2012 -o sim/$*.vvp mod/$*.sv
	vvp sim/$*.vvp > out/$*.out

# compile and run a testbench
test_%:
	iverilog -g2012 -o sim/$*.vvp test/$*_tb.sv
	vvp sim/$*.vvp > out/$*.out

# compile and run all testbenches
TEST_NAMES = $(basename $(notdir $(wildcard test/*_tb.sv)))
TEST_NAMES := $(foreach testname, $(TEST_NAMES), $(subst _tb,,$(testname)))
all_tests:
	echo $(TEST_NAMES)
	$(foreach testname, $(TEST_NAMES), iverilog -g2012 -o sim/$(testname).vvp test/$(testname)_tb.sv; vvp sim/$(testname).vvp > out/$(testname).out;)

eval:
	iverilog -g2012 -o sim/eval.vvp eval.sv
	vvp sim/eval.vvp > out/eval.out