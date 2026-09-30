# Simple Icarus Verilog flow for the single-bank DRAM controller.
#
#   make sim    compile + run the self-checking testbench
#   make broken run the same TB against a bugged controller (must FAIL)
#   make wave   open the VCD in GTKWave (run `make sim` first)
#   make clean  remove build products

IVERILOG ?= iverilog
VVP      ?= vvp
GTKWAVE  ?= gtkwave

RTL := rtl/dram_ctrl.v
TB  := tb/tb_dram_ctrl.v

BUILD := sim/tb_dram_ctrl.vvp
VCD   := sim/dram_ctrl.vcd
BROKEN_BUILD := sim/tb_dram_ctrl_broken.vvp

.PHONY: sim wave broken clean

sim: $(BUILD)
	$(VVP) $(BUILD)

$(BUILD): $(RTL) $(TB)
	$(IVERILOG) -g2012 -Wall -o $@ $(TB) $(RTL)

wave: $(VCD)
	$(GTKWAVE) $(VCD)

broken: $(BROKEN_BUILD)
	@echo "Running the testbench against the DELIBERATELY BROKEN controller."
	@echo "Expected outcome: the checker reports tRCD violations and OVERALL: FAIL."
	-$(VVP) $(BROKEN_BUILD)

$(BROKEN_BUILD): rtl/dram_ctrl_broken.v $(TB)
	$(IVERILOG) -g2012 -Wall -DBROKEN -o $@ $(TB) rtl/dram_ctrl_broken.v

clean:
	rm -f sim/*.vvp sim/*.vcd
