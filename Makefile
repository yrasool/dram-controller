# Icarus Verilog flow for the DRAM controller project.
#
#   make sim      single-bank controller, self-checking testbench
#   make mb       multi-bank + FR-FCFS controller, self-checking testbench
#   make compare  sweep bank count / queue depth over one address stream
#   make broken   run the TB against a bugged controller (must FAIL)
#   make all      everything that must pass, in order
#   make wave     open the single-bank VCD in GTKWave
#   make wave-mb  open the multi-bank VCD in GTKWave
#   make clean    remove build products

IVERILOG ?= iverilog
VVP      ?= vvp
GTKWAVE  ?= gtkwave
IVFLAGS  ?= -g2012

NREQ ?= 500
NB   ?= 4
QD   ?= 8

RTL    := rtl/dram_ctrl.v
TB     := tb/tb_dram_ctrl.v
RTL_MB := rtl/dram_ctrl_mb.v
TB_MB  := tb/tb_dram_ctrl_mb.v

BUILD        := sim/tb_dram_ctrl.vvp
BUILD_MB     := sim/tb_dram_ctrl_mb.vvp
BROKEN_BUILD := sim/tb_dram_ctrl_broken.vvp
VCD          := sim/dram_ctrl.vcd
VCD_MB       := sim/dram_ctrl_mb.vcd

.PHONY: all sim mb compare broken wave wave-mb clean

all: sim mb compare broken
	@echo ""
	@echo "All stages complete."

# ---- single-bank -------------------------------------------------------
sim: $(BUILD)
	$(VVP) $(BUILD)

$(BUILD): $(RTL) $(TB)
	$(IVERILOG) $(IVFLAGS) -Wall -o $@ $(TB) $(RTL)

# ---- multi-bank + FR-FCFS ----------------------------------------------
mb: $(BUILD_MB)
	$(VVP) $(BUILD_MB)

$(BUILD_MB): $(RTL_MB) $(TB_MB)
	$(IVERILOG) $(IVFLAGS) -DVCD -DNB=$(NB) -DQD=$(QD) -DNREQ=$(NREQ) -o $@ $(TB_MB) $(RTL_MB)

# ---- configuration sweep ------------------------------------------------
compare:
	@NREQ=$(NREQ) bash sim/compare.sh

# ---- Stage 3: the checker must catch a real bug -------------------------
broken: $(BROKEN_BUILD)
	@echo "Running the testbench against the DELIBERATELY BROKEN controller."
	@echo "Expected outcome: the checker reports tRCD violations and OVERALL: FAIL."
	@if $(VVP) $(BROKEN_BUILD) | grep -q "OVERALL: FAIL"; then \
		echo "  -> checker caught the injected bug, as required."; \
	else \
		echo "  -> ERROR: the checker did NOT catch the injected bug."; exit 1; \
	fi

$(BROKEN_BUILD): rtl/dram_ctrl_broken.v $(TB)
	$(IVERILOG) $(IVFLAGS) -Wall -DBROKEN -o $@ $(TB) rtl/dram_ctrl_broken.v

# ---- waveforms -----------------------------------------------------------
wave: $(VCD)
	$(GTKWAVE) $(VCD)

wave-mb: $(VCD_MB)
	$(GTKWAVE) $(VCD_MB)

clean:
	rm -f sim/*.vvp sim/*.vcd
