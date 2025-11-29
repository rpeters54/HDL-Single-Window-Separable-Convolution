# Find paths to RTL and include files
RTL_SRCS 	 := $(shell find rtl -name '*.sv' -or -name '*.v')
INCLUDE_DIRS := $(sort $(dir $(shell find . -name '*.svh' -or -name '*.vh')))
RTL_DIRS	 := $(sort $(dir $(RTL_SRCS)))

# Include both Include and RTL directories for linting
LINT_INCLUDES := $(foreach dir, $(INCLUDE_DIRS) $(RTL_DIRS), -I$(realpath $(dir)))

TEST_DIR = ./tests
TEST_SUBDIRS = $(shell cd $(TEST_DIR) && ls -d */ | grep -v "__pycache__" )
TESTS = $(TEST_SUBDIRS:/=)

FORMAL_DIR = ./formal
FORMAL_SUBDIRS = $(shell cd $(FORMAL_DIR) && ls -d */ | grep -v "__pycache__" )
FORMAL_TESTS = $(FORMAL_SUBDIRS:/=)

FW_DIR = ./fw
FW_SUBDIRS = $(shell cd $(FW_DIR) && ls -d */ | grep -v "__pycache__" )
FW_TESTS = $(FW_SUBDIRS:/=)

# Main Linter and Simulatior is Verilator
LINTER := verilator
SIMULATOR := verilator
SIMULATOR_ARGS := --binary --timing --trace --trace-structs --assert --timescale 1ns --quiet  
SIMULATOR_BINARY := ./obj_dir/V*
SIMULATOR_RUNNER := 
SIMULATOR_SRCS := *.sv

# Firmware Compilation Args
CC := riscv32-unknown-elf-gcc
CC_ARGS := -nostdlib
CC_LINKER_SCRIPT := link.ld
CC_SRCS_CMD := find . -maxdepth 1 -name "*.s" -o -name "*.c"
OBJCOPY := riscv32-unknown-elf-objcopy
OBJCOPY_ARGS := -O verilog --verilog-data-width=4 --change-addresses -0x80000000

# Formal Verification with SBY
VERIFIER := sby
VERIFIER_ARGS := -f
SBY_JOB_TYPE ?= bmc
JOB_TYPES = bmc prove cover

# Optional use of Icarus as Linter and Simulator
ifdef ICARUS
SIMULATOR := iverilog
SIMULATOR_ARGS := -g2012 -grelative-include
SIMULATOR_BINARY := a.out
SIMULATOR_RUNNER := vvp
SIMULATOR_SRCS = $(foreach src, $(RTL_SRCS), $(realpath $(src))) $@.sv
SIM_TOP = -s $@ 
endif

LINT_OPTS += --lint-only --timing $(LINT_INCLUDES)

# Text formatting for tests
BOLD  = `tput bold`
GREEN = `tput setaf 2`
ORANG = `tput setaf 214`
RED   = `tput setaf 1`
RESET = `tput sgr0`

TEST_GREEN := $(shell tput setaf 2)
TEST_ORANGE := $(shell tput setaf 214)
TEST_RED := $(shell tput setaf 1)
TEST_RESET := $(shell tput sgr0)

all: lint_all tests fw

lint: lint_all

.PHONY: lint_all
lint_all: 
	@printf "\n$(GREEN)$(BOLD) ----- Linting All Modules ----- $(RESET)\n"
	@for src in $(RTL_SRCS); do \
		top_module=$$(basename $$src .sv); \
		top_module=$$(basename $$top_module .v); \
		printf "Linting $$src . . . "; \
		if $(LINTER) $(LINT_OPTS) --top-module $$top_module $$src > /dev/null 2>&1; then \
			printf "$(GREEN)PASSED$(RESET)\n"; \
		else \
			printf "$(RED)FAILED$(RESET)\n"; \
			$(LINTER) $(LINT_OPTS) --top-module $$top_module $$src; \
		fi; \
	done

.PHONY: lint_top
lint_top:
	@printf "\n$(GREEN)$(BOLD) ----- Linting $(TOP_MODULE) ----- $(RESET)\n"
	@printf "Linting Top Level Module: $(TOP_FILE)\n";
	$(LINTER) $(LINT_OPTS) --top-module $(TOP_MODULE) $(TOP_FILE)

tests: $(TESTS) 

tests/%: FORCE
	make -s $(subst /,, $(basename $*))

itests: 
	@ICARUS=1 make tests

itests/%: FORCE
	@ICARUS=1 make -s $(subst /,, $(basename $*))


fw: $(FW_TESTS)

fw/%: FORCE
	make -s $(subst /,, $(basename $*))

ifw:
	@ICARUS=1 make fw

ifw/%: FORCE
	@ICARUS=1 make -s $(subst /,, $(basename $*))

formal: $(FORMAL_TESTS)

formal/%: FORCE
	make -s $(subst /,, $(basename $*))

# Simulation test targets
.PHONY: $(TESTS)
$(TESTS):
	@printf "\n$(GREEN)$(BOLD) ----- Running Test: $@ ----- $(RESET)\n"
	@printf "\n$(BOLD) Building with $(SIMULATOR)... $(RESET)\n"

# Build With Simulator
	@cd $(TEST_DIR)/$@; \
		$(SIMULATOR) $(SIMULATOR_ARGS) $(LINT_INCLUDES) $(SIM_TOP) $(SIMULATOR_SRCS) >> build.log

	@printf "\n$(BOLD) Running... $(RESET)\n"

# Run Binary and Check for Error in Result
	@if cd $(TEST_DIR)/$@;\
    	$(SIMULATOR_RUNNER) ./$(SIMULATOR_BINARY) > results.log \
    	&& !( cat results.log | grep -qi error ) \
    	then \
    		printf "$(GREEN) PASSED $@$(RESET)\n"; \
    	else \
        	printf "$(RED) FAILED $@$(RESET)\n"; \
        	cat results.log; \
    	fi; \

# Simulation test targets
.PHONY: $(FW_TESTS)
$(FW_TESTS):
	@printf "\n$(GREEN)$(BOLD) ----- Running Firmware Test: $@ ----- $(RESET)\n"

# Compiling Firmware .hex
	@printf "\n$(BOLD) Compiling with $(CC)... $(RESET)\n"
	@cd $(FW_DIR)/$@; \
		$(CC) $(CC_ARGS) -T $(CC_LINKER_SCRIPT) -o $@.elf $$($(CC_SRCS_CMD)) >> fw.log
	@cd $(FW_DIR)/$@; \
		$(OBJCOPY) $(OBJCOPY_ARGS) $@.elf $@.hex >> hex.log

# Build With Simulator
	@printf "\n$(BOLD) Building with $(SIMULATOR)... $(RESET)\n"
	@cd $(FW_DIR)/$@; \
		$(SIMULATOR) $(SIMULATOR_ARGS) $(LINT_INCLUDES) $(SIM_TOP) $(SIMULATOR_SRCS) >> build.log

	@printf "\n$(BOLD) Running... $(RESET)\n"

# Run Binary and Check for Error in Result
	@if cd $(FW_DIR)/$@;\
    	$(SIMULATOR_RUNNER) ./$(SIMULATOR_BINARY) > results.log \
    	&& !( cat results.log | grep -qi error ) \
    	then \
    		printf "$(GREEN) PASSED $@$(RESET)\n"; \
    	else \
        	printf "$(RED) FAILED $@$(RESET)\n"; \
        	cat results.log; \
    	fi; \


# Formal verification test targets
.PHONY: $(FORMAL_TESTS)
$(FORMAL_TESTS):
	@printf "\n$(GREEN)$(BOLD) ----- Running Formal Verif: $@ ----- $(RESET)\n"

# Run and check for error
	@printf "\n$(BOLD) Running with job type: $(SBY_JOB_TYPE)... $(RESET)\n"
	@if cd $(FORMAL_DIR)/$@;\
		$(VERIFIER) $(VERIFIER_ARGS) $@.sby $(SBY_JOB_TYPE) > results.log \
    	&& (cat results.log | grep -qi "PASS") \
    	then \
    		printf "$(GREEN) PASSED $@$(RESET)\n"; \
    	else \
        	printf "$(RED) FAILED $@$(RESET)\n"; \
        	cat results.log; \
    	fi; \

FORCE: ;

.PHONY: clean
clean:
	rm -f  `find tests -iname "*.vcd"`
	rm -f  `find tests -iname "a.out"`
	rm -f  `find tests -iname "*.log"`
	rm -rf `find tests -iname "obj_dir"`

	rm -f  `find fw -iname "*.elf"`
	rm -f  `find fw -iname "*.hex"`
	rm -f  `find fw -iname "*.vcd"`
	rm -f  `find fw -iname "a.out"`
	rm -f  `find fw -iname "*.log"`
	rm -rf `find fw -iname "obj_dir"`

	rm -f `find formal -iname "*.log"`
	$(foreach formal_test,$(FORMAL_TESTS),$(foreach job_type,$(JOB_TYPES),rm -rf `find formal/$(formal_test) -mindepth 1 -iname "$(formal_test)_$(job_type)"`;))
	$(foreach formal_test,$(FORMAL_TESTS),rm -rf `find formal/$(formal_test) -mindepth 1 -iname "$(formal_test)"`;)


