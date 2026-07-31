.PHONY: all build debug run plot setups setups-help setups-axle-params rebuild clean help venv

BUILD_DIR := build
BUILD_TYPE ?= Release
EXECUTABLE := $(BUILD_DIR)/laptime_simulator
VENV_DIR := .venv
PYTHON := $(VENV_DIR)/bin/python
CONFIG ?=
SETUP_ARGS ?= --config config_pacejka_v2.csv --param toe

all: build

build:
	@echo "=== Building laptime_simulator ($(BUILD_TYPE)) ==="
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..
	@cmake --build $(BUILD_DIR) --parallel
	@echo "=== Build complete ==="

debug:
	@$(MAKE) build BUILD_TYPE=Debug

run:
ifeq ($(CONFIG),)
	$(error CONFIG variable must be explicitly set. Example: make run CONFIG=config_pacejka_v1.csv)
endif
	@echo "=== Running laptime_simulator ==="
	@$(EXECUTABLE) $(CONFIG)

venv: $(VENV_DIR)/bin/activate

$(VENV_DIR)/bin/activate:
	@echo "=== Creating virtual environment ==="
	@python3 -m venv $(VENV_DIR)
	@$(PYTHON) -m pip install --upgrade pip
	@$(PYTHON) -m pip install matplotlib numpy

plot: run venv
	@echo "=== Generating plot ==="
	@$(PYTHON) tools/plot_yaw_diagram.py $(BUILD_DIR)/yaw_diagram.csv

setups: build venv
	@echo "=== Generating setup matrix ==="
	@$(PYTHON) tools/generate_setups.py --binary $(EXECUTABLE) $(SETUP_ARGS)

setups-help: venv
	@$(PYTHON) tools/generate_setups.py --help

setups-axle-params: venv
	@$(PYTHON) tools/generate_setups.py --list-axle-params $(SETUP_ARGS)

rebuild: clean build

clean:
	@echo "=== Cleaning build directory ==="
	@rm -rf $(BUILD_DIR)
	@echo "Done."

help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  build    Build Release (default)"
	@echo "  debug    Build Debug"
	@echo "  run      Run"
	@echo "  plot     Run and generate plot"
	@echo "  setups        Generate a setup-sweep matrix of diagrams"
	@echo "  setups-help   Show all setup-sweep options and examples"
	@echo "  setups-axle-params List the front/rear pair params usable with --param/--axle"
	@echo "  venv     Create Python virtual environment"
	@echo "  rebuild  Clean and rebuild"
	@echo "  clean    Remove build directory"
	@echo "  help     Show this help"
	@echo ""
	@echo "Examples:"
	@echo "  make"
	@echo "  make debug"
	@echo "  make run CONFIG=config_pacejka_v1.csv"
	@echo "  make plot CONFIG=config_simple.csv"
	@echo "  make setups"
	@echo "  make setups SETUP_ARGS=\"--config config_pacejka_v2.csv --param Vehicle.frontKarb --percent 20\""
