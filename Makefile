.PHONY: all build debug run plot rebuild clean help venv

BUILD_DIR := build
BUILD_TYPE ?= Release
EXECUTABLE := $(BUILD_DIR)/laptime_simulator
VENV_DIR := .venv
PYTHON := $(VENV_DIR)/bin/python

all: build

build:
	@echo "=== Building laptime_simulator ($(BUILD_TYPE)) ==="
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..
	@cmake --build $(BUILD_DIR) --parallel
	@echo "=== Build complete ==="

debug:
	@$(MAKE) build BUILD_TYPE=Debug

run: build
	@echo "=== Running laptime_simulator ==="
	@$(EXECUTABLE)

venv: $(VENV_DIR)/bin/activate

$(VENV_DIR)/bin/activate:
	@echo "=== Creating virtual environment ==="
	@python3 -m venv $(VENV_DIR)
	@$(PYTHON) -m pip install --upgrade pip
	@$(PYTHON) -m pip install matplotlib

plot: run venv
	@echo "=== Generating plot ==="
	@$(PYTHON) tools/plot_yaw_diagram.py $(BUILD_DIR)/yaw_diagram.csv

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
	@echo "  run      Build and run"
	@echo "  plot     Build, run and generate plot"
	@echo "  venv     Create Python virtual environment"
	@echo "  rebuild  Clean and rebuild"
	@echo "  clean    Remove build directory"
	@echo "  help     Show this help"
	@echo ""
	@echo "Examples:"
	@echo "  make"
	@echo "  make debug"
	@echo "  make run"
