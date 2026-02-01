.PHONY: all build debug run rebuild clean help

BUILD_DIR := build
BUILD_TYPE ?= Release
EXECUTABLE := $(BUILD_DIR)/laptime_simulator

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
	@echo "  rebuild  Clean and rebuild"
	@echo "  clean    Remove build directory"
	@echo "  help     Show this help"
	@echo ""
	@echo "Examples:"
	@echo "  make"
	@echo "  make debug"
	@echo "  make run"
