# Laptime Simulator

Vehicle dynamics simulator for generating yaw moment diagrams. Simulator includes vehicle modeling with suspension, aerodynamics, and tire dynamics.

Based on: <https://github.com/BAAdas/lap-doer>

## Requirements

- C++23 or higher
- CMake 3.28+
- Python 3 (optional, for visualization scripts)

## Building

### Linux / WSL

```bash
git clone https://github.com/PGRacing-Team-Vehicle-Perfomance/laptime_simulator.git
cd laptime_simulator
make build       # Build Release natively
make run CONFIG=config_pacejka_v1.csv    # Run with a target config
```

Available make targets:

| Target    | Description            |
|-----------|------------------------|
| `build`   | Build Release (default)|
| `debug`   | Build Debug            |
| `run`     | Run                    |
| `plot`    | Run and generate plot  |
| `venv`    | Create Python environment |
| `rebuild` | Clean and rebuild      |
| `clean`   | Remove build directory |
| `help`    | Show all targets       |

Or using CMake directly:

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
./laptime_simulator ../config_pacejka_v1.csv
```

## Configuration

All parameters are loaded at runtime via CSV configuration files (see `config_pacejka_v1.csv` or `config_simple.csv`). Configuration file allows to select frame of reference for vehicle and tires. Currently ISO8855 and SAE frames are supported.

- **Vehicle**: Mass, aero, suspension parameters.
- **Tire**: Implementation selection (`Simple`, `PacejkaV1`, `PacejkaV2`), magic formula coefficients.
- **Environment**: Air density, wind conditions, etc.
- **Simulation**: Iterator boundaries, slip angle ranges, and speed.

## Output

The simulator generates CSV output files containing simulation matrices. A Python plotting script visually generates a diagram:

```bash
make plot CONFIG=config_pacejka_v1.csv
# or natively via python:
# python3 tools/plot_yaw_diagram.py build/yaw_diagram.csv
```
