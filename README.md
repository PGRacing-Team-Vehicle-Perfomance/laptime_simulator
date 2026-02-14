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
make        # Build Release
make run    # Build and run
```

Available make targets:

| Target    | Description            |
|-----------|------------------------|
| `build`   | Build Release (default)|
| `debug`   | Build Debug            |
| `run`     | Build and run          |
| `rebuild` | Clean and rebuild      |
| `clean`   | Remove build directory |
| `help`    | Show all targets       |

Or using CMake directly:

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
./laptime_simulator
```

## Configuration

Vehicle parameters, tire models, and environment conditions can be configured in `src/config/config.h`:

- **VehicleConfig**: Mass distribution, suspension geometry, track width
- **TireConfig**: Pacejka model parameters
- **EnvironmentConfig**: Air density, gravity, wind conditions

## Output

The simulator generates CSV output files containing simulation results. A Python plotting script is available:

```bash
python3 tools/plot_yaw_diagram.py build/yaw_diagram.csv
```
