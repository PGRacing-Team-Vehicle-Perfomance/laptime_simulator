# Laptime Simulator

Vehicle dynamics simulator for Formula Student competition analysis on Skidpad and Acceleration events.

Based on: <https://github.com/BAAdas/lap-doer>

## Requirements

- C++23 or higher
- CMake 3.28+

## Building

### Linux / WSL

```bash
git clone https://github.com/PGRacing-Team-Vehicle-Perfomance/laptime_simulator.git
cd gfr_sim
mkdir build && cd build
cmake ..
cmake --build .
./gfr_sim
```

### Windows

```bash
git clone https://github.com/PGRacing-Team-Vehicle-Perfomance/laptime_simulator.git
```

- Open Visual Studio
- Click File -> Open -> CMake and select the top level CMakeLists.txt
- If you do not have it, install "Linux, macOS, and embedded development with C++" (most likely you will be prompted by the IDE to do so, if not go to Visual Studio Installer and download the package manually)

## Contributing

After cloning repository run:

```bash
pre-commit install
```

to enforce correct code formatting before every commit. If you do not have pre-commit run:

```bash
pip install pre-commit
```

or on WSL(Debian based):

```bash
sudo apt install pre-commit
```
