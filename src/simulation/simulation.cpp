#include "simulation/simulation.h"

#include <cmath>

Simulation::Simulation(Vehicle &vehicle, SimConfig simConfig,
                       SimulationConstants simulationConstants)
    : vehicle(vehicle), simulationConstants(simulationConstants), simConfig(simConfig) {}
