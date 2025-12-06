#pragma once

#include "vehicle/vehicle.h"
#include "config/config.h"

class Simulation
{
protected:
    SimulationConstants simulationConstants;
    SimConfig simConfig;
    Vehicle &vehicle;

public:
    Simulation(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants);
    virtual float run() = 0;
    virtual float calculatePoints(float time, const PointsConfig &pointsConfig) const = 0;
};
