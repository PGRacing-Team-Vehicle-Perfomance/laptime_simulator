#pragma once

#include "vehicle/vehicle.h"
#include "config/config.h"
#include "simulation/simulation.h"

class SkidPad : public Simulation
{
    float diameter;
    float radius;
    float trackLength;
    SkidPadConfig skidPadConfig;

public:
    SkidPad(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants, SkidPadConfig skidPadConfig);
    float run() override;
    float calculatePoints(float time, const PointsConfig &pointsConfig) const override;
};
