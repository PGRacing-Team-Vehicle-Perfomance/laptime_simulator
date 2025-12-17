#pragma once

#include "config/config.h"
#include "simulation/simulation.h"

class Acceleration : public Simulation {
    int currentGear;
    bool isShifting;
    float shiftEndTime;
    AccelerationConfig dragConfig;

   public:
    Acceleration(Vehicle &vehicle, SimConfig simConfig, SimulationConstants simulationConstants,
                 AccelerationConfig dragConfig);
    float run() override;
    float calculatePoints(float time, const PointsConfig &pointsConfig) const override;
};
