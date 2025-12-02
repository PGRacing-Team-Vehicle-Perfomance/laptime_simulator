#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

#include <array>

class Vehicle
{
    std::array<Tire, 4> tires;
    std::array<float, 4> distributeForces(float totalForce, float frontDist, float leftDist);
    std::array<float, 4> totalTireLoads(float velocity, float acceleration, SimConfig simConfig, bool isLateral);
    std::array<float, 4> staticLoad(SimConfig simConfig);
    std::array<float, 4> aeroLoad(float velocity, SimConfig simConfig);
    std::array<float, 4> loadTransfer(float acceleration, bool isLateral);

public:

    VehicleConfig config;
    Vehicle(VehicleConfig config);
    float getTireForces(float startSpeed, float acceleration, float airDensity, bool isLateral);
};
