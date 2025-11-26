#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

#include <vector>
#include <array>
#include <unordered_map>

class Vehicle
{
    std::unordered_map<std::string, Tire> tires;
    const std::vector<std::string> WHEELS = {"LF", "RF", "LR", "RR"};
    std::array<float, 4> distributeForces(float totalForce, float frontDist, float leftDist);

public:
    VehicleConfig config;
    Vehicle(VehicleConfig config, float tireScalingFactor);
    float getTireForces(float startSpeed, float acceleration, float airDensity, bool isLateral);
    std::array<float, 4> totalTireLoads(float velocity, float acceleration, SimConfig simConfig, bool isLateral);
    std::array<float, 4> staticLoad(SimConfig simConfig);
    std::array<float, 4> aeroLoad(float velocity, SimConfig simConfig);
    std::array<float, 4> loadTransfer(float acceleration, bool isLateral);
};
