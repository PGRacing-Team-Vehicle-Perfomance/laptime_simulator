#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

class Vehicle
{
    CarWheelBase<Tire> tires;
    CarWheelBase<float> distributeForces(float totalForce, float frontDist, float leftDist);
    CarWheelBase<float> totalTireLoads(float velocity, float acceleration, const SimConfig& simConfig, bool isLateral);
    CarWheelBase<float> staticLoad(float earthAcc);
    CarWheelBase<float> aeroLoad(float velocity, float airDensity);
    CarWheelBase<float> loadTransfer(float acceleration, bool isLateral);

public:

    VehicleConfig config;
    Vehicle(VehicleConfig config);
    float getTireForces(float startSpeed, float acceleration, const SimConfig& simConfig, bool isLateral);
};
