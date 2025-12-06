#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

class Vehicle
{
    VehicleConfig config;

    CarWheelBase<Tire> tires;
    CarWheelBase<float> distributeForces(float totalForce, float frontDist, float leftDist);
    CarWheelBase<float> totalTireLoads(float velocity, float acceleration, const SimConfig &simConfig, bool isLateral);
    CarWheelBase<float> staticLoad(float earthAcc);
    CarWheelBase<float> aeroLoad(float velocity, float airDensity);
    CarWheelBase<float> loadTransfer(float acceleration, bool isLateral);
    float getEngineTorque(float rpm);
    float getWheelTorque(float engine_torque, int gear);

public:
    Vehicle(VehicleConfig config);
    float getTireForces(float startSpeed, float acceleration, const SimConfig &simConfig, bool isLateral);
    float speedToRpm(float speed_ms, int gear);
    float getPowerThrust(float speed_ms, int gear);

    float getMass();
    float getCRR();
    float getCDA();
    float getMaxTorqueRpm();
    unsigned int getGearCount();
    float getShiftTime();
};
