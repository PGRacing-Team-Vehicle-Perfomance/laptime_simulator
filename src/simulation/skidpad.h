#pragma once

#include "config/config.h"
#include "vehicle/vehicle.h"

class SkidPad {
    Vehicle vehicle;

    float errDelta;
    int maxIterConv;
    float diameter;
    float speed;

    float airDensity;
    float earthAcc;

   public:
    SkidPad(const VehicleConfig& vehicle, const SkidPadConfig& skidPadConfig,
            const EnvironmentConfig& environmentConfig);
};
