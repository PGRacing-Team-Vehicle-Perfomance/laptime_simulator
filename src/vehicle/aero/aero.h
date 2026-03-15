#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Aero : public ForcefullObject<> {
    float cla;
    
    void calculateForces(VehicleState state, float airDensity, Vec<> wind);
    void downforce(VehicleState state, float airDensity, Vec<> wind);

   public:
    Aero(const VehicleConfig& config);
    Aero() = default;
    void calculate(VehicleState state, float airDensity, Vec<> wind = {});
};
