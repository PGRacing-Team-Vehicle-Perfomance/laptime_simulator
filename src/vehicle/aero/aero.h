#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Aero : public ForcefullObject<>, public MassiveObject<> {
    float cla;
    
    void calculateForces(VehicleState state, float airDensity, Vec<ISO8855> wind);
    void downforce(VehicleState state, float airDensity, Vec<ISO8855> wind);

   public:
    Aero(const VehicleConfig& config);
    Aero() = default;
    void calculate(VehicleState state, float airDensity, Vec<ISO8855> wind = {});
};
