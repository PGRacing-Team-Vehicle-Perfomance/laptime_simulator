#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Aero: public ReactiveEntity {
    float cla;
    void calculateTorques(VehicleState state, float airDensity, PolarVec3 wind);

    void yawingTorque(VehicleState state, float airDensity, PolarVec3 wind);
    void rollingTorque(VehicleState state, float airDensity, PolarVec3 wind);
    void pitchingTorque(VehicleState state, float airDensity, PolarVec3 wind);

    void calculateForces(VehicleState state, float airDensity, PolarVec3 wind);

    void resistance(VehicleState state, float airDensity, PolarVec3 wind);
    void sideForce(VehicleState state, float airDensity, PolarVec3 wind);
    void downforce(VehicleState state, float airDensity, PolarVec3 wind);

   public:
    Aero(const VehicleConfig& config);
    Aero() = default;
    void calculate(VehicleState state, float airDensity,
                             PolarVec3 wind = {.amplitude = 0, .alfa = 0, .ro = 0});
};
