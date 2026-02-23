#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Aero: public ForcefullObject, public MassiveObject {
    float cla;
    void calculateTorques(VehicleState state, float airDensity, Vec<ISO8855> wind);

    void yawingTorque(VehicleState state, float airDensity, Vec<ISO8855> wind);
    void rollingTorque(VehicleState state, float airDensity, Vec<ISO8855> wind);
    void pitchingTorque(VehicleState state, float airDensity, Vec<ISO8855> wind);

    void calculateForces(VehicleState state, float airDensity, Vec<ISO8855> wind);

    void resistance(VehicleState state, float airDensity, Vec<ISO8855> wind);
    void sideForce(VehicleState state, float airDensity, Vec<ISO8855> wind);
    void downforce(VehicleState state, float airDensity, Vec<ISO8855> wind);

   public:
    Aero(const VehicleConfig& config);
    Aero() = default;
    void calculate(VehicleState state, float airDensity,
                             Vec<ISO8855> wind = {});
};
