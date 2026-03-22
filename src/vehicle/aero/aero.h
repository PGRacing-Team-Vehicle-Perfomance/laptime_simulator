#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

template <typename Frame>
class Aero : public ForcefullObject<Frame> {
    float cla;
    
    void calculateForces(VehicleState<Frame> state, float airDensity, Vec<Frame> wind);
    void downforce(VehicleState<Frame> state, float airDensity, Vec<Frame> wind);

   public:
    Aero(const VehicleConfig<Frame>& config);
    Aero() = default;
    void calculate(VehicleState<Frame> state, float airDensity, Vec<Frame> wind = {});
};

#include "aero.inl"
