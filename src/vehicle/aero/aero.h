#pragma once

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"
#include "vehicle/vehicleHelper.h"

template <typename External>
class Aero : public ForcefullObject<External>, public TorquedObject<External> {
    using Internal = ISO8855;

    Transform<Internal, External> toExternal;

    Force<Internal> internalForce;
    Torque<Internal> internalTorque;

    float cla;

    void calculateInternal(float airDensity, float speedSq);

   public:
    Aero(const Config& config);
    Aero() = default;
    void calculate(VehicleState<External> state, float airDensity, Vec<External> wind = {});
};

#include "aero.inl"
