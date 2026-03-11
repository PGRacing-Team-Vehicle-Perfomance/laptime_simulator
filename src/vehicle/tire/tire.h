#pragma once

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/vehicleHelper.h"

template <typename Internal = ISO8855, typename External = ISO8855>
class Tire : public ForcefullObject<External>, public TorquedObject<External> {
    using ForcefullObject<External>::force;
    using TorquedObject<External>::torque;
    Transform<External, Internal> toInternal{FLIP_YZ};
    Transform<Internal, External> toExternal{FLIP_YZ};

   protected:
    bool isDriven;
    Force<Internal> internalForce;
    Torque<Internal> internalTorque;
    virtual void calculateInternal(float load, Alpha<Internal> slip, float slipRatio) = 0;

   public:
    Tire() = default;
    Tire(bool isDriven) : isDriven(isDriven) {}
    void calculate(float load, Alpha<External> slip, float slipRatio = 0.f) {
        calculateInternal(load, toInternal(slip), slipRatio);
        force =
            Force<External>(toExternal(internalForce.value), toExternal(internalForce.position));
        torque = Torque<External>(toExternal(internalTorque));
    }
};
