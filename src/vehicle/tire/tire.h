#pragma once

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/vehicleHelper.h"

template <typename Internal, typename External>
class Tire : public ForcefullObject<External>, public TorquedObject<External> {
    Transform<External, Internal> toInternal;
    Transform<Internal, External> toExternal;

   protected:
    bool isDriven;
    Force<Internal> internalForce;
    Torque<Internal> internalTorque;

    virtual void calculateInternal(float load, Alpha<Internal> slip, float slipRatio) = 0;
    
   public:
    Tire() = default;
    Tire(const TireConfig& config, bool isDriven) : isDriven(isDriven) {}
    void calculate(float verticalLoad, Alpha<External> slipAngle, float slipRatio) {
        calculateInternal(verticalLoad, toInternal(slipAngle), slipRatio);
        this->force = Force<External>(toExternal(internalForce.value), toExternal(internalForce.position));
        this->torque = Torque<External>(toExternal(internalTorque));
    };
};
