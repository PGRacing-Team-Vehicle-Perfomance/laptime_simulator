#pragma once

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/vehicleHelper.h"

template <typename Frame = ISO8855>
class Tire : public ForcefullObject<Frame>, public TorquedObject<Frame> {
   protected:
    bool isDriven;

   public:
    Tire() = default;
    Tire(const TireConfig& config, bool isDriven) : isDriven(isDriven) {}
    virtual void calculate(float verticalLoad, Alpha<SAE> slipAngle, float slipRatio) = 0;
};
