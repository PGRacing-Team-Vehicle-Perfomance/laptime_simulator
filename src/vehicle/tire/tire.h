#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Tire : public ForcefullObject, public TorquedObject {
   protected:
    // tires should also have spring rate or wheel rate however you want to call it
    float scalingFactor;
    float quadFac;
    float linFac;
    bool isDriven;

   public:
    Tire() = default;
    Tire(const TireConfig& config, bool isDriven)
        : scalingFactor(config.scalingFac),
          quadFac(config.quadFac),
          linFac(config.linFac),
          isDriven(isDriven) {}
    virtual void calculate(float verticalLoad, float slipAngle, float slipRatio) = 0;
};
