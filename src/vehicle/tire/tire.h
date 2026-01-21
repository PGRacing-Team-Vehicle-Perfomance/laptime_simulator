#pragma once

#include "vehicle/vehicleHelper.h"
#include "config/config.h"

class Tire: public ReactiveEntity {
   protected:
    float scalingFactor;
    float quadFac;
    float linFac;
    bool isDriven;

   public:
    Tire() = default;
    Tire(const TireConfig& config, bool isDriven)
        : scalingFactor(config.scalingFac), quadFac(config.quadFac), linFac(config.linFac), isDriven(isDriven) {}
    virtual void calculate(float verticalLoad, float slipAngle, float slipRatio) = 0;
};
