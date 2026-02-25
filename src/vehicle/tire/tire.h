#pragma once

#include "vehicle/vehicleHelper.h"
#include "config/config.h"

class Tire: public ForcefullObject, public TorquedObject {
   protected:
    bool isDriven;

   public:
    Tire() = default;
    Tire(const TireConfig& config, bool isDriven)
        : isDriven(isDriven) {}
    virtual void calculate(float Fz, float camber, float slipAngel, float slipRatio) = 0;
};
