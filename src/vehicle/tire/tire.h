#pragma once

#include "coordTypes.h"
#include "vehicle/vehicleHelper.h"
#include "config/config.h"

struct TireOutput {
    X<SAE> Fx{0};
    Y<SAE> Fy{0};
    Z<SAE> Mz{0};
};

class Tire: public ForcefullObject, public TorquedObject {
   protected:
    bool isDriven;
    TireOutput output;

   public:
    Tire() = default;
    Tire(const TireConfig& config, bool isDriven)
        : isDriven(isDriven) {}
    virtual void calculate(float verticalLoad, Alpha<SAE> slipAngle, float slipRatio) = 0;
    TireOutput getOutput() const { return output; }
};
