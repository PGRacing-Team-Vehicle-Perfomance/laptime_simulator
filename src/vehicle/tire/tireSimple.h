#pragma once

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/tire/tire.h"

class SimpleModel {
   protected:
    float scalingFactor;
    float quadFac;
    float linFac;

    float computeFy(float verticalLoad);
    float computeFx(float verticalLoad, bool isDriven);

   public:
    SimpleModel() = default;
    SimpleModel(const TireConfig& config);
};

template <typename External = ISO8855>
class TireSimple : public Tire<SAE, External>, private SimpleModel {
   public:
    TireSimple() = default;
    TireSimple(const TireConfig& config, bool isDriven)
        : Tire<SAE, External>(isDriven), SimpleModel(config) {}

    void calculateInternal(float verticalLoad, Alpha<ISO8855> slipAngle, float slipRatio) override {
        float Fy = computeFy(verticalLoad);
        float Fx = computeFx(verticalLoad, this->isDriven);
        this->internalForce = Force<ISO8855>(Vec<ISO8855>(Fx, Fy, 0), Vec<ISO8855>(0, 0, 0));
        this->internalTorque = Torque<ISO8855>(0, 0, 0);
    }
};
