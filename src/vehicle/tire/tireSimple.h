#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

class TireSimple : public Tire {
    float scalingFactor;
    float quadFac;
    float linFac;
    
    float calculateForce(float verticalLoad, bool isLateral);

   public:
    TireSimple() = default;
    TireSimple(const TireConfig& config, bool isDriven);
    void calculate(float verticalLoad, Alpha<SAE> slipAngle, float slipRatio) override;
};
