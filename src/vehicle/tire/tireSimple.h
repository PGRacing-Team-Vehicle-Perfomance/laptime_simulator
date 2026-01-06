#pragma once

#include "vehicle/tire/tire.h"

class TireSimple : public Tire {
    float calculateForce(float verticalLoad, bool isLateral);

   public:
    TireSimple() = default;
    TireSimple(float scalingFactor, float quadFac, float linFac, bool isDriven);
    float getLateralForce(float verticalLoad, float slipAngle) override;
    float getLongitudinalForce(float verticalLoad) override;
    float getLateralMoment(float verticalLoad, float slipAngle) override;
};
