#pragma once

#include "vehicle/tire/tire.h"

class TireV2 : public Tire {
   public:
    TireV2() = default;
    TireV2(float scalingFactor, float quadFac, float linFac, bool isDriven);
    float getLateralForce(float verticalLoad) override;
    float getLongitudinalForce(float verticalLoad) override;
};
