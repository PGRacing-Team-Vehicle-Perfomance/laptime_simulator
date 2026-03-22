#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

template <typename Internal, typename External>
class TireSimple : public Tire<Internal, External> {
    float scalingFactor;
    float quadFac;
    float linFac;

    float calculateForce(float verticalLoad, bool isLateral);
    void calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) override;
   public:
    TireSimple() = default;
    TireSimple(const TireConfig& config, bool isDriven);
};

#include "tireSimple.inl"
