#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

template <typename Frame>
class TireSimple : public Tire<Frame> {
    float scalingFactor;
    float quadFac;
    float linFac;

    float calculateForce(float verticalLoad, bool isLateral);

   public:
    TireSimple() = default;
    TireSimple(const TireConfig& config, bool isDriven);
    void calculate(float verticalLoad, Alpha<Frame> slipAngle, float slipRatio) override;
};

#include "tireSimple.inl"
