#pragma once

#include "vehicle/tire/tire.h"

class TireSimple : public Tire {
    float calculateForce(float verticalLoad, bool isLateral);

   public:
    TireSimple() = default;
    TireSimple(const TireConfig& config, bool isDriven);
    void calculate(float verticalLoad, float slipAngle, float slipRatio) override;
};
