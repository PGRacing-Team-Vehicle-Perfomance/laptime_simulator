#include "vehicle/tire/tireSimple.h"
#include "config/config.h"

#include <cmath>

TireSimple::TireSimple(const TireConfig& config, bool isDriven)
    : Tire(config, isDriven) {}

float TireSimple::calculateForce(float verticalLoad, bool isLateral) {
    if (isDriven || isLateral) {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}

void TireSimple::calculate(float verticalLoad, float slipAngle, float slipRatio) {
    force.y = calculateForce(verticalLoad, true);
    force.y = calculateForce(verticalLoad, false);
}