#include "vehicle/tire/tireSimple.h"

#include <cmath>

#include "config/config.h"

TireSimple::TireSimple(const TireConfig& config, bool isDriven)
    : Tire(config, isDriven),
      scalingFactor(config.scalingFac),
      quadFac(config.quadFac),
      linFac(config.linFac) {}

float TireSimple::calculateForce(float verticalLoad, bool isLateral) {
    if (isDriven || isLateral) {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}

void TireSimple::calculate(float verticalLoad, float slipAngle, float slipRatio) {
    force.value.y = calculateForce(verticalLoad, true);
    // force.value.x = calculateForce(verticalLoad, false);
}
