#include "vehicle/tire/tireSimple.h"

#include <cmath>

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"

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

void TireSimple::calculate(float verticalLoad, Alpha<SAE> slipAngle, float slipRatio) {
    float Fy = calculateForce(verticalLoad, true);
    torque = Torque<>(0, 0, 0);
    force = Force<>(Vec<>(0, Fy, 0), Vec<>(0, 0, 0));
}
