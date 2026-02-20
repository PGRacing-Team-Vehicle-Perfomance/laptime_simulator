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

void TireSimple::calculate(float verticalLoad, Alpha<SAE> slipAngle, float slipRatio) {
    force = {};
    torque = {};
    float Fy = calculateForce(verticalLoad, true);
    output = TireOutput{.Fx = X<SAE>{0}, .Fy = Y<SAE>{Fy}, .Mz = 0};
    force.value.y = Fy;
}
