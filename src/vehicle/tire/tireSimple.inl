#include "vehicle/tire/tireSimple.h"

#include <cmath>

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"

template <typename Frame>
TireSimple<Frame>::TireSimple(const TireConfig& config, bool isDriven)
    : Tire(config, isDriven),
      scalingFactor(config.scalingFac),
      quadFac(config.quadFac),
      linFac(config.linFac) {}

template <typename Frame>
float TireSimple<Frame>::calculateForce(float verticalLoad, bool isLateral) {
    if (isDriven || isLateral) {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}

template <typename Frame>
void TireSimple<Frame>::calculate(float verticalLoad, Alpha<Frame> slipAngle, float slipRatio) {
    float Fy = calculateForce(verticalLoad, true);
    torque = Torque<Frame>(0, 0, 0);
    force = Force<Frame>(Vec<Frame>(0, Fy, 0), Vec<Frame>(0, 0, 0));
}
