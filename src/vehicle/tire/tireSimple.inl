#include "vehicle/tire/tireSimple.h"

#include <cmath>

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"

template <typename Internal, typename External>
TireSimple<Internal, External>::TireSimple(const Config& config, bool isDriven)
    : Tire<Internal, External>(config, isDriven),
      scalingFactor(config.get("Tire", "scalingFac")),
      quadFac(config.get("Tire", "quadFac")),
      linFac(config.get("Tire", "linFac")) {}

template <typename Internal, typename External>
float TireSimple<Internal, External>::calculateForce(float verticalLoad, bool isLateral) {
    if (this->isDriven || isLateral) {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}

template <typename Internal, typename External>
void TireSimple<Internal, External>::calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) {
    float Fy = calculateForce(verticalLoad, true);
    this->internalTorque = Torque<Internal>(0, 0, 0);
    this->internalForce = Force<Internal>(Vec<Internal>(0, Fy, 0), Vec<Internal>(0, 0, 0));
}
