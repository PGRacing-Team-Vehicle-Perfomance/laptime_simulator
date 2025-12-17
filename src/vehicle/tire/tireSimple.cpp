#include "vehicle/tire/tireSimple.h"

#include <cmath>

TireSimple::TireSimple(float scalingFactor, float quadFac, float linFac, bool isDriven)
    : Tire(scalingFactor, quadFac, linFac, isDriven) {}

float TireSimple::calculateForce(float verticalLoad, bool isLateral) {
    if (isDriven || isLateral) {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}

float TireSimple::getLongitudinalForce(float verticalLoad) {
    return calculateForce(verticalLoad, false);
}

float TireSimple::getLateralForce(float verticalLoad) { return calculateForce(verticalLoad, true); }
