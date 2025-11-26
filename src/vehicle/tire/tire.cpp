#include "vehicle/tire/tire.h"

#include <cmath>

Tire::Tire(float scalingFactor, float quadFac, float linFac) : scalingFactor(scalingFactor), quadFac(quadFac), linFac(linFac) {}

float Tire::calculateForce(float verticalLoad)
{
    return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
}
