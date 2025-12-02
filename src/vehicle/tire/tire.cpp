#include "vehicle/tire/tire.h"

#include <cmath>

Tire::Tire(float scalingFactor, float quadFac, float linFac, bool isDriven) : scalingFactor(scalingFactor), quadFac(quadFac), linFac(linFac), isDriven(isDriven) {}

float Tire::calculateForce(float verticalLoad, bool isLateral)
{
    if (isDriven || isLateral)
    {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);   
    }
    return 0;
}
