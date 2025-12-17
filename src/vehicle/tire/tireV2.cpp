#include "vehicle/tire/tireV2.h"

#include <cmath>

TireV2::TireV2(float scalingFactor, float quadFac, float linFac, bool isDriven)
    : Tire(scalingFactor, quadFac, linFac, isDriven) {}

float TireV2::getLateralForce(float verticalLoad) {
    // find max force

    return 0;
}

float TireV2::getLongitudinalForce(float verticalLoad) {
    // find max force

    return 0;
}

/*
float TireV2::calculateHorizontalForce(float slipRatio = 0, float verticalLoad, float
chasieSlipAngle, float steeringAngle, float targetForce)
{
    // check if target force is reacheble
    //find best angles to get best force

    if (isDriven || isLateral)
    {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}

float TireV2::calculateLongitudinallForce(float slipAngle = 0, float verticalLoad, float
targetForce)
{
    // check if target force is reacheble
    //find best angles to get best force

    if (isDriven || isLateral)
    {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}
*/