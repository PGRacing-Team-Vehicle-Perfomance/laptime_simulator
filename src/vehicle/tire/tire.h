#pragma once

class Tire
{
    float scalingFactor;
    float quadFac;
    float linFac;
    bool isDriven;
public:
    Tire() = default;
    Tire(float scalingFactor, float quadFac, float linFac, bool isDriven);
    float calculateForce(float verticalLoad, bool isLateral);
};
