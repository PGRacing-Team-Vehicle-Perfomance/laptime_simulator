#pragma once

class Tire
{
    float scalingFactor;
    float quadFac;
    float linFac;

public:
    Tire() = default;
    Tire(float scalingFactor, float quadFac, float linFac);
    float calculateForce(float verticalLoad);
};
