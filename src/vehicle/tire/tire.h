#pragma once

class Tire {
   protected:
    float scalingFactor;
    float quadFac;
    float linFac;
    bool isDriven;

   public:
    Tire() = default;
    Tire(float scalingFactor, float quadFac, float linFac, bool isDriven)
        : scalingFactor(scalingFactor), quadFac(quadFac), linFac(linFac), isDriven(isDriven) {}
    virtual float getLateralForce(float verticalLoad, float slipAngle) = 0;
    virtual float getLongitudinalForce(float verticalLoad) = 0;
    virtual float getLateralMoment(float verticalLoad, float slipAngle) = 0;
};
