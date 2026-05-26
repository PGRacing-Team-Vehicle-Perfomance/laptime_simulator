#pragma once

#include <cmath>

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/vehicleHelper.h"

template <typename External>
class TireBase : public ForcefullObject<External>, public TorquedObject<External> {
  protected:
    inline double sgn(double x) { return (x >= 0.0) ? 1.0 : -1.0; }
  public:
    virtual ~TireBase() = default;
    virtual void calculate(float verticalLoad, Alpha<External> slipAngle, float slipRatio,
                           Gamma<External> camber) = 0;
};

template <typename Internal, typename External>
class Tire : public TireBase<External> {
    Transform<External, Internal> toInternal;
    Transform<Internal, External> toExternal;

   protected:
    static constexpr float MIN_VERTICAL_LOAD = 1.0f;

    bool isDriven;
    Force<Internal> internalForce;
    Torque<Internal> internalTorque;

    virtual void calculateInternal(float load, Alpha<Internal> slip, float slipRatio,
                                   Gamma<Internal> camber) = 0;

   public:
    Tire() = default;
    Tire(const Config& config, bool isDriven) : isDriven(isDriven) {}
    void calculate(float verticalLoad, Alpha<External> slipAngle, float slipRatio,
                   Gamma<External> camber) override {
        if (std::fabs(verticalLoad) < MIN_VERTICAL_LOAD) {
            this->force = Force<External>(Vec<External>(0, 0, 0), Vec<External>(0, 0, 0));
            this->torque = Torque<External>(0, 0, 0);
            return;
        }
        calculateInternal(verticalLoad, toInternal(slipAngle), slipRatio, toInternal(camber));
        this->force = Force<External>(toExternal(internalForce.value), toExternal(internalForce.position));
        this->torque = Torque<External>(toExternal(internalTorque));
    }
};
