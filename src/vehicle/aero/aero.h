#pragma once

#include "types.h"
#include "vehicle/vehicleHelper.h"

template <typename External>
class AeroBase : public ForcefullObject<External> {
   public:
    virtual ~AeroBase() = default;
    virtual void calculate(VehicleState<External> state, float airDensity) = 0;
};
