#pragma once

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"
#include "vehicle/aero/aero.h"
#include "vehicle/vehicleHelper.h"

template <typename Internal, typename External>
class AeroSimple : public AeroBase<External> {
    Transform<Internal, External> toExternal;
    Transform<External, Internal> toInternal;

    Force<Internal> internalForce;

    float cla = 0;

    void calculateInternal(float airDensity, float speed);

   public:
    AeroSimple() = default;
    AeroSimple(const Config& config);
    void calculate(VehicleState<External> state, float airDensity) override;
};

#include "aeroSimple.inl"
