#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

#ifndef SIDE_ENUM_DEFINED
#define SIDE_ENUM_DEFINED
enum Side { Left, Right };
#endif

template <typename Internal, typename External>
class TirePacejkaV1 : public Tire<Internal, External> {
    Side sideRelativeToVehicle;

    float PCY1;
    float PDY1;
    float PDY2;
    float PDY3;
    float PEY1;
    float PEY2;
    float PEY3;
    float PEY4;
    float PKY1;
    float PKY2;
    float PKY3;
    float PHY1;
    float PHY2;
    float PHY3;
    float PVY1;
    float PVY2;
    float PVY3;
    float PVY4;
    float FNOMIN;
    void calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) override;
   public:
    TirePacejkaV1() = default;
    TirePacejkaV1(const Config& config, bool isDriven, Side sideRelativeToVehicle);
};

#include "tirePacejka.inl"
