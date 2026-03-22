#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

enum Side { Left, Right };

template <typename Internal, typename External>
class TirePacejka : public Tire<Internal, External> {
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
    TirePacejka() = default;
    TirePacejka(const TireConfig& config, bool isDriven, Side sideRelativeToVehicle);
};

#include "tirePacejka.inl"
