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

    std::unordered_map<std::string, float> tp;
    void calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) override;
   public:
    TirePacejkaV1() = default;
    TirePacejkaV1(const Config& config, bool isDriven, Side sideRelativeToVehicle);
};

#include "tirePacejkaV1.inl"
