#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

#include <unordered_map>
#include <string>

// Assuming Side is already defined in tirePacejka.h, but just in case:
#ifndef SIDE_ENUM_DEFINED
#define SIDE_ENUM_DEFINED
enum Side { Left, Right };
#endif

template <typename Internal, typename External>
class TirePacejkaV2 : public Tire<Internal, External> {
    Side sideRelativeToVehicle;
    std::unordered_map<std::string, float> tp;

    void MF2002(float Fz, float alpha, float gamma, float kappa);
    void calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) override;

   public:
    TirePacejkaV2() = default;
    TirePacejkaV2(const Config& config, bool isDriven, Side sideRelativeToVehicle);
};

#include "tirePacejkaV2.inl"
