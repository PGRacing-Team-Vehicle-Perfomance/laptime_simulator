#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

#include <unordered_map>
#include <string>

template <typename Internal, typename External>
class TirePacejkaV2 : public Tire<Internal, External> {
    Side sideRelativeToVehicle;
    std::unordered_map<std::string, float> tp;

    void MF2002(float Fz, float alpha, float gamma, float kappa);
    void calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio,
                           Gamma<Internal> camber) override;

   public:
    TirePacejkaV2() = default;
    TirePacejkaV2(const Config& config, bool isDriven, Side sideRelativeToVehicle);
};

#include "tirePacejkaV2.inl"
