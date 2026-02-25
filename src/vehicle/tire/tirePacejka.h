#pragma once

#include "config/config.h"
#include "vehicle/tire/tire.h"

#include <unordered_map>
#include <string>

enum Side { Left, Right };

struct TireParams {
    std::unordered_map<std::string, float> p;
};

class TirePacejka : public Tire {
    Side sideRelativeToVehicle;
    TireParams tp;

    TireParams loadParams(const std::string& filename);
    void MF2002(float Fz, float alpha, float gamma, float kappa);

   public:
    TirePacejka() = default;
    TirePacejka(const TireConfig& config, bool isDriven, Side sideRelativeToVehicle);
    void calculate(float Fz, float camber, float slipAngel, float slipRatio) override;
};
