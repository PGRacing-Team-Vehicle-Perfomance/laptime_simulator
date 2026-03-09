#include "vehicle/tire/tirePacejka.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

#include "config/config.h"

inline double sgn(double x) { return (x >= 0.0) ? 1.0 : -1.0; }

TirePacejka::TirePacejka(const TireConfig& config, bool isDriven, Side sideRelativeToVehicle)
    : Tire(config, isDriven),
      sideRelativeToVehicle(sideRelativeToVehicle),
      PCY1(config.PCY1),
      PDY1(config.PDY1),
      PDY2(config.PDY2),
      PDY3(config.PDY3),
      PEY1(config.PEY1),
      PEY2(config.PEY2),
      PEY3(config.PEY3),
      PEY4(config.PEY4),
      PKY1(config.PKY1),
      PKY2(config.PKY2),
      PKY3(config.PKY3),
      PHY1(config.PHY1),
      PHY2(config.PHY2),
      PHY3(config.PHY3),
      PVY1(config.PVY1),
      PVY2(config.PVY2),
      PVY3(config.PVY3),
      PVY4(config.PVY4),
      FNOMIN(config.FNOMIN) {}

void TirePacejka::calculate(float verticalLoad, float slipAngle, float slipRatio) {
    //if(verticalLoad > 2800) std::cout<<"gowno/n"<<std::endl;
    float Fz = -verticalLoad;                                               // [N]
    float alpha = sideRelativeToVehicle == Left ? slipAngle : -slipAngle;  // [rad]
    float gamma = 0;                                                       // [rad]

    // --- Normalized load deviation
    float dfz = (Fz - FNOMIN) / FNOMIN;

    // --- Shifts
    float Sh = PHY1 + PHY2 * dfz + PHY3 * gamma;
    float Sv = Fz * ((PVY1 + PVY2 * dfz) + (PVY3 + PVY4 * dfz) * gamma);

    // --- Slip argument
    float ay = alpha + Sh;

    // --- Shape factor
    float Cy = PCY1;

    // --- Peak factor
    float Dy = Fz * (PDY1 + PDY2 * dfz) * (1.0 - PDY3 * gamma * gamma);

    // --- Cornering stiffness
    float Ky = FNOMIN * PKY1 * sin(2.0 * atan(Fz / (PKY2 * FNOMIN))) * (1.0 - PKY3 * std::fabs(gamma));

    // --- Stiffness factor
    float By = Ky / (Cy * Dy);

    // --- Curvature factor
    float Ey = (PEY1 + PEY2 * dfz) * (1.0 - (PEY3 + PEY4 * gamma) * sgn(ay));

    // --- Magic Formula
    float Fy = Dy * sin(Cy * atan(By * ay - Ey * (By * ay - atan(By * ay)))) + Sv;
    force.value.y = sideRelativeToVehicle == Left? Fy: -Fy;
}