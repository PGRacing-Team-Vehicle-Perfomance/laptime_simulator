#include "vehicle/tire/tirePacejka.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"

#ifndef SGN_FUNCTION_DEFINED
#define SGN_FUNCTION_DEFINED
inline double sgn(double x) { return (x >= 0.0) ? 1.0 : -1.0; }
#endif

template <typename Internal, typename External>
TirePacejkaV1<Internal, External>::TirePacejkaV1(const Config& config, bool isDriven, Side sideRelativeToVehicle)
    : Tire<Internal, External>(config, isDriven),
      sideRelativeToVehicle(sideRelativeToVehicle),
      PCY1(config.get("Tire", "PCY1")),
      PDY1(config.get("Tire", "PDY1")),
      PDY2(config.get("Tire", "PDY2")),
      PDY3(config.get("Tire", "PDY3")),
      PEY1(config.get("Tire", "PEY1")),
      PEY2(config.get("Tire", "PEY2")),
      PEY3(config.get("Tire", "PEY3")),
      PEY4(config.get("Tire", "PEY4")),
      PKY1(config.get("Tire", "PKY1")),
      PKY2(config.get("Tire", "PKY2")),
      PKY3(config.get("Tire", "PKY3")),
      PHY1(config.get("Tire", "PHY1")),
      PHY2(config.get("Tire", "PHY2")),
      PHY3(config.get("Tire", "PHY3")),
      PVY1(config.get("Tire", "PVY1")),
      PVY2(config.get("Tire", "PVY2")),
      PVY3(config.get("Tire", "PVY3")),
      PVY4(config.get("Tire", "PVY4")),
      FNOMIN(config.get("Tire", "FNOMIN")) {}

template <typename Internal, typename External>
void TirePacejkaV1<Internal, External>::calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) {
    float Fz = -verticalLoad;
    float alpha = sideRelativeToVehicle == Left ? -slipAngle.v : slipAngle.v;
    float gamma = 0;

    float dfz = (Fz - FNOMIN) / FNOMIN;
    float Sh = PHY1 + PHY2 * dfz + PHY3 * gamma;
    float Sv = Fz * ((PVY1 + PVY2 * dfz) + (PVY3 + PVY4 * dfz) * gamma);
    float ay = alpha + Sh;
    float Cy = PCY1;
    float Dy = Fz * (PDY1 + PDY2 * dfz) * (1.0 - PDY3 * gamma * gamma);
    float Ky =
        FNOMIN * PKY1 * sin(2.0 * atan(Fz / (PKY2 * FNOMIN))) * (1.0 - PKY3 * std::fabs(gamma));
    float By = Ky / (Cy * Dy);
    float Ey = (PEY1 + PEY2 * dfz) * (1.0 - (PEY3 + PEY4 * gamma) * sgn(ay));
    float Fy = Dy * sin(Cy * atan(By * ay - Ey * (By * ay - atan(By * ay)))) + Sv;

    float FySAE = sideRelativeToVehicle == Left ? -Fy : Fy;
    this->internalTorque = Torque<Internal>(0, 0, 0);
    this->internalForce = Force<Internal>(Vec<Internal>(0, FySAE, 0), Vec<Internal>(0, 0, 0));
}
