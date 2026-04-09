#include "vehicle/tire/tirePacejkaV1.h"

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
      sideRelativeToVehicle(sideRelativeToVehicle) {
    tp = config.getModule("Tire");
}

template <typename Internal, typename External>
void TirePacejkaV1<Internal, External>::calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) {
    float Fz = -verticalLoad;
    float alpha = sideRelativeToVehicle == Left ? -slipAngle.v : slipAngle.v;
    float gamma = 0;

    float FNOMIN = tp.at("FNOMIN");
    float dfz = (Fz - FNOMIN) / FNOMIN;
    float Sh = tp.at("PHY1") + tp.at("PHY2") * dfz + tp.at("PHY3") * gamma;
    float Sv = Fz * ((tp.at("PVY1") + tp.at("PVY2") * dfz) + (tp.at("PVY3") + tp.at("PVY4") * dfz) * gamma);
    float ay = alpha + Sh;
    float Cy = tp.at("PCY1");
    float Dy = Fz * (tp.at("PDY1") + tp.at("PDY2") * dfz) * (1.0 - tp.at("PDY3") * gamma * gamma);
    float Ky =
        FNOMIN * tp.at("PKY1") * sin(2.0 * atan(Fz / (tp.at("PKY2") * FNOMIN))) * (1.0 - tp.at("PKY3") * std::fabs(gamma));
    float By = Ky / (Cy * Dy);
    float Ey = (tp.at("PEY1") + tp.at("PEY2") * dfz) * (1.0 - (tp.at("PEY3") + tp.at("PEY4") * gamma) * sgn(ay));
    float Fy = Dy * sin(Cy * atan(By * ay - Ey * (By * ay - atan(By * ay)))) + Sv;

    float FySAE = sideRelativeToVehicle == Left ? -Fy : Fy;
    this->internalTorque = Torque<Internal>(0, 0, 0);
    this->internalForce = Force<Internal>(Vec<Internal>(0, FySAE, 0), Vec<Internal>(0, 0, 0));
}
