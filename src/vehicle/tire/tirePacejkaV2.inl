#include "vehicle/tire/tirePacejkaV2.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#define _USE_MATH_DEFINES
#include <math.h>

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"

template <typename Internal, typename External>
TirePacejkaV2<Internal, External>::TirePacejkaV2(const Config& config, bool isDriven, Side sideRelativeToVehicle)
    : Tire<Internal, External>(config, isDriven), sideRelativeToVehicle(sideRelativeToVehicle) {
    tp = config.getModule("Tire");
}

template <typename Internal, typename External>
void TirePacejkaV2<Internal, External>::MF2002(float Fz, float alpha, float gamma, float kappa) {
    float FNOMIN = tp.at("FNOMIN") * tp.at("LFZ0");
    float dfz = (Fz - FNOMIN) / FNOMIN;

    // ================= PURE LONGITUDINAL FORCE (FX) =================
    
    // Shifts
    float Shx = tp.at("PHX1") + tp.at("PHX2") * dfz * tp.at("LHX");
    float Svx = Fz * (tp.at("PVX1") + tp.at("PVX2") * dfz) * tp.at("LMUX") * tp.at("LVX");
    
    // Modified slip
    float kx = kappa + Shx;
    
    // Modified IA
    float gamma_x = gamma * tp.at("LGAX");
    
    // Shape factor
    float Cx = tp.at("PCX1") * tp.at("LCX");
    
    // Peak factor
    float Dx = Fz * (tp.at("PDX1") + tp.at("PDX2") * dfz) *
               (1.0 - tp.at("PDX3") * gamma * gamma) * tp.at("LMUX");
    
    // Longitudinal stiffness
    float Kx = Fz * (tp.at("PKX1") + tp.at("PKX2") * dfz) * exp(tp.at("PKX3") * dfz) *
               tp.at("LKX");
    
    // Stiffness factor
    float Bx = Kx / (Cx * Dx);
    
    // Curvature factor
    float Ex = (tp.at("PEX1") + tp.at("PEX2") * dfz + tp.at("PEX3") * dfz * dfz) *
               (1.0 - tp.at("PEX4") * this->sgn(kx)) * tp.at("LEX");
    
    // Magic Formula
    float Fx = Dx * sin(Cx * atan(Bx * kx - Ex * (Bx * kx - atan(Bx * kx)))) + Svx;

    // ================= LATERAL FORCE =================
    float gamma_y = gamma * tp.at("LGAY");
    float Shy = (tp.at("PHY1") + tp.at("PHY2") * dfz) * tp.at("LHY") + tp.at("PHY3") * gamma_y;
    float Svy = Fz * ((tp.at("PVY1") + tp.at("PVY2") * dfz) +
                      (tp.at("PVY3") + tp.at("PVY4") * dfz) * gamma_y) * tp.at("LMUY");
    float ay = alpha + Shy;
    float Cy = tp.at("PCY1") * tp.at("LCY");
    float Dy = Fz * (tp.at("PDY1") + tp.at("PDY2") * dfz) *
               (1.0 - tp.at("PDY3") * gamma_y * gamma_y) * tp.at("LMUY");
    float Ky = FNOMIN * tp.at("PKY1") * sin(2.0 * atan(Fz / (tp.at("PKY2") * FNOMIN))) *
               (1.0 - tp.at("PKY3") * std::fabs(gamma_y)) * tp.at("LKY") * tp.at("LFZ0");
    float By = Ky / (Cy * Dy);
    float Ey = (tp.at("PEY1") + tp.at("PEY2") * dfz) *
               (1.0 - (tp.at("PEY3") + tp.at("PEY4") * gamma_y) * this->sgn(ay)) * tp.at("LEY");
    float Fy = Dy * sin(Cy * atan(By * ay - Ey * (By * ay - atan(By * ay)))) + Svy;

    // ================= MECHANICAL TRAIL =================
    float gamma_z = gamma * tp.at("LGAZ");
    float Sht = tp.at("QHZ1") + tp.at("QHZ2") * dfz +
                (tp.at("QHZ3") + tp.at("QHZ4") * dfz) * gamma_z;
    float alpha_t = alpha + Sht;
    float Ct = tp.at("QCZ1");
    float Dt = Fz * (tp.at("QDZ1") + tp.at("QDZ2") * dfz) *
               (1.0 + tp.at("QDZ3") * std::fabs(gamma_z) + tp.at("QDZ4") * gamma_z * gamma_z) *
               tp.at("R0") / tp.at("FNOMIN") * tp.at("LTR");
    float Bt = (tp.at("QBZ1") + tp.at("QBZ2") * dfz + tp.at("QBZ3") * dfz * dfz) *
               (1.0 + tp.at("QBZ5") * std::fabs(gamma_z)) * tp.at("LKY") / tp.at("LMUY");
    float Et = (tp.at("QEZ1") + tp.at("QEZ2") * dfz + tp.at("QEZ3") * dfz * dfz) *
               (1.0 + (tp.at("QEZ4") + tp.at("QEZ5") * gamma_z) * 2 / (M_PI)*atan(Bt * Ct * alpha));
    double Shr = (tp.at("QEZ1") + tp.at("QEZ2") * dfz + tp.at("QEZ3") * dfz * dfz);
    double Set= (1.0 + (tp.at("QEZ4") + tp.at("QEZ5") * gamma_z) * (2 / (M_PI))*atan(Bt * Ct*alpha));
    
    // Limit Et to <= 1 (MF2002 standard safeguard)
    if (Et > 1.0) Et = 1.0;

    float t = Dt * std::cos(Ct * std::atan(Bt * alpha_t - Et * (Bt * alpha_t - std::atan(Bt * alpha_t))));

    // ================= RESIDUAL ALIGNING MOMENT =================
    float Shf = Shy + Svy / Ky;
    float alpha_r = alpha + Shf;
    float Br = tp.at("QBZ9") * tp.at("LKY") / tp.at("LMUY") + tp.at("QBZ10") * By * Cy;
    float Cr = 1.0;
    float Dr = Fz * tp.at("R0") *
                   ((tp.at("QDZ6") + tp.at("QDZ7") * dfz) * tp.at("LRES") +
                    (tp.at("QDZ8") + tp.at("QDZ9") * dfz) * gamma_z) * tp.at("LMUY") - 1;
    float Mzr = Dr * cos(Cr * atan(Br * alpha_r));

    // ================= FINAL RESULTS =================
    float Mz = -t * Fy + Mzr;

    float FySAE = sideRelativeToVehicle == Left ? -Fy : Fy;
    this->internalTorque = Torque<Internal>(0, 0, Mz);
    this->internalForce = Force<Internal>(Vec<Internal>(Fx, FySAE, 0), Vec<Internal>(0, 0, 0));
}

template <typename Internal, typename External>
void TirePacejkaV2<Internal, External>::calculateInternal(float verticalLoad, Alpha<Internal> slipAngle, float slipRatio) {
    float Fz = -verticalLoad;
    float alpha = sideRelativeToVehicle == Left ? -slipAngle.v : slipAngle.v;
    float gamma = 0; // camber
    float kappa = slipRatio;

    MF2002(Fz, alpha, gamma, kappa);
}
