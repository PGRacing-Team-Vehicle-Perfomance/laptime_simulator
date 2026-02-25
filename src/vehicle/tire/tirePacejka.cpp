#include "vehicle/tire/tirePacejka.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "config/config.h"

#define _USE_MATH_DEFINES
#include <math.h>

constexpr bool DEBUG_PRINT = false;

inline float sgn(float x) { return (x >= 0.0) ? 1.0 : -1.0; }

TireParams TirePacejka::loadParams(const std::string& filename) {
    TireParams tp;
    std::ifstream file(filename);
    if (!file.is_open()) throw std::runtime_error("Nie moge otworzyc pliku z parametrami.");

    std::string name;
    float value;
    while (file >> name >> value) {
        if (name[0] == '#') {
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        tp.p[name] = value;
    }
    return tp;
}

void TirePacejka::MF2002(float Fz, float alpha, float gamma, float kappa) {
    float FNOMIN = tp.p.at("FNOMIN") * tp.p.at("LFZ0");
    float dfz = (Fz - FNOMIN) / FNOMIN;

    // ================= PURE LONGITUDINAL FORCE (FX) =================

    // Shifts
    float Shx = tp.p.at("PHX1") + tp.p.at("PHX2") * dfz * tp.p.at("LHX");

    float Svx = Fz * (tp.p.at("PVX1") + tp.p.at("PVX2") * dfz) * tp.p.at("LMUX") * tp.p.at("LVX");

    // Modified slip
    float kx = kappa + Shx;

    // Modified IA
    float gamma_x = gamma * tp.p.at("LGAX");

    // Shape factor
    float Cx = tp.p.at("PCX1") * tp.p.at("LCX");

    // Peak factor
    float Dx = Fz * (tp.p.at("PDX1") + tp.p.at("PDX2") * dfz) *
               (1.0 - tp.p.at("PDX3") * gamma * gamma) * tp.p.at("LMUX");

    // Longitudinal stiffness
    float Kx = Fz * (tp.p.at("PKX1") + tp.p.at("PKX2") * dfz) * exp(tp.p.at("PKX3") * dfz) *
               tp.p.at("LKX");

    // Stiffness factor
    float Bx = Kx / (Cx * Dx);

    // Curvature factor
    float Ex = (tp.p.at("PEX1") + tp.p.at("PEX2") * dfz + tp.p.at("PEX3") * dfz * dfz) *
               (1.0 - tp.p.at("PEX4") * sgn(kx)) * tp.p.at("LEX");

    // Magic Formula
    float Fx = Dx * sin(Cx * atan(Bx * kx - Ex * (Bx * kx - atan(Bx * kx)))) + Svx;

    // ================= LATERAL FORCE =================
    float gamma_y = gamma * tp.p.at("LGAY");

    float Shy =
        (tp.p.at("PHY1") + tp.p.at("PHY2") * dfz) * tp.p.at("LHY") + tp.p.at("PHY3") * gamma_y;

    float Svy = Fz *
                ((tp.p.at("PVY1") + tp.p.at("PVY2") * dfz) +
                 (tp.p.at("PVY3") + tp.p.at("PVY4") * dfz) * gamma_y) *
                tp.p.at("LMUY");

    float ay = alpha + Shy;
    float Cy = tp.p.at("PCY1") * tp.p.at("LCY");

    float Dy = Fz * (tp.p.at("PDY1") + tp.p.at("PDY2") * dfz) *
               (1.0 - tp.p.at("PDY3") * gamma_y * gamma_y) * tp.p.at("LMUY");

    float Ky = FNOMIN * tp.p.at("PKY1") * sin(2.0 * atan(Fz / (tp.p.at("PKY2") * FNOMIN))) *
               (1.0 - tp.p.at("PKY3") * std::fabs(gamma_y)) * tp.p.at("LKY") * tp.p.at("LFZ0");

    float By = Ky / (Cy * Dy);

    float Ey = (tp.p.at("PEY1") + tp.p.at("PEY2") * dfz) *
               (1.0 - (tp.p.at("PEY3") + tp.p.at("PEY4") * gamma_y) * sgn(ay)) * tp.p.at("LEY");

    float Fy = Dy * sin(Cy * atan(By * ay - Ey * (By * ay - atan(By * ay)))) + Svy;

    // ================= MECHANICAL TRAIL =================
    float gamma_z = gamma * tp.p.at("LGAZ");

    float Sht = tp.p.at("QHZ1") + tp.p.at("QHZ2") * dfz +
                (tp.p.at("QHZ3") + tp.p.at("QHZ4") * dfz) * gamma_z;

    float alpha_t = alpha + Sht;

    float Ct = tp.p.at("QCZ1");

    float Dt = Fz * (tp.p.at("QDZ1") + tp.p.at("QDZ2") * dfz) *
               (1.0 + tp.p.at("QDZ3") * std::fabs(gamma_z) + tp.p.at("QDZ4") * gamma_z * gamma_z) *
               tp.p.at("R0") / tp.p.at("FNOMIN") * tp.p.at("LTR");

    float Bt = (tp.p.at("QBZ1") + tp.p.at("QBZ2") * dfz + tp.p.at("QBZ3") * dfz * dfz) *
               (1.0 + tp.p.at("QBZ5") * std::fabs(gamma_z)) * tp.p.at("LKY") / tp.p.at("LMUY");

    float Et =
        (tp.p.at("QEZ1") + tp.p.at("QEZ2") * dfz + tp.p.at("QEZ3") * dfz * dfz) *
        (1.0 + (tp.p.at("QEZ4") + tp.p.at("QEZ5") * gamma_z) * 2 / (M_PI)*atan(Bt * Ct * alpha));

    float Shr = (tp.p.at("QEZ1") + tp.p.at("QEZ2") * dfz + tp.p.at("QEZ3") * dfz * dfz);

    float Set = (1.0 + (tp.p.at("QEZ4") + tp.p.at("QEZ5") * gamma_z) * (2 / (M_PI)) *
                           atan(Bt * Ct * alpha));

    // Limit Et to <= 1 (MF2002 standard safeguard)
    if (Et > 1.0) Et = 1.0;

    float t =
        Dt * std::cos(Ct * std::atan(Bt * alpha_t - Et * (Bt * alpha_t - std::atan(Bt * alpha_t))));

    // ================= RESIDUAL ALIGNING MOMENT =================

    float Shf = Shy + Svy / Ky;
    float alpha_r = alpha + Shf;

    float Br = tp.p.at("QBZ9") * tp.p.at("LKY") / tp.p.at("LMUY") + tp.p.at("QBZ10") * By * Cy;

    float Cr = 1.0;

    float Dr = Fz * tp.p.at("R0") *
                   ((tp.p.at("QDZ6") + tp.p.at("QDZ7") * dfz) * tp.p.at("LRES") +
                    (tp.p.at("QDZ8") + tp.p.at("QDZ9") * dfz) * gamma_z) *
                   tp.p.at("LMUY") -
               1;

    float Mzr = Dr * cos(Cr * atan(Br * alpha_r));

    // ================= FINAL ALIGNING MOMENT =================

    float Mz = -t * Fy + Mzr;

    // ================= DEBUG PRINT =================

    if (DEBUG_PRINT) {
        std::cout << "\n=========== MF2002 DEBUG ===========\n";

        std::cout << "Fz     = " << Fz << "\n";
        std::cout << "dfz    = " << dfz << "\n\n";

        std::cout << "Cy = " << Cy << "\n";
        std::cout << "Dy = " << Dy << "\n";
        std::cout << "Ky = " << Ky << "\n";
        std::cout << "By = " << By << "\n";
        std::cout << "Ey = " << Ey << "\n\n";

        std::cout << "Ct = " << Ct << "\n";
        std::cout << "Dt = " << Dt << "\n";
        std::cout << "Bt = " << Bt << "\n";
        std::cout << "Et_base = " << Et << "\n";
        std::cout << "Et = " << Et << "\n\n";

        std::cout << "Cr = " << Cr << "\n";
        std::cout << "Dr = " << Dr << "\n";
        std::cout << "Br = " << Br << "\n\n";

        std::cout << "alpha_r = " << alpha_r << "\n";
        std::cout << "alpha_t = " << alpha_t << "\n";
        std::cout << "Shy = " << Shy << "\n";
        std::cout << "Svy = " << Svy << "\n";
        std::cout << "FNOMIN = " << FNOMIN << "\n\n";

        std::cout << "Fy  = " << Fy << "\n";
        std::cout << "Fx  = " << Fx << "\n";
        std::cout << "t   = " << t << "\n";
        std::cout << "Mzr = " << Mzr << "\n";
        std::cout << "Mz  = " << Mz << "\n";

        std::cout << "====================================\n";
    }

    force.value.y = sideRelativeToVehicle == Left ? Fy : -Fy;
    force.value.x = Fx;
    torque.z = Mz;
}

void TirePacejka::calculate(float Fz, float camber, float slipAngel, float slipRatio) {
    float alpha_deg = slipAngel;
    float gamma_deg = camber;
    float kappa = slipRatio;

    Fz = -Fz;
    float alpha = alpha_deg * M_PI / 180.0;
    float gamma = gamma_deg * M_PI / 180.0;

    MF2002(Fz, alpha, gamma, kappa);
}

TirePacejka::TirePacejka(const TireConfig& config, bool isDriven, Side sideRelativeToVehicle)
    : Tire(config, isDriven), sideRelativeToVehicle(sideRelativeToVehicle) {
    tp = loadParams(config.pacejkaParams);
}
