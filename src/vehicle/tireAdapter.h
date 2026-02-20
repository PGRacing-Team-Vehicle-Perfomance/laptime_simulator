#pragma once

#include "vehicle/coordTypes.h"
#include "vehicle/tire/tire.h"

// =============================================================================
// TireAdapter — safe interface to Tire models
//
// Problem: TirePacejka::calculate() only sets force.value.y.
//          force.value.x and torque.z are NEVER set -> reading them is UB.
//
// Solution: This adapter calls tire.calculate() and reads ONLY force.value.y.
//           Fx and Mz are returned as 0 (no longitudinal/aligning-torque model).
//
// All values are in ISO8855 frame (x=forward, y=left, z=up).
// TirePacejka handles its internal convention (Fz negation, side flips) via its
// Side enum — we do NOT duplicate that logic here.
// =============================================================================

namespace TireAdapter {

struct Output {
    X<ISO8855> Fx{0};  // ISO8855 — always 0 (no longitudinal model)
    Y<ISO8855> Fy{0};  // ISO8855 — from tire.getForce().value.y
    float Mz = 0;      // always 0 (no aligning torque model)
};

// Calls tire with ISO8855 inputs. Returns forces in ISO8855.
// load: vertical load magnitude (always positive, scalar)
// slip: slip angle in ISO8855 frame
// Eliminates UB: force.value.x and torque.z are never read.
inline Output call(Tire& tire, float load, Alpha<ISO8855> slip, float slipRatio = 0.f) {
    tire.calculate(load, slip.rad, slipRatio);
    return Output{
        .Fx = X<ISO8855>{0},                        // force.value.x never set -> 0 explicitly
        .Fy = Y<ISO8855>{tire.getForce().value.y},  // only safely-set field
        .Mz = 0,                                     // torque.z never set -> 0 explicitly
    };
}

}  // namespace TireAdapter
