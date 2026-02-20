#pragma once

#include "vehicle/coordTypes.h"
#include "vehicle/tire/tire.h"

// ═══════════════════════════════════════════════════════════════════════════════
// TireAdapter — safe interface to Tire models
//
// Problem: TirePacejka::calculate() only sets force.value.y.
//          force.value.x and torque.z are NEVER set → reading them is UB.
//
// Solution: This adapter calls tire.calculate() and reads ONLY force.value.y.
//           Fx and Mz are returned as 0 (no longitudinal/aligning-torque model).
//
// All values are in VehicleFrame = ISO8855Frame (x=forward, y=left, z=up).
// TirePacejka handles its internal convention (Fz negation, side flips) via its
// Side enum — we do NOT duplicate that logic here.
// ═══════════════════════════════════════════════════════════════════════════════

namespace TireAdapter {

struct Output {
    float Fx = 0;  // VehicleFrame — always 0 (no longitudinal model)
    float Fy = 0;  // VehicleFrame — from tire.getForce().value.y
    float Mz = 0;  // VehicleFrame — always 0 (no aligning torque model)
};

// Calls tire with VehicleFrame inputs. Returns forces in VehicleFrame.
// Eliminates UB: force.value.x and torque.z are never read.
inline Output call(Tire& tire, VerticalLoad<VehicleFrame> load, SlipAngle<VehicleFrame> slip,
                   float slipRatio = 0.f) {
    tire.calculate(load.N, slip.rad, slipRatio);
    return Output{
        .Fx = 0,                        // force.value.x never set by TirePacejka → 0 explicitly
        .Fy = tire.getForce().value.y,  // only safely-set field
        .Mz = 0,                        // torque.z never set by TirePacejka → 0 explicitly
    };
}

}  // namespace TireAdapter
