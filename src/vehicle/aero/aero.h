#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Aero {
    float cla;
    vec2<float> claPosition;
    dim3Loads loads;

    vec3<float> torques(vehicleState state, float airDensity, polarVec3 wind);

    float yawingTorque(vehicleState state, float airDensity, polarVec3 wind);
    float rollingTorque(vehicleState state, float airDensity, polarVec3 wind);
    float pitchingTorque(vehicleState state, float airDensity, polarVec3 wind);

    vec3<vecAmp3> forces(vehicleState state, float airDensity, polarVec3 wind);

    vecAmp3 resistance(vehicleState state, float airDensity, polarVec3 wind);
    vecAmp3 sideForce(vehicleState state, float airDensity, polarVec3 wind);
    vecAmp3 downforce(vehicleState state, float airDensity, polarVec3 wind);

   public:
    // Aero(aeroConfig);
    Aero(const VehicleConfig& config);
    Aero() = default;
    dim3Loads calculateLoads(vehicleState state, float airDensity,
                             polarVec3 wind = {.amplitude = 0, .angle = Angle(0)});
    dim3Loads getLoads();
};
