#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Aero {
    float cla;
    vec2<float> claPosition;
    dim3Loads loads;

    vec3<float> Torques(vehicleState state, float airDensity, polarVector wind);

    float YawingTorque(vehicleState state, float airDensity, polarVector wind);
    float RollingTorque(vehicleState state, float airDensity, polarVector wind);
    float PithingTorque(vehicleState state, float airDensity, polarVector wind);

    vec3<vecAmp3> Forces(vehicleState state, float airDensity, polarVector wind);

    vecAmp3 Resistance(vehicleState state, float airDensity, polarVector wind);
    vecAmp3 SideForce(vehicleState state, float airDensity, polarVector wind);
    vecAmp3 Downforce(vehicleState state, float airDensity, polarVector wind);

   public:
    // Aero(aeroConfig);
    Aero(const VehicleConfig& config);
    Aero() = default;
    dim3Loads calculteLoads(vehicleState state, float airDensity,
                            polarVector wind = {.amplitude = 0, .angle = Angle(0)});
    dim3Loads getLoads();
};
