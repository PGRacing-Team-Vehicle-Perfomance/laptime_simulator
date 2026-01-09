#pragma once

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

class Aero {
    float cla;
    vec2<float> claPosition;
    dim3Loads loads;

    // Aero(aeroConfig);
    vec3<float> Torques(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                        float windAngleOfAttack);

    float YawingTorque(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                       float windAngleOfAttack);
    float RollingTorque(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                        float windAngleOfAttack);
    float PithingTorque(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                        float windAngleOfAttack);

    vec3<vecAmp3> Forces(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                         float windAngleOfAttack);

    vecAmp3 Resistance(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                       float windAngleOfAttack);
    vecAmp3 SideForce(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                      float windAngleOfAttack);
    vecAmp3 Downforce(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                      float windAngleOfAttack);

   public:
    Aero(const VehicleConfig& config);
    Aero() = default;
    dim3Loads calculteLoads(float velocity, float chassisSlipAngle, float airDensity,
                            float windSpeed, float windAngleOfAttack);
    dim3Loads getLoads();
};
