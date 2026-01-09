#include "vehicle/aero.h"

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

Aero::Aero(const VehicleConfig& config) : cla(config.cla), claPosition(config.claPosition) {}

dim3Loads Aero::calculteLoads(float velocity, float chassisSlipAngle, float airDensity,
                              float windSpeed, float windAngleOfAttack) {
    Torques(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);
    Forces(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);

    return loads;
}
dim3Loads Aero::getLoads() { return loads; }

vec3<float> Aero::Torques(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                          float windAngleOfAttack) {
    YawingTorque(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);
    RollingTorque(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);
    PithingTorque(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);

    return loads.torque;
}

float Aero::YawingTorque(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                         float windAngleOfAttack) {
    throw std::runtime_error("Not implemented");
}
float Aero::RollingTorque(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                          float windAngleOfAttack) {
    throw std::runtime_error("Not implemented");
}
float Aero::PithingTorque(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                          float windAngleOfAttack) {
    throw std::runtime_error("Not implemented");
}

vec3<vecAmp3> Aero::Forces(float velocity, float chassisSlipAngle, float airDensity,
                           float windSpeed, float windAngleOfAttack) {
    Resistance(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);
    SideForce(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);
    Downforce(velocity, chassisSlipAngle, airDensity, windSpeed, windAngleOfAttack);

    return loads.force;
}

vecAmp3 Aero::Resistance(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                         float windAngleOfAttack) {
    throw std::runtime_error("Not implemented");
}
vecAmp3 Aero::SideForce(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                        float windAngleOfAttack) {
    throw std::runtime_error("Not implemented");
}
vecAmp3 Aero::Downforce(float velocity, float chassisSlipAngle, float airDensity, float windSpeed,
                        float windAngleOfAttack) {
    throw std::runtime_error("Not implemented");
}
