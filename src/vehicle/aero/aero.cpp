#include "vehicle/aero/aero.h"

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

Aero::Aero(const VehicleConfig& config) : cla(config.cla), claPosition(config.claPosition) {}

dim3Loads Aero::calculteLoads(vehicleState state, float airDensity, polarVector wind) {
    Torques(state, airDensity, wind);
    Forces(state, airDensity, wind);

    return loads;
}
dim3Loads Aero::getLoads() { return loads; }

vec3<float> Aero::Torques(vehicleState state, float airDensity, polarVector wind) {
    YawingTorque(state, airDensity, wind);
    RollingTorque(state, airDensity, wind);
    PithingTorque(state, airDensity, wind);

    return loads.torque;
}

float Aero::YawingTorque(vehicleState state, float airDensity, polarVector wind) { return 0; }
float Aero::RollingTorque(vehicleState state, float airDensity, polarVector wind) { return 0; }
float Aero::PithingTorque(vehicleState state, float airDensity, polarVector wind) { return 0; }

vec3<vecAmp3> Aero::Forces(vehicleState state, float airDensity, polarVector wind) {
    Resistance(state, airDensity, wind);
    SideForce(state, airDensity, wind);
    Downforce(state, airDensity, wind);

    return loads.force;
}

vecAmp3 Aero::Resistance(vehicleState state, float airDensity, polarVector wind) {
    loads.force.x.amplitude = 0;
    return loads.force.x;
}
vecAmp3 Aero::SideForce(vehicleState state, float airDensity, polarVector wind) {
    loads.force.x.amplitude = 0;
    return loads.force.x;
}
vecAmp3 Aero::Downforce(vehicleState state, float airDensity, polarVector wind) {
    float totalForce = 0.5 * cla * airDensity * std::pow(state.velocity.amplitude, 2);
    loads.force.z.amplitude = totalForce;
    loads.force.z.origin.x = claPosition.x;
    loads.force.z.origin.y = claPosition.y;
    return loads.force.z;
}
