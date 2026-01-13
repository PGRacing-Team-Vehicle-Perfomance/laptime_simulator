#include "vehicle/aero/aero.h"

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

Aero::Aero(const VehicleConfig& config) : cla(config.cla), claPosition(config.claPosition) {}

dim3Loads Aero::calculteLoads(vehicleState state, float airDensity, polarVec3 wind) {
    torques(state, airDensity, wind);
    forces(state, airDensity, wind);

    return loads;
}
dim3Loads Aero::getLoads() { return loads; }

vec3<float> Aero::torques(vehicleState state, float airDensity, polarVec3 wind) {
    yawingTorque(state, airDensity, wind);
    rollingTorque(state, airDensity, wind);
    pithingTorque(state, airDensity, wind);

    return loads.torque;
}

float Aero::yawingTorque(vehicleState state, float airDensity, polarVec3 wind) { return 0; }
float Aero::rollingTorque(vehicleState state, float airDensity, polarVec3 wind) { return 0; }
float Aero::pithingTorque(vehicleState state, float airDensity, polarVec3 wind) { return 0; }

vec3<vecAmp3> Aero::forces(vehicleState state, float airDensity, polarVec3 wind) {
    resistance(state, airDensity, wind);
    sideForce(state, airDensity, wind);
    downforce(state, airDensity, wind);

    return loads.force;
}

vecAmp3 Aero::resistance(vehicleState state, float airDensity, polarVec3 wind) {
    loads.force.x.amplitude = 0;
    return loads.force.x;
}
vecAmp3 Aero::sideForce(vehicleState state, float airDensity, polarVec3 wind) {
    loads.force.x.amplitude = 0;
    return loads.force.x;
}
vecAmp3 Aero::downforce(vehicleState state, float airDensity, polarVec3 wind) {
    float totalForce = 0.5 * cla * airDensity * std::pow(state.velocity->amplitude, 2);
    loads.force.z.amplitude = totalForce;
    loads.force.z.origin.x = claPosition.x;
    loads.force.z.origin.y = claPosition.y;
    return loads.force.z;
}
