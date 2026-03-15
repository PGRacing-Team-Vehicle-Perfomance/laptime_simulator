#include "vehicle/aero/aero.h"

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

Aero::Aero(const VehicleConfig& config) : cla(config.cla) {}

void Aero::calculate(VehicleState state, float airDensity, Vec<ISO8855> wind) {
    calculateTorques(state, airDensity, wind);
    calculateForces(state, airDensity, wind);
}

void Aero::calculateTorques(VehicleState state, float airDensity, Vec<ISO8855> wind) {
    yawingTorque(state, airDensity, wind);
    rollingTorque(state, airDensity, wind);
    pitchingTorque(state, airDensity, wind);
}

void Aero::yawingTorque(VehicleState state, float airDensity, Vec<ISO8855> wind) { return; }
void Aero::rollingTorque(VehicleState state, float airDensity, Vec<ISO8855> wind) { return; }
void Aero::pitchingTorque(VehicleState state, float airDensity, Vec<ISO8855> wind) { return; }

void Aero::calculateForces(VehicleState state, float airDensity, Vec<ISO8855> wind) {
    resistance(state, airDensity, wind);
    sideForce(state, airDensity, wind);
    downforce(state, airDensity, wind);
}

void Aero::resistance(VehicleState state, float airDensity, Vec<ISO8855> wind) {}
void Aero::sideForce(VehicleState state, float airDensity, Vec<ISO8855> wind) {
}
void Aero::downforce(VehicleState state, float airDensity, Vec<ISO8855> wind) {
    force.value.z = Z<ISO8855>{static_cast<float>(0.5 * cla * airDensity * std::pow(state.velocity.getLength(), 2))};
}
