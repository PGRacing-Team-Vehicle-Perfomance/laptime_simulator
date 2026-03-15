#include "vehicle/aero/aero.h"

#include "config/config.h"
#include "vehicle/vehicleHelper.h"

Aero::Aero(const VehicleConfig& config) : cla(config.cla) {}

void Aero::calculate(VehicleState state, float airDensity, Vec<> wind) {
    calculateForces(state, airDensity, wind);
}

void Aero::calculateForces(VehicleState state, float airDensity, Vec<> wind) {
    downforce(state, airDensity, wind);
}

void Aero::downforce(VehicleState state, float airDensity, Vec<> wind) {
    force.value.z = Z<>{static_cast<float>(0.5 * cla * airDensity * std::pow(state.velocity.getLength(), 2))};
}
