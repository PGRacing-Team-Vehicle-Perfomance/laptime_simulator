#include "vehicle/aero/aero.h"

#include "config/config.h"
#include "vehicle/vehicleHelper.h"
#include "types.h"

template <typename Frame>
Aero<Frame>::Aero(const VehicleConfig<Frame>& config) : cla(config.cla) {}

template <typename Frame>
void Aero<Frame>::calculate(VehicleState<Frame> state, float airDensity, Vec<Frame> wind) {
    downforce(state, airDensity, wind);
}

template <typename Frame>
void Aero<Frame>::downforce(VehicleState<Frame> state, float airDensity, Vec<Frame> wind) {
    this->force.value.z = Z<Frame>{static_cast<float>(0.5 * cla * airDensity * std::pow(state.velocity.getLength(), 2))};
}
