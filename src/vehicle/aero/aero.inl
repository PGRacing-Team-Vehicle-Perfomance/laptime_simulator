#include "config/config.h"
#include "coordTypes.h"
#include "types.h"
#include "vehicle/aero/aero.h"
#include "vehicle/vehicleHelper.h"

template <typename External>
Aero<External>::Aero(const Config& config) : cla(config.get("Vehicle", "cla")) {}

template <typename External>
void Aero<External>::calculateInternal(float airDensity, float speedSq) {
    internalForce.value.z = Z<Internal>{-0.5f * cla * airDensity * speedSq};
}

template <typename External>
void Aero<External>::calculate(VehicleState<External> state, float airDensity, Vec<External>) {
    calculateInternal(airDensity, std::pow(state.velocity.getLength(), 2));
    this->force.value = toExternal(internalForce.value);
    this->torque = Torque<External>(toExternal(internalTorque));
}
