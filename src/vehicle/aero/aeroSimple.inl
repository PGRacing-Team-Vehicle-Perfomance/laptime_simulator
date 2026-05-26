#include "vehicle/aero/aeroSimple.h"

template <typename Internal, typename External>
AeroSimple<Internal, External>::AeroSimple(const Config& config) : cla(config.get("Aero", "cla")) {}

template <typename Internal, typename External>
void AeroSimple<Internal, External>::calculateInternal(float airDensity, float speedSq) {
    internalForce.value.z = isoToInternal(Z<ISO8855>{-0.5f * cla * airDensity * speedSq});
}

template <typename Internal, typename External>
void AeroSimple<Internal, External>::calculate(VehicleState<External> state, float airDensity) {
    float speed = state.velocity.getLength();
    calculateInternal(airDensity, speed * speed);
    this->force = Force<External>(toExternal(internalForce.value), toExternal(internalForce.position));
}
