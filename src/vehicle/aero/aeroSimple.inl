#include "vehicle/aero/aeroSimple.h"

template <typename Internal, typename External>
AeroSimple<Internal, External>::AeroSimple(const Config& config) : cla(config.get("Aero", "cla")) {}

template <typename Internal, typename External>
void AeroSimple<Internal, External>::calculateInternal(float airDensity, float speed) {
    internalForce.value.z = Z<Internal>{-0.5f * cla * airDensity * speed * speed};
}

template <typename Internal, typename External>
void AeroSimple<Internal, External>::calculate(VehicleState<External> state, float airDensity) {
    float speed = state.velocity.getLength();
    calculateInternal(airDensity, speed);
    this->force =
        Force<External>(toExternal(internalForce.value), toExternal(internalForce.position));
}
