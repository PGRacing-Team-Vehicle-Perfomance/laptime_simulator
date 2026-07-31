#include "vehicle/aero/aeroSimple.h"

template <typename Internal, typename External>
AeroSimple<Internal, External>::AeroSimple(const Config& config)
    : cla(config.get("Aero", "cla")), cda(config.get("Aero", "cda", 0.0f)) {}

template <typename Internal, typename External>
void AeroSimple<Internal, External>::calculateInternal(float airDensity, float speed) {
    float dynamicPressure = 0.5f * airDensity * speed * speed;
    internalForce.value.z = Z<Internal>{-dynamicPressure * cla};
    internalForce.value.x = X<Internal>{-dynamicPressure * cda};
}

template <typename Internal, typename External>
void AeroSimple<Internal, External>::calculate(VehicleState<External> state, float airDensity) {
    float speed = state.velocity.getLength();
    calculateInternal(airDensity, speed);
    this->force =
        Force<External>(toExternal(internalForce.value), toExternal(internalForce.position));
}
