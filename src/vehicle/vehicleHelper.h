#pragma once

#include <array>
#include <cwchar>

#include "types.h"

struct CarAcronyms {
    static constexpr unsigned int WHEEL_COUNT = 4;

    enum wheelNames : size_t { FL, FR, RL, RR };
};

template <typename T>
struct WheelData {
    T& operator[](size_t i) { return _data[i]; }

    const T& operator[](size_t i) const { return _data[i]; }

    std::array<T, CarAcronyms::WHEEL_COUNT> _data;
};

struct VehicleState {
    Vec3f angular_velocity = {0, 0, 0};
    Vec3Angle rotation = {0, 0, 0};

    Vec3f position = {0, 0, 0};
    Vec3f velocity = {0, 0, 0};

    float& heave() { return position.z; };
    Angle& pitch() { return rotation.y; };
    Angle& roll() { return rotation.x; };
    Angle& chassisSlipAngle() { return rotation.z; };
    float& yawRate() { return angular_velocity.z; };

    float steeringAngle = 0;
};
