#pragma once

#include <array>
#include <cmath>
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

struct RotationalState {
    Vec3f angular_velocity = {0, 0, 0};
    Vec3<Angle> rotation = {0, 0, 0};
};

struct LinearState {
    Vec3f velocity = {0, 0, 0};
    Vec3f position = {0, 0, 0}; // chyba nie potrzebne
};

struct VehicleState : LinearState, RotationalState {
    float deltaHeave = 0;
    float steeringAngle = 0;
};
