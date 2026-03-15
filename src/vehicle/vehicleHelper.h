#pragma once

#include <array>
#include <cmath>
#include <cwchar>

#include "types.h"

namespace CarConstants {
static constexpr unsigned int WHEEL_COUNT = 4;
}

template <typename T>
struct WheelData {
    T FL;
    T FR;
    T RL;
    T RR;

    constexpr T& operator[](size_t i) {
        switch (i) {
            case 0:
                return FL;
            case 1:
                return FR;
            case 2:
                return RL;
            case 3:
                return RR;
        }
        return FL;
    }

    constexpr const T& operator[](size_t i) const {
        switch (i) {
            case 0:
                return FL;
            case 1:
                return FR;
            case 2:
                return RL;
            case 3:
                return RR;
        }
        return FL;
    }
};

struct VehicleState {
    Alpha<> steeringAngle;
    WheelData<Alpha<>> wheelAngles;
    Vec<> velocity;

    Vec<> angularVelocity;
    struct {
        Alpha<> z = Alpha<>(0);
    } rotation;
};
