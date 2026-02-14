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

struct RotationalState {
    Vec3f angular_velocity = {0, 0, 0};
    Vec3<Angle> rotation = {0, 0, 0};
};

struct LinearState {
    Vec3f velocity = {0, 0, 0};
    Vec3f position = {0, 0, 0};
};

struct VehicleState : LinearState, RotationalState {
    float deltaHeave = 0;
    float steeringAngle = 0;
};
