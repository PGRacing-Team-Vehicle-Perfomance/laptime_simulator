#pragma once

#include <array>
#include <cmath>
#include <cwchar>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "types.h"

namespace CarConstants {
static constexpr unsigned int WHEEL_COUNT = 4;
}

inline constexpr float DEG_TO_RAD = M_PI / 180.0f;

enum class ArbStiffnessUnit { NewtonPerMm, NewtonPerDeg, NewtonMeterPerDeg };

enum class ArbMotionRatioUnit { Linear, Angular };

inline ArbStiffnessUnit arbStiffnessUnit(const std::string& unit) {
    static const std::unordered_map<std::string, ArbStiffnessUnit> units = {
        {"N/mm", ArbStiffnessUnit::NewtonPerMm},
        {"N/deg", ArbStiffnessUnit::NewtonPerDeg},
        {"Nm/deg", ArbStiffnessUnit::NewtonMeterPerDeg},
    };
    auto it = units.find(unit);
    if (it == units.end()) {
        throw std::runtime_error("Unknown ARB stiffness unit: " + unit);
    }
    return it->second;
}

inline ArbMotionRatioUnit arbMotionRatioUnit(const std::string& unit) {
    static const std::unordered_map<std::string, ArbMotionRatioUnit> units = {
        {"mm/mm", ArbMotionRatioUnit::Linear},
        {"deg/deg", ArbMotionRatioUnit::Angular},
    };
    auto it = units.find(unit);
    if (it == units.end()) {
        throw std::runtime_error("Unknown ARB motion ratio unit: " + unit);
    }
    return it->second;
}

inline ArbMotionRatioUnit expectedMotionRatioUnit(ArbStiffnessUnit unit) {
    return unit == ArbStiffnessUnit::NewtonPerMm ? ArbMotionRatioUnit::Linear
                                                 : ArbMotionRatioUnit::Angular;
}

inline float antiRollBarTorque(float stiffness, const std::string& stiffnessUnit, float motionRatio,
                               const std::string& motionRatioUnit, float trackWidth) {
    ArbStiffnessUnit kind = arbStiffnessUnit(stiffnessUnit);
    if (expectedMotionRatioUnit(kind) != arbMotionRatioUnit(motionRatioUnit)) {
        throw std::runtime_error("ARB stiffness unit '" + stiffnessUnit +
                                 "' is incompatible with motion ratio unit '" + motionRatioUnit +
                                 "'");
    }
    float motionRatioSquared = std::pow(motionRatio, 2);
    switch (kind) {
        case ArbStiffnessUnit::NewtonMeterPerDeg:
            return stiffness / motionRatioSquared;
        case ArbStiffnessUnit::NewtonPerDeg:
            return stiffness * trackWidth / motionRatioSquared;
        case ArbStiffnessUnit::NewtonPerMm:
            return stiffness * std::pow(trackWidth, 2) * std::tan(M_PI / 180) / motionRatioSquared;
    }
    return 0.0f;
}

enum Side { Left, Right };

template <typename T>
constexpr T mirrorBySide(T value, Side side) {
    return side == Left ? -value : value;
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

inline constexpr WheelData<Side> WHEEL_SIDE{Left, Right, Left, Right};

template <typename Frame>
struct VehicleState {
    Alpha<Frame> steeringAngle;
    WheelData<Alpha<Frame>> wheelAngles;

    Vec<Frame> velocity;
    Vec<Frame> angularVelocity;
};
