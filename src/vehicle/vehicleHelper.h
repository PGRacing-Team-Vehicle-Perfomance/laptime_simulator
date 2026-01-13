#pragma once

#include <array>
#include <cmath>
#include <cwchar>
#include <optional>

class Angle {
    float value_;

    static constexpr float normalize(double v) {
        v = std::fmod(v, 360.0);
        return v < 0 ? v + 360.0 : v;
    }

   public:
    constexpr Angle(float v = 0) : value_(normalize(v)) {}
    constexpr float get() const { return value_; }

    Angle operator+(float rhs) const { return Angle(value_ + rhs); }
    Angle operator-(float rhs) const { return Angle(value_ - rhs); }

    Angle& operator+=(float rhs) {
        value_ = normalize(value_ + rhs);
        return *this;
    }
    Angle& operator-=(float rhs) {
        value_ = normalize(value_ - rhs);
        return *this;
    }

    float getRadians() { return value_ / 180.0f * M_PI; }
};

template <typename T>
struct vec2 {
    T x;
    T y;
};

template <typename T>
struct vec3 {
    T x;
    T y;
    T z;
};

struct polarVec3 {
    float amplitude;
    vec2<Angle> angle;
};

struct vecAmp3 {
    vec3<float> origin;
    float amplitude;
};

struct dim3Loads {
    vec3<vecAmp3> force;
    vec3<float> torque;
};

struct Body {
    float mass;
    vec3<float> position;
};

struct CarAcronyms {
    static constexpr unsigned int WHEEL_COUNT = 4;

    enum wheelNames : size_t { FL, FR, RL, RR };
};

template <typename T>
struct CarWheelBase {
    T& operator[](size_t i) { return _data[i]; }

    const T& operator[](size_t i) const { return _data[i]; }

    std::array<T, CarAcronyms::WHEEL_COUNT> _data;
};

struct State {
    std::optional<polarVec3> velocity;
    std::optional<vec3<float>> angular_velocity;
    std::optional<vec3<Angle>> rotation;
    std::optional<vec3<float>> position;
};

struct vehicleState : State {
    float deltaHeave = 0;
    float steeringAngle = 0;
};
