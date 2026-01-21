#pragma once

#include <array>
#include <cmath>
#include <cwchar>

#define _USE_MATH_DEFINES
#include <math.h>


class Angle {
    float value;

    static float normalize(double v) {
        v = std::fmod(v, 360.0);
        return v < 0 ? v + 360.0 : v;
    }

   public:
    Angle(float v = 0) : value(normalize(v)) {}
    float get() const { return value; }

    Angle operator+(float rhs) const { return Angle(value + rhs); }
    Angle operator-(float rhs) const { return Angle(value - rhs); }

    Angle& operator+=(float rhs) {
        value = normalize(value + rhs);
        return *this;
    }
    Angle& operator-=(float rhs) {
        value = normalize(value - rhs);
        return *this;
    }

    float getRadians() { return value / 180.0f * M_PI; }
};

template <typename T>
struct Vec3 {
    T x;
    T y;
    T z;
};

struct PolarVec3 {
    float amplitude;
    Angle alfa;
    Angle ro;
};

struct VecAmp3 {
    Vec3<float> origin;
    float amplitude;
};

struct Dim3Loads {
    Vec3<VecAmp3> force;
    Vec3<float> torque;
};

struct Body {
    float mass;
    Vec3<float> position;
};

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
    Vec3<float> angular_velocity = {0, 0, 0};
    Vec3<Angle> rotation = {0, 0, 0};
};

struct LinearState {
    PolarVec3 velocity = {.amplitude = 0, .alfa = 0, .ro = 0};
    Vec3<float> position = {0, 0, 0}; // chyba nie potrzebne
};

struct VehicleState : LinearState, RotationalState {
    float deltaHeave = 0;
    float steeringAngle = 0;
};

class ReactiveEntity {
   public:
    Vec3<float> getForce() { return force; }
    Vec3<float> getPosition() { return position; }
    Vec3<float> getSelfMoment() { return selfMoment; }

   protected:
    Vec3<float> position;  // position relative to parent / object owner - vechile has a aero object
    Vec3<float> force;     // combined force with orign at entity position
    Vec3<float>
        selfMoment;  // moment on the entity - not the moment caused on paretn / object owner
};
