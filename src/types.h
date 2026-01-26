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

    static Angle fromRadians(float rad) { return Angle(rad * 180.0f / M_PI); }

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

    const float getRadians() { return value / 180.0f * M_PI; }
};

template <typename T>
struct Vec3 {
    T x;
    T y;
    T z;
};

template <>
struct Vec3<float> {
    float x, y, z;

    Vec3() {};
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {};
    Vec3(float length, Angle phi, Angle theta) {
        float sinPhi = std::sin(phi.getRadians());
        float cosPhi = std::cos(phi.getRadians());
        float sinTheta = std::sin(theta.getRadians());
        float cosTheta = std::cos(theta.getRadians());

        x = length * sinPhi * cosTheta;
        y = length * sinPhi * sinTheta;
        z = length * cosPhi;
    }

    float getLength() const { return std::sqrt(x * x + y * y + z * z); }

    const Angle getPhi() const {
        float len = getLength();
        if (len == 0.0f) return Angle::fromRadians(0.0f);
        return Angle::fromRadians(std::acos(z / len));
    }

    const Angle getTheta() const {
        return Angle::fromRadians(std::atan2(y, x));
    }

    void setLength(float newLength) {
        float current = getLength();
        if (current == 0.0f) return;
        float scale = newLength / current;
        x *= scale;
        y *= scale;
        z *= scale;
    }

    void setPhi(Angle newPhi) {
        float len = getLength();
        Angle theta = getTheta();
        *this = Vec3<float>(len, newPhi, theta);
    }

    void setTheta(Angle newTheta) {
        float len = getLength();
        Angle phi = getPhi();
        *this = Vec3<float>(len, phi, newTheta);
    }
};

using Vec3f = Vec3<float>;

template <typename T>
struct Positioned {
    T value;
    Vec3f position;

    // Positioned<T> operator+(Positioned<T> rhs);
    // Positioned<T> operator-(Positioned<T> rhs);

    // Positioned<T>& operator+=(Positioned<T> rhs);
    // Positioned<T>& operator-=(Positioned<T> rhs);
};

using Mass = Positioned<float>;
using Force = Positioned<Vec3f>;
using Torque = Vec3f;

class MassiveObject {
   protected:
    Mass mass;
   public:
    Mass getMass() { return mass; }
};

class ForcefullObject {
   protected:
    Force force;
   public:
    Force getForce() { return force; }
};

class TorquedObject {
   protected:
    Torque torque;
   public:
    Torque getTorque() { return torque; }
};
