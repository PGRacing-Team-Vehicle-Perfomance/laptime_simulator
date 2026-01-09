#pragma once

#include <array>
#include <cwchar>
#include <memory>
#include <optional>

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

struct vecAmp3 {
    vec3<float> position;
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
