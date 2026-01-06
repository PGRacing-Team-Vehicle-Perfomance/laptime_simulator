#pragma once

#include <array>
#include <cwchar>
#include <memory>

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
