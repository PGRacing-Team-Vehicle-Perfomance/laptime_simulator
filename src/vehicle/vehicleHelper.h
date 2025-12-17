#pragma once

#include <cwchar>
#include <memory>

struct CarAcronyms {
    static constexpr unsigned int WHEEL_COUNT = 4;

    enum DriveType : size_t { AWD, RWD, FWD };

    enum wheelNames : size_t { FL, FR, RL, RR };
};

template <typename T>
struct CarWheelBase {
    T& operator[](size_t i) { return _data[i]; }

    const T& operator[](size_t i) const { return _data[i]; }

    std::array<T, CarAcronyms::WHEEL_COUNT> _data;
};
