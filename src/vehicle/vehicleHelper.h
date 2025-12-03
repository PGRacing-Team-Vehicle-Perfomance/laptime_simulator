#pragma once

#include <cwchar>

enum DriveType {
    AWD, RWD, FWD
};

template <typename T, int C>
struct WheelBase {
    constexpr static auto WHEEL_COUNT = C;
    
    T& operator[](size_t i) {
        return _data[i];
    }

    const T& operator[](size_t i) const {
        return _data[i];
    }

    T _data[C];
};

template <typename T>
struct WheelBase<T, 4> {
    constexpr static auto WHEEL_COUNT = 4;
    
    T& operator[](size_t i) {
        return _data[i];
    }

    const T& operator[](size_t i) const {
        return _data[i];
    }

    union{
        T _data[4];

        struct{
            T fl;
            T fr;
            T rl;
            T rr;
        };
    };
};

template <typename T>
struct CarWheelBase: public WheelBase<T, 4> {};
