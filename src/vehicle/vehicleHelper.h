#pragma once

#include <cwchar>

enum DriveType {
    AWD, RWD, FWD
};

namespace Wheel
{
    static constexpr size_t FL = 0;
    static constexpr size_t FR = 1;
    static constexpr size_t RL = 2;
    static constexpr size_t RR = 3;

    static constexpr size_t COUNT = 4;
}
