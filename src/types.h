#pragma once

#include <array>
#include <cmath>
#include <cwchar>

#define _USE_MATH_DEFINES
#include <math.h>

#include "coordTypes.h"

template <typename T, typename Frame>
struct Positioned {
    T value;
    Vec<Frame> position;

    Positioned() = default;
    Positioned(T value, Vec<Frame> position) : value(value), position(position) {}
};

template <typename Frame>
struct Mass : Positioned<float, Frame> {
    Mass() = default;
    Mass(float value, Vec<Frame> position) : Positioned<float, Frame>(value, position) {}
};

template <typename Frame>
struct Force : Positioned<Vec<Frame>, Frame> {
    Force() = default;
    Force(Vec<Frame> value, Vec<Frame> position) : Positioned<Vec<Frame>, Frame>(value, position) {}
};

template <typename Frame>
struct Torque : Vec<Frame> {
    Torque() = default;
    Torque(Vec<Frame> vec) : Vec<Frame>(vec) {};
    Torque(float x, float y, float z) : Vec<Frame>(x, y, z) {};
};

template <typename Frame>
class ForcefullObject {
   protected:
    Force<Frame> force;

   public:
    Force<Frame> getForce() { return force; }
};

template <typename Frame>
class TorquedObject {
   protected:
    Torque<Frame> torque;

   public:
    Torque<Frame> getTorque() { return torque; }
};
