#pragma once

#include <array>
#include <cmath>
#include <cwchar>

#define _USE_MATH_DEFINES
#include <math.h>

#include "coordTypes.h"

template <typename T, typename VFrame>
struct Positioned {
    T value;
    Vec<VFrame> position;

    Positioned() = default;
    Positioned(T value, Vec<VFrame> position) : value(value), position(position) {}
};

template <typename VFrame>
struct Mass : Positioned<float, VFrame> {
    Mass() = default;
    Mass(float value, Vec<VFrame> position) : Positioned<float, VFrame>(value, position) {}
};

template <typename VFrame>
struct Force : Positioned<Vec<VFrame>, VFrame> {
    Force() = default;
    Force(Vec<VFrame> value, Vec<VFrame> position) : Positioned<Vec<VFrame>, VFrame>(value, position) {}
};

template <typename VFrame>
struct Torque : Vec<VFrame> {
    Torque() = default;
    Torque(float x, float y, float z) : Vec<VFrame>(x, y, z) {};
};

template <typename VFrame>
class ForcefullObject {
   protected:
    Force<VFrame> force;

   public:
    Force<VFrame> getForce() { return force; }
};

template <typename VFrame>
class TorquedObject {
   protected:
    Torque<VFrame> torque;

   public:
    Torque<VFrame> getTorque() { return torque; }
};
