#pragma once

#include <cmath>

// ISO 8855: x=fwd, y=left, z=up — SAE J670: x=fwd, y=right, z=down
// Relation: 180° rotation about x (component along axis unchanged, others flip)
// Typed (X<F>, Alpha<F>...) = directional, raw float = magnitude/scalar

struct ISO8855 {};
struct SAE {};

using Default = ISO8855;

template <typename Frame, typename Tag>
struct CoordBase {
    float v = 0;
    CoordBase() = default;
    explicit CoordBase(float val) : v(val) {}
};

struct XTag {};
struct YTag {};
struct ZTag {};
struct AlphaTag{};
struct GammaTag{};
struct KappaTag{};

template <typename Frame = Default>
using X = CoordBase<Frame, XTag>;

template <typename Frame = Default>
using Y = CoordBase<Frame, YTag>;

template <typename Frame = Default>
using Z = CoordBase<Frame, ZTag>;


template <typename Frame = Default>
using Alpha = CoordBase<Frame, AlphaTag>;

template <typename Frame = Default>
using Gamma = CoordBase<Frame, GammaTag>;

template <typename Frame = Default>
using Kappa = CoordBase<Frame, KappaTag>;


template <typename Frame = Default>
struct Vec {
    X<Frame> x;
    Y<Frame> y;
    Z<Frame> z;

    Vec() = default;
    Vec(X<Frame> x, Y<Frame> y, Z<Frame> z) : x(x), y(y), z(z) {}
    Vec(float x, float y, float z) : x(x), y(y), z(z) {}

    float getLength() const { return std::sqrt(x.v * x.v + y.v * y.v + z.v * z.v); }

    void setLength(float newLength) {
        float current = getLength();
        if (current == 0.0f) {
            x.v = newLength;
            return;
        }
        float scale = newLength / current;
        x.v *= scale;
        y.v *= scale;
        z.v *= scale;
    }
};

enum class Axis { X, Y, Z };

struct Rotation180 {
    Axis axis;

    float x(float v) const { return axis == Axis::X ? v : -v; }
    float y(float v) const { return axis == Axis::Y ? v : -v; }
    float z(float v) const { return axis == Axis::Z ? v : -v; }

    float alpha(float v) const { return axis == Axis::Z ? v : -v; }
    float gamma(float v) const { return axis == Axis::X ? v : -v; }
    float kappa(float v) const { return axis == Axis::Y ? v : -v; }
};

constexpr Rotation180 FLIP_YZ{Axis::X};

template <typename From, typename To>
struct Transform {
    Rotation180 rotation;

    X<To> operator()(X<From> v) const { return X<To>{rotation.x(v.v)}; }
    Y<To> operator()(Y<From> v) const { return Y<To>{rotation.y(v.v)}; }
    Z<To> operator()(Z<From> v) const { return Z<To>{rotation.z(v.v)}; }
    Alpha<To> operator()(Alpha<From> a) const { return Alpha<To>{rotation.alpha(a.v)}; }
    Gamma<To> operator()(Gamma<From> g) const { return Gamma<To>{rotation.gamma(g.v)}; }
    Kappa<To> operator()(Kappa<From> k) const { return Kappa<To>{rotation.kappa(k.v)}; }
    Vec<To> operator()(Vec<From> v) const { return {(*this)(v.x), (*this)(v.y), (*this)(v.z)}; }
};

constexpr Transform<ISO8855, SAE> isoToSae{FLIP_YZ};
constexpr Transform<SAE, ISO8855> saeToIso{FLIP_YZ};

template <typename From, typename To>
struct FrameBridge {
    Transform<From, To> toTarget;
    Transform<To, From> fromTarget;
};

constexpr FrameBridge<ISO8855, SAE> isoSae{isoToSae, saeToIso};
