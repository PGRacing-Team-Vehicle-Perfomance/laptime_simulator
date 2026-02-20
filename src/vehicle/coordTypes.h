#pragma once

// =============================================================================
// Coordinate frames, typed axis components, and rotation-based transforms
//
// ISO 8855:  x=forward, y=left, z=up    (right-handed: fwd x left = up)
// SAE J670:  x=forward, y=right, z=down (right-handed: fwd x right = down)
//
// Relation: ISO <-> SAE = 180 deg rotation about x-axis
//   Axes:   x_SAE =  x_ISO,  y_SAE = -y_ISO,  z_SAE = -z_ISO
//   Angles: alpha_SAE = -alpha_ISO (about z, flips)
//           gamma_SAE =  gamma_ISO (about x, invariant)
//           kappa_SAE = -kappa_ISO (about y, flips)
//
// Rule: 180 deg rotation about axis A -> component along A unchanged,
//       other two components flip sign. Applies to both axes and angles.
//
// Typed vs raw:
//   Typed (X<F>, Y<F>, Z<F>, Alpha<F>, Gamma<F>, Kappa<F>):
//     -> has direction -> subject to rotation via Transform
//   Raw (float):
//     -> magnitude / scalar -> NOT subject to rotation
//     Example: VerticalLoad is magnitude (always +), passed as float.
//              Fy is directional force, passed as Y<Frame>.
// =============================================================================

// -- Frames -------------------------------------------------------------------
struct ISO8855 {};  // x=forward, y=left, z=up
struct SAE {};      // x=forward, y=right, z=down

// -- Axis components (directional vector components) --------------------------
template <typename Frame>
struct X {
    float v = 0;
    X() = default;
    explicit X(float val) : v(val) {}
};

template <typename Frame>
struct Y {
    float v = 0;
    Y() = default;
    explicit Y(float val) : v(val) {}
};

template <typename Frame>
struct Z {
    float v = 0;
    Z() = default;
    explicit Z(float val) : v(val) {}
};

// -- Angles (rotations about axes) --------------------------------------------
// Alpha = rotation about z (yaw / slip angle)
// Gamma = rotation about x (camber)
// Kappa = rotation about y (inclination)
template <typename Frame>
struct Alpha {
    float rad = 0;
    Alpha() = default;
    explicit Alpha(float r) : rad(r) {}
};

template <typename Frame>
struct Gamma {
    float rad = 0;
    Gamma() = default;
    explicit Gamma(float r) : rad(r) {}
};

template <typename Frame>
struct Kappa {
    float rad = 0;
    Kappa() = default;
    explicit Kappa(float r) : rad(r) {}
};

// -- Rotation mechanism -------------------------------------------------------
enum class Axis { X, Y, Z };

struct Rotation180 {
    Axis axis;

    // Axes: component along rotation axis -> unchanged, others -> flip
    float x(float v) const { return axis == Axis::X ? v : -v; }
    float y(float v) const { return axis == Axis::Y ? v : -v; }
    float z(float v) const { return axis == Axis::Z ? v : -v; }

    // Angles: angle about rotation axis -> unchanged, others -> flip
    float alpha(float v) const { return axis == Axis::Z ? v : -v; }  // about z
    float gamma(float v) const { return axis == Axis::X ? v : -v; }  // about x
    float kappa(float v) const { return axis == Axis::Y ? v : -v; }  // about y
};

// -- Predefined rotations -----------------------------------------------------
constexpr Rotation180 FLIP_YZ{Axis::X};  // 180 deg about x -> ISO<->SAE, Left<->Right tire

// -- Transform between frames -------------------------------------------------
template <typename From, typename To>
struct Transform {
    Rotation180 rotation;

    X<To> operator()(X<From> v) const { return X<To>{rotation.x(v.v)}; }
    Y<To> operator()(Y<From> v) const { return Y<To>{rotation.y(v.v)}; }
    Z<To> operator()(Z<From> v) const { return Z<To>{rotation.z(v.v)}; }
    Alpha<To> operator()(Alpha<From> a) const { return Alpha<To>{rotation.alpha(a.rad)}; }
    Gamma<To> operator()(Gamma<From> g) const { return Gamma<To>{rotation.gamma(g.rad)}; }
    Kappa<To> operator()(Kappa<From> k) const { return Kappa<To>{rotation.kappa(k.rad)}; }
};

constexpr Transform<ISO8855, SAE> isoToSae{FLIP_YZ};
constexpr Transform<SAE, ISO8855> saeToIso{FLIP_YZ};  // 180 deg is its own inverse
