#pragma once

// ═══════════════════════════════════════════════════════════════════════════════
// Canonical coordinate system: ISO8855Frame (= VehicleFrame)
//
//   x — forward (front axle at x=0, rear axle at x=trackDistance)
//   y — leftward (ISO 8855: FL/RL at y=+t/2, FR/RR at y=-t/2)
//   z — upward
//
//   SlipAngle    > 0  → wheel velocity has leftward (+y) component vs heading
//   LateralForce > 0  → leftward (+y)
//   VerticalLoad > 0  → normal force magnitude (always positive, reaction force)
//   YawMoment    > 0  → CCW from above (nose goes left)
//   LatAcc       > 0  → leftward (+y)
//
// SAEFrame: SAE J670 tire convention (used internally by TirePacejka)
//   x=forward, y=right, z=down  (right-handed: forward×right=down ✓)
//   SlipAngle > 0 → velocity to the RIGHT of wheel heading (CW about z=down)
//                   opposite to ISO where α>0 = velocity to the LEFT
//   Fy        > 0 → rightward (+y)
//   Fz        > 0 → compressed (load INTO tire, z=down so downward = positive)
//
// Conversions ISO8855 ↔ SAE:
//   toSAE(α):  negate rad  (ISO α>0=left, SAE α>0=right)
//   toSAE(Fz): keep N as-is (both use magnitude/reaction convention)
//   toISO(Fy): negate N    (SAE Fy>0=right, ISO Fy>0=left)
//
// Note: TirePacejka handles its own Side flips (Left/Right) internally.
//       The adapter passes values directly — TirePacejka's internals absorb
//       the remaining ISO→SAE conversion via the sign chain in slip angle calc.
// ═══════════════════════════════════════════════════════════════════════════════

struct ISO8855Frame {};  // x=fwd, y=left, z=up  (canonical vehicle frame)
struct SAEFrame {};      // x=fwd, y=right, z=down  (SAE J670: forward×right=down)

// VehicleFrame is an alias for ISO8855Frame — used throughout the codebase.
using VehicleFrame = ISO8855Frame;

template <typename Frame>
struct SlipAngle {
    float rad;
    explicit SlipAngle(float r) : rad(r) {}
};

template <typename Frame>
struct LateralForce {
    float N;
    explicit LateralForce(float n) : N(n) {}
};

template <typename Frame>
struct VerticalLoad {
    float N;
    explicit VerticalLoad(float n) : N(n) {}
};

enum class TireSide { Left, Right };

// ── Conversions at the ISO8855Frame ↔ SAEFrame boundary ─────────────────────
// These are the ONLY place where coordinate system knowledge is encoded.

// ISO → SAE: negate slip angle (ISO α>0=left, SAE α>0=right)
inline SlipAngle<SAEFrame> toSAE(SlipAngle<ISO8855Frame> a) { return SlipAngle<SAEFrame>(-a.rad); }

// ISO → SAE: vertical load magnitude is unchanged
inline VerticalLoad<SAEFrame> toSAE(VerticalLoad<ISO8855Frame> l) {
    return VerticalLoad<SAEFrame>(l.N);
}

// SAE → ISO: negate lateral force (SAE Fy>0=right, ISO Fy>0=left)
inline LateralForce<ISO8855Frame> toISO(LateralForce<SAEFrame> f) {
    return LateralForce<ISO8855Frame>(-f.N);
}
