#pragma once

#include <cmath>

constexpr float minSolveForce = 0.5f;
constexpr float slipRatioScanStep = 0.015f;
constexpr int slipRatioScanSteps = 20;
constexpr float goldenSectionInvPhi = 0.618033988f;
constexpr int goldenSectionIterations = 10;
constexpr int forceBisectionIterations = 18;

struct BranchPeak {
    float slipRatio;
    float force;
};

template <typename ForceFn>
inline BranchPeak ascendingBranchPeak(ForceFn forceAt, float direction) {
    float peakSlipRatio = 0;
    float peakForce = forceAt(0.0f);
    for (int scan = 1; scan <= slipRatioScanSteps; scan++) {
        float slipRatio = direction * slipRatioScanStep * scan;
        float force = forceAt(slipRatio);
        if (direction * force > direction * peakForce) {
            peakForce = force;
            peakSlipRatio = slipRatio;
        }
    }
    float low = peakSlipRatio - direction * slipRatioScanStep;
    float high = peakSlipRatio + direction * slipRatioScanStep;
    float c = high - (high - low) * goldenSectionInvPhi;
    float d = low + (high - low) * goldenSectionInvPhi;
    float fc = forceAt(c);
    float fd = forceAt(d);
    for (int iter = 0; iter < goldenSectionIterations; iter++) {
        if (direction * fc > direction * fd) {
            high = d;
            d = c;
            fd = fc;
            c = high - (high - low) * goldenSectionInvPhi;
            fc = forceAt(c);
        } else {
            low = c;
            c = d;
            fc = fd;
            d = low + (high - low) * goldenSectionInvPhi;
            fd = forceAt(d);
        }
    }
    float slipRatio = (low + high) * 0.5f;
    float force = forceAt(slipRatio);
    // never report a peak worse than the best scanned sample
    if (direction * force < direction * peakForce) return {peakSlipRatio, peakForce};
    return {slipRatio, force};
}

// solve the slip ratio on the ascending branch producing `target`, reusing a known peak
template <typename ForceFn>
inline float ascendingBranchSlipRatio(ForceFn forceAt, float target, const BranchPeak& knownPeak,
                                      float knownPeakDirection) {
    float zeroForce = forceAt(0.0f);
    if (std::abs(target - zeroForce) < minSolveForce) return 0;
    float direction = target > zeroForce ? 1.0f : -1.0f;

    BranchPeak peak =
        direction == knownPeakDirection ? knownPeak : ascendingBranchPeak(forceAt, direction);
    if (direction * target >= direction * peak.force) return peak.slipRatio;

    float lo = 0.0f;
    float hi = peak.slipRatio;
    for (int iter = 0; iter < forceBisectionIterations; iter++) {
        float mid = (lo + hi) * 0.5f;
        if (direction * forceAt(mid) < direction * target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return (lo + hi) * 0.5f;
}

template <typename ForceFn>
inline float ascendingBranchSlipRatio(ForceFn forceAt, float target) {
    return ascendingBranchSlipRatio(forceAt, target, BranchPeak{0.0f, 0.0f}, 0.0f);
}
