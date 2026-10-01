#pragma once

#include <algorithm>
#include <cmath>
#include <functional>

#include "coordTypes.h"
#include "vehicle/slipSolve.h"

template <typename Frame>
struct AxleWheel {
    std::function<X<Frame>(float)> forceAt;
};

struct AxleSlipRatios {
    float left;
    float right;
};

template <typename Frame>
inline AxleSlipRatios equalForceSolve(X<Frame> axleDemand, const AxleWheel<Frame>& left,
                                      const AxleWheel<Frame>& right) {
    auto leftForce = [&](float s) { return left.forceAt(s).v; };
    auto rightForce = [&](float s) { return right.forceAt(s).v; };
    float direction = axleDemand.v >= 0 ? 1.0f : -1.0f;

    // one peak search per wheel, reused as the wheel capacity; clamped at 0
    BranchPeak leftPeak = ascendingBranchPeak(leftForce, direction);
    BranchPeak rightPeak = ascendingBranchPeak(rightForce, direction);
    float leftCapacity = std::max(0.0f, direction * leftPeak.force);
    float rightCapacity = std::max(0.0f, direction * rightPeak.force);

    float deliverable = std::min({std::abs(axleDemand.v) * 0.5f, leftCapacity, rightCapacity});
    float force = std::copysign(deliverable, axleDemand.v);
    return {ascendingBranchSlipRatio(leftForce, force, leftPeak, direction),
            ascendingBranchSlipRatio(rightForce, force, rightPeak, direction)};
}

template <typename Frame>
class DifferentialBase {
   public:
    virtual ~DifferentialBase() = default;
    virtual AxleSlipRatios solve(X<Frame> axleDemand, const AxleWheel<Frame>& left,
                                 const AxleWheel<Frame>& right) const = 0;
};

template <typename Frame>
class OpenDifferential : public DifferentialBase<Frame> {
   public:
    AxleSlipRatios solve(X<Frame> axleDemand, const AxleWheel<Frame>& left,
                         const AxleWheel<Frame>& right) const override {
        return equalForceSolve(axleDemand, left, right);
    }
};
