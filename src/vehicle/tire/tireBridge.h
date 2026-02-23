#pragma once

#include "coordTypes.h"
#include "vehicle/tire/tire.h"

template <typename Frame>
struct TireCallResult {
    X<Frame> Fx;
    Y<Frame> Fy;
    Z<Frame> Mz;
};

template <typename From, typename To>
TireCallResult<From> callTire(const FrameBridge<From, To>& bridge, Tire& tire, float load,
                              Alpha<From> slip, float slipRatio = 0.f) {
    tire.calculate(load, bridge.toTarget(slip), slipRatio);
    auto o = tire.getOutput();
    return {bridge.fromTarget(o.Fx), bridge.fromTarget(o.Fy), bridge.fromTarget(o.Mz)};
}
